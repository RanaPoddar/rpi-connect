# Out-of-Range Detection & Image Transmission - Solution

## Problem
When the drone flies out of WiFi range, the GCS dashboard stops receiving:
- Yellow crop detections
- Periodic mission images
- Real-time updates

The Raspberry Pi sends data via Socket.IO (WiFi), which fails when out of range.

## Root Cause
1. **Pi Controller**: Sends detections/images primarily via Socket.IO (`sio.emit`)
2. **MAVLink Fallback Incomplete**: While MAVLink detection sender exists, periodic images are NOT sent over MAVLink
3. **GCS Missing MAVLink Receiver**: GCS doesn't listen for MAVLink STATUSTEXT messages containing detections/images

## Current Data Flow

### When IN WiFi Range (Working):
```
Pi Camera → Detection → Socket.IO (WiFi) → GCS Server → Dashboard
Pi Camera → Periodic Image → Socket.IO (WiFi) → GCS Server → Dashboard
```

### When OUT of WiFi Range (Broken):
```
Pi Camera → Detection → MAVLink (Radio) → [GCS NOT LISTENING] ❌
Pi Camera → Periodic Image → [NOT SENT AT ALL] ❌
```

## Solution: Complete MAVLink Fallback System

### Phase 1: Fix Pi Controller (rpi-connect/pi_controller.py)

#### A. Send Periodic Images Over MAVLink
Current code (line ~940):
```python
if self.periodic_capture_config.get('send_to_server', True) and sio.connected:
    sio.emit('periodic_image', image_packet)
```

**Fix**: Always try MAVLink when Socket.IO fails
```python
# Try Socket.IO first
socketio_sent = False
if self.periodic_capture_config.get('send_to_server', True) and sio.connected:
    try:
        sio.emit('periodic_image', image_packet)
        socketio_sent = True
    except Exception as e:
        print(f"⚠️  Socket.IO periodic image send failed: {e}")

# Fallback to MAVLink if Socket.IO failed or unavailable
if not socketio_sent and self.mavlink_detection_sender:
    try:
        # Send compressed image metadata over MAVLink
        # Note: Full images too large, send metadata + store locally
        metadata = {
            'image_id': image_id,
            'mission_id': self.current_mission_id,
            'timestamp': timestamp,
            'latitude': telemetry.get('latitude', 0.0),
            'longitude': telemetry.get('longitude', 0.0),
            'altitude': telemetry.get('altitude', 0.0),
            'image_type': 'periodic',
            'stored_locally': True
        }
        self.mavlink_detection_sender.send_image_metadata(metadata)
    except Exception as e:
        print(f"⚠️  MAVLink image metadata send failed: {e}")
```

#### B. Add Image Metadata Sender to MAVLink Detection Sender
Add method to `modules/mavlink_detection_sender.py`:
```python
def send_image_metadata(self, metadata: Dict) -> bool:
    """Send image capture metadata over MAVLink (image stored locally)"""
    if not self.enabled or not self.master:
        return False
    try:
        # Format: IMG|ID|LAT|LON|TYPE|MISSION
        img_id = metadata.get('image_id', 'unknown')[:15]
        lat = metadata.get('latitude', 0.0)
        lon = metadata.get('longitude', 0.0)
        img_type = metadata.get('image_type', 'periodic')[:8]
        mission = metadata.get('mission_id', 'none')[:10]
        
        message = f"IMG|{img_id}|{lat:.6f}|{lon:.6f}|{img_type}|{mission}"
        self._send_statustext(message, self.SEVERITY_INFO)
        return True
    except Exception as e:
        print(f"❌ MAVLink image metadata send failed: {e}")
        return False
```

### Phase 2: Add GCS MAVLink Listener

#### A. Create MAVLink Message Listener Module
File: `GCS-without-pi/services/mavlinkMessageListener.js`

```javascript
/**
 * MAVLink STATUSTEXT Message Listener
 * Listens for detection and image metadata from Pi over long-range radio
 * Converts MAVLink messages back to Socket.IO events for dashboard
 */

const logger = require('../config/logger');
const axios = require('axios');

const PYMAVLINK_SERVICE_URL = process.env.PYMAVLINK_URL || 'http://localhost:5000';

class MAVLinkMessageListener {
  constructor(io) {
    this.io = io;
    this.listening = false;
    this.pollInterval = null;
  }

  /**
   * Start listening for MAVLink messages from all drones
   */
  start() {
    if (this.listening) return;
    
    this.listening = true;
    this.pollInterval = setInterval(() => this.pollMessages(), 500);
    logger.info('📡 MAVLink message listener started');
  }

  /**
   * Stop listening
   */
  stop() {
    if (this.pollInterval) {
      clearInterval(this.pollInterval);
      this.pollInterval = null;
    }
    this.listening = false;
    logger.info('📡 MAVLink message listener stopped');
  }

  /**
   * Poll for new STATUSTEXT messages
   */
  async pollMessages() {
    try {
      // Query PyMAVLink service for recent STATUSTEXT messages
      const response = await axios.get(`${PYMAVLINK_SERVICE_URL}/messages/statustext`);
      
      if (response.data && response.data.messages) {
        for (const msg of response.data.messages) {
          this.parseMessage(msg.text, msg.drone_id);
        }
      }
    } catch (error) {
      // Silent fail - service might not be running yet
    }
  }

  /**
   * Parse MAVLink STATUSTEXT message
   */
  parseMessage(text, droneId) {
    try {
      // Detection message: DET|ID|LAT|LON|CONF|AREA
      if (text.startsWith('DET|')) {
        const parts = text.split('|');
        if (parts.length >= 6) {
          const detection = {
            detection_id: parts[1],
            latitude: parseFloat(parts[2]),
            longitude: parseFloat(parts[3]),
            confidence: parseFloat(parts[4]),
            detection_area: parseInt(parts[5]),
            pi_id: `drone_${droneId}_pi`,
            source: 'mavlink',
            timestamp: new Date().toISOString()
          };
          
          // Emit to dashboard
          this.io.emit('crop_detection', detection);
          logger.info(`📡 MAVLink Detection received from Drone ${droneId}`);
        }
      }
      
      // Image metadata: IMG|ID|LAT|LON|TYPE|MISSION
      else if (text.startsWith('IMG|')) {
        const parts = text.split('|');
        if (parts.length >= 6) {
          const imageMetadata = {
            image_id: parts[1],
            latitude: parseFloat(parts[2]),
            longitude: parseFloat(parts[3]),
            image_type: parts[4],
            mission_id: parts[5],
            pi_id: `drone_${droneId}_pi`,
            source: 'mavlink',
            stored_locally: true,
            timestamp: new Date().toISOString()
          };
          
          // Emit to dashboard (notify image was captured)
          this.io.emit('periodic_image_metadata', imageMetadata);
          logger.info(`📡 MAVLink Image metadata received from Drone ${droneId}`);
        }
      }
      
      // Detection stats: DSTAT|TOTAL|ACTIVE|MISSION
      else if (text.startsWith('DSTAT|')) {
        const parts = text.split('|');
        if (parts.length >= 4) {
          const stats = {
            total_detections: parseInt(parts[1]),
            detection_active: Boolean(parseInt(parts[2])),
            mission_id: parts[3],
            source: 'mavlink',
            timestamp: new Date().toISOString()
          };
          
          this.io.emit('detection_stats', stats);
        }
      }
    } catch (error) {
      logger.error(`Error parsing MAVLink message: ${error.message}`);
    }
  }
}

module.exports = MAVLinkMessageListener;
```

#### B. Integrate into GCS Server
Edit `GCS-without-pi/server.js`:
```javascript
// Add after other imports
const MAVLinkMessageListener = require('./services/mavlinkMessageListener');

// Add after setupSocketHandlers(io)
const mavlinkListener = new MAVLinkMessageListener(io);
mavlinkListener.start();
```

#### C. Update Dashboard to Handle MAVLink Image Metadata
Edit `GCS-without-pi/public/mission_control.js`:
```javascript
// Add new socket listener
this.socket.on('periodic_image_metadata', (data) => {
    // Show notification that image was captured (stored on Pi)
    this.addAlert(`📷 Image captured: ${data.image_id} (stored on Pi, sync after landing)`, 'info');
    
    // Optionally show marker on map where image was taken
    if (data.latitude && data.longitude) {
        const marker = L.circleMarker([data.latitude, data.longitude], {
            radius: 3,
            color: '#3b82f6',
            fillColor: '#60a5fa',
            fillOpacity: 0.5
        }).addTo(this.map);
        
        marker.bindPopup(`📷 ${data.image_id}<br>Type: ${data.image_type}`);
    }
});
```

### Phase 3: Update PyMAVLink Service (If Needed)

Add endpoint to retrieve STATUSTEXT messages:
File: `GCS-without-pi/external-services/pymavlink_service.py`

```python
# Add to message buffer
statustext_buffer = []
MAX_BUFFER_SIZE = 100

# Add message listener in telemetry loop
@app.route('/messages/statustext', methods=['GET'])
def get_statustext_messages():
    """Get recent STATUSTEXT messages"""
    global statustext_buffer
    messages = statustext_buffer.copy()
    statustext_buffer.clear()  # Clear after reading
    return jsonify({'messages': messages})

# In telemetry collection, capture STATUSTEXT
def collect_statustext(vehicle, drone_id):
    """Collect STATUSTEXT messages"""
    msg = vehicle.recv_match(type='STATUSTEXT', blocking=False)
    if msg:
        statustext_buffer.append({
            'drone_id': drone_id,
            'text': msg.text,
            'severity': msg.severity,
            'timestamp': time.time()
        })
        # Trim buffer
        if len(statustext_buffer) > MAX_BUFFER_SIZE:
            statustext_buffer.pop(0)
```

## Implementation Priority

### Critical (Do First):
1. ✅ Fix periodic image sending in `pi_controller.py` to use MAVLink fallback
2. ✅ Add `send_image_metadata()` to `mavlink_detection_sender.py`
3. ⚠️  Create MAVLink listener service for GCS
4. ⚠️  Update PyMAVLink service to buffer STATUSTEXT

### Important (Do Second):
5. Update dashboard to show MAVLink-received data differently (e.g., "📡 Via Radio")
6. Add local image storage on Pi with post-landing sync
7. Test end-to-end with actual drone out of WiFi range

### Nice to Have:
8. Compress detection data further for MAVLink bandwidth
9. Add detection image thumbnails over MAVLink (very compressed)
10. Automatic image sync when drone returns to WiFi range

## Testing Plan

### Test 1: WiFi Disconnection Simulation
1. Start mission with WiFi connected
2. Disable WiFi on Pi (`sudo ifconfig wlan0 down`)
3. Verify detections still appear on GCS via MAVLink
4. Verify periodic image metadata appears
5. Re-enable WiFi (`sudo ifconfig wlan0 up`)
6. Verify automatic reconnection

### Test 2: Range Test
1. Fly drone progressively farther from GCS
2. Monitor when Socket.IO drops
3. Verify MAVLink continues working
4. Compare detection counts: sent vs received

### Test 3: Data Integrity
1. Compare detections sent via Socket.IO vs MAVLink
2. Verify GPS coordinates match
3. Check for message loss/duplication

## Configuration Changes

Add to `rpi-connect/config.json`:
```json
{
  "mavlink_detection": {
    "enabled": true,
    "send_metadata": true,
    "send_periodic_images_meta": true,
    "send_summary_interval": 30.0
  }
}
```

## Expected Behavior After Implementation

✅ **When in WiFi range**: Data sent via Socket.IO (primary, instant)
✅ **When out of WiFi range**: Data sent via MAVLink (fallback, reliable)
✅ **GCS Dashboard**: Shows all data regardless of transmission method
✅ **Periodic Images**: Metadata sent, full images stored locally on Pi
✅ **Post-Landing**: Automatic sync of stored images when WiFi reconnects

## Files to Modify

1. `rpi-connect/pi_controller.py` - Line ~940 (periodic capture)
2. `rpi-connect/modules/mavlink_detection_sender.py` - Add image metadata method
3. `GCS-without-pi/services/mavlinkMessageListener.js` - NEW FILE
4. `GCS-without-pi/server.js` - Integrate listener
5. `GCS-without-pi/public/mission_control.js` - Handle new events
6. `GCS-without-pi/external-services/pymavlink_service.py` - Add STATUSTEXT endpoint
7. `rpi-connect/config.json` - Update MAVLink config

## Notes

- Full periodic images (~50-200KB) are too large for MAVLink radio bandwidth
- Solution: Send metadata over MAVLink, store images locally, sync after landing
- Detection data (~50 bytes) fits perfectly in STATUSTEXT messages
- Consider image compression/thumbnails for critical captures
