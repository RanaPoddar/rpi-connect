# Detection Transmission Architecture

## Overview
Your drone detection system uses a **dual-channel hybrid transmission** architecture for maximum reliability in the field.

## CRITICAL CONCEPT: Two Independent Connections

### The Pi has TWO separate connections:

1. **WiFi Connection (Optional):**
   - Pi ↔ WiFi Hotspot ↔ GCS Server
   - **Range Limited:** ~50-100m
   - **Disconnects when out of range** ❌

2. **Serial Connection (Always Active):**
   - Pi ↔ Serial Cable (TELEM2) ↔ Pixhawk
   - **Local connection - NOT affected by WiFi range** ✅
   - **Always works** as long as Pi is powered ✅

## What Happens When Out of WiFi Range?

```
BEFORE (In WiFi Range):
┌─────────────────────────────────────────────────┐
│ Pi Detection System                              │
│   ↓ (both channels active)                      │
│   ├─→ Socket.IO → WiFi → GCS Server ✓          │
│   └─→ Serial → Pixhawk → Radio → GCS ✓         │
└─────────────────────────────────────────────────┘

AFTER (Out of WiFi Range):
┌─────────────────────────────────────────────────┐
│ Pi Detection System                              │
│   ↓                                              │
│   ├─→ Socket.IO → WiFi → GCS Server ✗ FAILS    │
│   └─→ Serial → Pixhawk → Radio → GCS ✓ WORKS!  │
└─────────────────────────────────────────────────┘
```

**Key Point:** The Pi doesn't need WiFi to send data to Pixhawk! The serial cable (TELEM2) is a direct physical connection that works regardless of WiFi status.

## Detection Data Flow

### In WiFi Range:
```
Pi Camera → Detection → BOTH channels:
  1. Socket.IO → WiFi → GCS Server (fast) ✓
  2. Serial → Pixhawk → Radio → GCS (backup) ✓
```

### Out of WiFi Range (Field Operation):
```
Pi Camera → Detection → MAVLink only:
  Pi (disconnected from WiFi but still running)
   ↓
  Serial Cable (TELEM2) - LOCAL CONNECTION
   ↓
  Pixhawk Autopilot (receives detection data)
   ↓
  TELEM1 Port (Pixhawk forwards data)
   ↓
  Radio Transmitter (915MHz/433MHz)
   ↓
  ~~ AIR TRANSMISSION ~~
   ↓
  Radio Receiver @ GCS
   ↓
  PyMAVLink Service (parses detection messages)
   ↓
  Node.js Server
   ↓
  Dashboard ✓ DETECTION SHOWN!
```

## How the Pi Stays Independent

### Pi's Operation When WiFi Drops:

1. **Detection System Continues Running**
   - Camera keeps capturing
   - Yellow crop detection keeps processing
   - GPS coordinates keep calculating

2. **Socket.IO Disconnects (Expected)**
   - `sio.connected` returns `False`
   - Code gracefully handles this: `if sio.connected: ... else: skip`
   - No crash, no error - just skips WiFi channel

3. **MAVLink Keeps Working**
   - Pi → Pixhawk connection is **LOCAL SERIAL** (not WiFi-dependent)
   - `self.mavlink_detection_sender.send_detection()` still works
   - Pixhawk receives detection data
   - Pixhawk forwards to GCS via its own radio link

4. **Code Behavior:**
```python
# From pi_controller.py line 505-520:

# Try Socket.IO first
socketio_sent = False
if sio.connected:  # This will be False out of range
    try:
        sio.emit('crop_detection', detection_data)
        socketio_sent = True
    except:
        pass  # Failed, but MAVLink will handle it

# ALWAYS try MAVLink (independent of WiFi)
mavlink_sent = False
if self.mavlink_detection_sender and self.mavlink_detection_sender.enabled:
    mavlink_sent = self.mavlink_detection_sender.send_detection(detection_data)
    # This works because it uses serial connection to Pixhawk!

# Result when out of WiFi:
# socketio_sent = False
# mavlink_sent = True ✓
```

## Why This Works:

### Physical Connections on the Drone:

```
Raspberry Pi (on drone)
  │
  ├─── WiFi Antenna ──→ Hotspot (range limited) ──→ GCS Server
  │                     ⚠️ BREAKS when out of range
  │
  └─── GPIO Serial Pins ──→ TELEM2 on Pixhawk
                           ✅ ALWAYS CONNECTED (wired)
                           
Pixhawk (on drone)
  │
  ├─── TELEM2 ←── Pi (receives detection data)
  │
  └─── TELEM1 ──→ Radio Module ──→ ~~ AIR ~~
                                   ✅ Long Range (km)
                                   
                                   ──→ GCS Radio ──→ GCS Computer
```

### The Magic:
- **Pi sends to Pixhawk via wire** (TELEM2) - no WiFi needed
- **Pixhawk forwards to GCS via radio** (TELEM1) - long range
- **Even though Pi lost WiFi to GCS, it still has wired connection to Pixhawk!**

## Configuration

### Pi Configuration (config.json)
```json
{
  "pixhawk": {
    "enabled": true,
    "connection_string": "/dev/serial0",
    "baud_rate": 921600,
    "read_only": false  // MUST BE FALSE to send detections
  },
  "mavlink_detection": {
    "enabled": true,     // Enable MAVLink transmission
    "send_metadata": true
  }
}
```

### Detection Message Format

#### Socket.IO (WiFi):
```json
{
  "detection_id": "det_20260112_123456_001",
  "timestamp": 1704825600.123,
  "latitude": 12.971234,
  "longitude": 77.594567,
  "altitude": 25.5,
  "confidence": 0.85,
  "detection_area": 1250,
  "bounding_box": {"x": 100, "y": 150, "width": 80, "height": 60},
  "drone_mode": "AUTO",
  "heading": 45.2,
  "ground_speed": 3.5
}
```

#### MAVLink (Radio):
Uses STATUSTEXT messages with prefix `DET:`
```
DET:det_20260112_123456_001,12.971234,77.594567,0.85
```

Format: `DET:detection_id,latitude,longitude,confidence`

## GCS Reception

### Updated Components:

1. **pymavlink_service.py** ✅ UPDATED
   - Parses `DET:` prefixed STATUSTEXT messages
   - Extracts detection coordinates
   - Forwards to Node.js server

2. **server.js** ✅ UPDATED
   - Receives detection via POST `/api/mavlink-detection`
   - Emits to dashboard via Socket.IO event `mavlink_detection`

3. **index.html** ✅ UPDATED
   - Listens for `mavlink_detection` Socket.IO events
   - Button to test MAVLink detection reception
   - Visual feedback on detection status

## Testing the System

### 1. WiFi Testing
```bash
# On Pi, start detection
# Detections will appear on dashboard via Socket.IO
```

### 2. MAVLink Testing
```bash
# On dashboard, click "📡 Check MAVLink Telemetry"
# Listens for 10 seconds
# Shows:
#   ✅ SUCCESS if detections received via radio
#   ❌ FAILED if no detections (check config)
```

### 3. Field Testing
```bash
# 1. Start mission with WiFi
# 2. Fly out of WiFi range
# 3. Detections automatically switch to MAVLink radio
# 4. Verify on dashboard using "Check MAVLink Telemetry"
```

## Troubleshooting

### No Detections via MAVLink:

**Check Pi:**
- ✅ `mavlink_detection.enabled: true` in config.json
- ✅ `pixhawk.read_only: false` (must allow Pi to send)
- ✅ Pixhawk connected to Pi via TELEM2
- ✅ Detection system running (`start_detection` called)

**Check GCS:**
- ✅ Radio connected to GCS computer
- ✅ PyMAVLink service running (port 5000)
- ✅ Node.js server running (port 3000)
- ✅ Radio receiving telemetry from drone

**Check Radio Link:**
- ✅ Drone radio connected to TELEM1 on Pixhawk
- ✅ GCS radio connected to computer (USB/Serial)
- ✅ Radio frequencies match (915MHz or 433MHz)
- ✅ Radio power sufficient for range

### Verify Detection Flow:

```bash
# On Pi, check logs
tail -f /path/to/pi_controller.log
# Look for: "📡 MAVLink: Detection sent"

# On GCS PyMAVLink, check logs
# Look for: "📡 Drone X MAVLink Detection: ..."

# On GCS Node.js, check logs
# Look for: "📡 MAVLink detection received: ..."

# On Dashboard
# Click "📡 Check MAVLink Telemetry"
```

## Bandwidth Considerations

### Socket.IO (WiFi):
- Full metadata: ~500 bytes per detection
- No images sent (optimized)
- Fast, real-time

### MAVLink (Radio):
- Compressed format: ~50 bytes per detection
- Critical data only: ID, lat, lon, confidence
- Reliable over long range
- Radio bandwidth: typically 9600-57600 baud

## System Requirements

### Hardware:
- ✅ Raspberry Pi with camera
- ✅ Pixhawk flight controller
- ✅ TELEM2 connected to Pi (high-speed serial)
- ✅ TELEM1 with radio telemetry (SiK radio or similar)
- ✅ GCS radio receiver

### Software:
- ✅ Pi: Python 3.7+, pymavlink, picamera2, cv2
- ✅ GCS: Python 3.7+ (PyMAVLink service)
- ✅ GCS: Node.js 16+ (server)
- ✅ Browser: Modern browser for dashboard

## Summary

Your system is now **fully configured** for dual-channel detection transmission:

1. **In WiFi Range:** Fast Socket.IO transmission
2. **Out of WiFi Range:** Automatic MAVLink radio fallback
3. **Dashboard:** Can monitor both channels
4. **Testing:** Built-in MAVLink detection test button

The detection data will **always reach the GCS**, regardless of WiFi availability! 🚁📡✅

## Real-World Field Operation Scenario

### Typical Mission Flow:

**Phase 1: Launch (In WiFi Range)**
```
- Drone takes off near GCS hotspot
- Pi connects via WiFi to GCS server
- Both channels active (WiFi + MAVLink)
- Dashboard shows real-time detections via Socket.IO
```

**Phase 2: Flying Away (Leaving WiFi Range)**
```
- Drone flies 100m+ away from hotspot
- WiFi signal weakens... Socket.IO disconnects
- Pi console: "Socket.IO disconnected" (normal!)
- Detection system KEEPS RUNNING
- MAVLink channel STILL ACTIVE (serial connection)
- Detections sent via: Pi → Pixhawk → Radio → GCS
- Dashboard shows detections via MAVLink (slight delay, but reliable!)
```

**Phase 3: Field Survey (No WiFi)**
```
- Drone flying 500m-2km from GCS
- No WiFi connection at all
- Pi running autonomously
- Camera detecting yellow crops
- All detections flowing via MAVLink radio
- GCS dashboard updating with detection coordinates
- Click "Check MAVLink Telemetry" button: ✅ SUCCESS!
```

**Phase 4: Return Home (Re-entering WiFi Range)**
```
- Drone flies back toward GCS
- Enters WiFi range
- Socket.IO automatically reconnects
- Both channels active again
- Seamless transition back to fast WiFi transmission
```

### Key Takeaway:
**The Pi's disconnection from WiFi is EXPECTED and NORMAL during field operations. The MAVLink channel is specifically designed to work independently when WiFi is unavailable.** The Pi continues detecting, the Pixhawk continues forwarding, and the GCS continues receiving - all without WiFi! 🚁📡
