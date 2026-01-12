# Quick Reference: Detection Transmission System

## Two Independent Channels

| Channel | Connection | Range | Out of WiFi Range? |
|---------|-----------|-------|-------------------|
| **WiFi (Socket.IO)** | Pi → Hotspot → Server | ~100m | ❌ DISCONNECTS |
| **MAVLink (Radio)** | Pi → Pixhawk → Radio → GCS | ~2km+ | ✅ KEEPS WORKING |

## Common Questions

### Q: "Will detections reach GCS when out of WiFi range?"
**A: YES!** Via MAVLink radio through Pixhawk.

### Q: "Does the Pi need WiFi to send detections?"
**A: NO!** Pi has wired connection to Pixhawk (TELEM2).

### Q: "What happens when WiFi disconnects?"
**A: Socket.IO fails, MAVLink takes over. Detections continue flowing.**

### Q: "How does Pi send to GCS without WiFi?"
**A: Pi → Serial Cable → Pixhawk → Radio → GCS**

### Q: "Will detection stop working out of range?"
**A: NO! Detection keeps running, data sent via radio.**

## System Status in Different Ranges

### 0-100m (In WiFi Range):
- ✅ Socket.IO active (fast)
- ✅ MAVLink active (backup)
- Dashboard: Real-time updates

### 100m-2km (Out of WiFi):
- ❌ Socket.IO disconnected
- ✅ MAVLink active (primary)
- Dashboard: Updates via radio (2-5 sec delay)

### 2km+ (Out of Radio Range):
- ❌ Socket.IO disconnected
- ❌ MAVLink signal weak
- Pi: Stores detections locally
- Dashboard: No updates until drone returns

## Verification Commands

### Test MAVLink Detection:
```
1. Open GCS Dashboard (index.html)
2. Click "📡 Check MAVLink Telemetry"
3. Wait 10 seconds
4. See result: ✅ SUCCESS or ❌ FAILED
```

### Check Pi Logs:
```bash
# On Pi
tail -f ~/pi_controller.log | grep "MAVLink"
# Should see: "📡 MAVLink: Detection sent"
```

### Check GCS Logs:
```bash
# PyMAVLink Service
# Should see: "📡 Drone X MAVLink Detection: ..."

# Node.js Server
# Should see: "📡 MAVLink detection received: ..."
```

## Troubleshooting Decision Tree

```
Detection not reaching GCS?
│
├─ In WiFi range?
│  ├─ Yes → Check Socket.IO connection
│  │         Dashboard shows "Connected"?
│  │         ├─ Yes → Check Pi detection enabled
│  │         └─ No → Restart server
│  │
│  └─ No → Expected! Check MAVLink
│            Click "Check MAVLink Telemetry"
│            ├─ Success → System working correctly
│            └─ Failed → Check below
│
└─ MAVLink Failed?
   ├─ Check Pi config.json:
   │  - mavlink_detection.enabled = true
   │  - pixhawk.read_only = false
   │
   ├─ Check Radio Link:
   │  - Drone radio connected to TELEM1
   │  - GCS radio connected to computer
   │  - Radios paired and transmitting
   │
   └─ Check PyMAVLink Service:
      - Running on port 5000
      - Connected to drone
      - Receiving telemetry
```

## File Locations

| Component | File | Purpose |
|-----------|------|---------|
| Pi Detection | `pi_controller.py` | Main detection system |
| Pi MAVLink Sender | `modules/mavlink_detection_sender.py` | Sends to Pixhawk |
| GCS MAVLink Parser | `external-services/pymavlink_service.py` | Parses radio messages |
| GCS Server | `server.js` | Forwards to dashboard |
| Dashboard | `public/index.html` | Shows detections |

## Configuration Checklist

### Pi (rpi-connect/config.json):
- [ ] `pixhawk.enabled = true`
- [ ] `pixhawk.read_only = false`
- [ ] `pixhawk.connection_string = "/dev/serial0"`
- [ ] `pixhawk.baud_rate = 921600`
- [ ] `mavlink_detection.enabled = true`

### GCS:
- [ ] PyMAVLink service running (port 5000)
- [ ] Node.js server running (port 3000)
- [ ] Radio connected to computer
- [ ] Radio paired with drone radio

## Expected Behavior

### Normal Operation:
```
Pi Log:
  "✅ Detection 001 sent via BOTH channels"
  "📡 MAVLink: Detection sent"

GCS PyMAVLink:
  "📡 Drone 1 MAVLink Detection: det_xxx at (12.97, 77.59)"

GCS Server:
  "📡 MAVLink detection received: det_xxx"

Dashboard:
  Green marker appears on detection list
  "📡 MAVLink detection: det_xxx at (12.97, 77.59)"
```

### Out of WiFi (Normal):
```
Pi Log:
  "Socket.IO disconnected"
  "⚠️ Detection sent via MAVLink only (Socket.IO unavailable)"
  "📡 MAVLink: Detection sent"

Dashboard:
  Still shows detections!
  Via MAVLink channel (slight delay)
```

## Key Takeaway

**The system is designed to work WITHOUT WiFi in the field!**
- WiFi = Fast but short range
- MAVLink Radio = Slower but long range
- Both channels ensure you never miss a detection! 🎯
