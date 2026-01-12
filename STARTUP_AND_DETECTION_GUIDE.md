# rpi-connect Startup & Detection Control Guide

## Quick Start

### Option 1: Manual Start (Recommended for Testing)

```bash
# On Raspberry Pi
cd /home/pi/rpi-connect

# Activate virtual environment
source venv/bin/activate

# Start the controller
python3 pi_controller.py
```

Expected output:
```
🚀 Starting Raspberry Pi Controller...
📡 Connecting to server: http://10.141.104.94:3000
✅ Connected to server!
📹 Camera initialized
✅ Pixhawk telemetry connected
📍 GeoLocation Calculator initialized
🌾 Yellow Crop Detector initialized
📡 MAVLink Detection Sender initialized
✅ System ready!
```

---

### Option 2: System Service (Automatic on Boot)

```bash
# Install as service (one-time setup)
cd /home/pi/rpi-connect
./setup.sh

# Service commands
sudo systemctl start pi-controller      # Start now
sudo systemctl stop pi-controller       # Stop
sudo systemctl restart pi-controller    # Restart
sudo systemctl enable pi-controller     # Enable on boot
sudo systemctl status pi-controller     # Check status

# View live logs
sudo journalctl -u pi-controller -f
```

---

## Detection Control Methods

### Method 1: From GCS Dashboard (via WiFi/LTE)

**Start Detection:**
```javascript
// From your GCS web interface
socket.emit('start_detection', {
  mission_id: 'mission_001',  // Optional: auto-generated if not provided
  pi_id: 'detection_drone_pi_pushpak'
});
```

**Stop Detection:**
```javascript
socket.emit('stop_detection', {
  pi_id: 'detection_drone_pi_pushpak'
});
```

**What Happens:**
1. ✅ Auto-starts mission if not active
2. ✅ Auto-starts camera stream
3. ✅ Activates yellow plant detection
4. ✅ Sends detections via Socket.IO + MAVLink

---

### Method 2: From Mission Planner / QGroundControl (Long-Range)

**Via MAVLink Commands:**

#### Using Mission Planner:
1. Connect to drone via telemetry radio
2. Go to **Flight Data** → **Actions** tab
3. Click **"Scripts"** or **"Commands"**
4. Send custom COMMAND_LONG with:
   - Command: `MAV_CMD_USER_1` (31010)
   - Param1: 1 (start) or 0 (stop)

#### Using MAVProxy:
```bash
# Start detection
mavproxy.py --master=/dev/ttyUSB0 --baudrate=57600
> command long 0 0 31010 0 1 0 0 0 0 0 0

# Stop detection
> command long 0 0 31010 0 0 0 0 0 0 0 0
```

**Note:** This method requires `mavlink_command_receiver` to be enabled (currently disabled in config to avoid conflicts).

---

### Method 3: Automatic Detection in AUTO Mode

**In config.json:**
```json
"detection": {
  "auto_detect_in_auto_mode": true,  // ← Enable this
}
```

**What Happens:**
- Detection automatically starts when drone enters **AUTO mode**
- Detection stops when drone exits AUTO mode (LOITER, RTL, LAND)
- No manual command needed!

**Best for:** Competition scenarios where detection should happen during mission only.

---

### Method 4: Manual Python Script (Testing)

```bash
# On Raspberry Pi
cd /home/pi/rpi-connect
source venv/bin/activate
python3 << EOF
from pi_controller import controller
import time

# Start detection
controller.detection_active = True
print("Detection started!")

# Wait and let it run
time.sleep(60)

# Stop detection
controller.detection_active = False
print("Detection stopped!")
EOF
```

---

## Complete Startup & Flight Workflow

### Pre-Flight Setup

1. **Power on Raspberry Pi** (connects to Pixhawk via TELEM2)
2. **Wait for boot** (~30 seconds)
3. **Connect GCS to drone** (via TELEM1 telemetry radio or WiFi)

```bash
# Check if Pi controller is running
sudo systemctl status pi-controller

# If not running, start it
sudo systemctl start pi-controller
```

---

### Competition Flight Sequence

#### With WiFi/LTE Connection:

**From GCS Dashboard:**

1. **Arm drone** → Controller detects drone armed
2. **Start mission** (send to drone via Mission Planner/QGC)
3. **Start detection:**
   ```javascript
   socket.emit('start_detection', {
     mission_id: 'competition_mission_001'
   });
   ```
4. **Monitor detections in real-time** on dashboard map
5. **After mission completes:**
   ```javascript
   socket.emit('stop_detection', {});
   ```

---

#### Long-Range (No WiFi, MAVLink Only):

**Auto-Detection Method (Recommended):**

1. **Set in config.json:**
   ```json
   "detection": {
     "auto_detect_in_auto_mode": true,
   }
   ```

2. **Arm drone** via Mission Planner
3. **Upload mission** (lawn mower pattern from kml_mission_planner.py)
4. **Switch to AUTO mode** → Detection automatically starts!
5. **Drone executes mission** → Detections sent via MAVLink STATUSTEXT
6. **After RTL/Landing** → Detection automatically stops

**Monitor detections:**
- Mission Planner → **Messages** tab
- Look for: `DET|mission_001_0001|40.712800|-74.006000|0.95|1732`

---

#### Manual MAVLink Control (Advanced):

Enable MAVLink command receiver first:

**Edit config.json:**
```json
"mavlink_command_receiver": {
  "enabled": true,  // Change from false
}
```

**Restart controller:**
```bash
sudo systemctl restart pi-controller
```

**Send commands via Mission Planner/MAVProxy:**
```
# Start detection: COMMAND_LONG with CMD_USER_1, param1=1
command long 0 0 31010 0 1 0 0 0 0 0 0

# Stop detection: COMMAND_LONG with CMD_USER_1, param1=0
command long 0 0 31010 0 0 0 0 0 0 0 0
```

---

## Checking System Status

### On Raspberry Pi:

```bash
# Check if controller is running
sudo systemctl status pi-controller

# View recent logs
sudo journalctl -u pi-controller -n 50

# View live logs (real-time)
sudo journalctl -u pi-controller -f

# Check for detection activity
sudo journalctl -u pi-controller -f | grep -E "Detection|🌾|DET"

# Check for MAVLink messages
sudo journalctl -u pi-controller -f | grep -E "MAVLink|📡"
```

### From GCS Dashboard:

**Check connection:**
```javascript
// Request Pi status
socket.emit('get_status', { pi_id: 'detection_drone_pi_pushpak' });

// Response includes:
{
  connected: true,
  detection_active: false,
  mission_active: false,
  telemetry: { ... },
  camera_active: true
}
```

---

## Troubleshooting

### Detection Not Starting:

**Check 1: Controller Running?**
```bash
sudo systemctl status pi-controller
# If not: sudo systemctl start pi-controller
```

**Check 2: Camera Working?**
```bash
cd /home/pi/rpi-connect
source venv/bin/activate
python3 test_bottom_camera.py
```

**Check 3: Detector Initialized?**
```bash
sudo journalctl -u pi-controller | grep "Yellow Crop Detector"
# Should see: "✅ Yellow Crop Detector initialized"
```

**Check 4: Socket.IO Connected?**
```bash
sudo journalctl -u pi-controller | grep "Connected to server"
# Should see: "✅ Connected to server!"
```

---

### Detection Not Sending to GCS:

**Check MAVLink Connection:**
```bash
# On Pi, check logs
sudo journalctl -u pi-controller | grep "MAVLink"

# Should see:
# ✅ MAVLink Detection Sender initialized
# 📡 MAVLink: Sent detection XXXX via telemetry
```

**Check read_only mode:**
```bash
cat /home/pi/rpi-connect/config.json | grep read_only
# Should be: "read_only": false
```

**Test MAVLink directly:**
```bash
cd /home/pi/rpi-connect
source venv/bin/activate
python3 modules/mavlink_detection_sender.py
```

---

### Auto-Detection in AUTO Mode Not Working:

**Check config setting:**
```bash
cat config.json | grep auto_detect_in_auto_mode
# Should be: "auto_detect_in_auto_mode": true,
```

**Check drone mode detection:**
```bash
sudo journalctl -u pi-controller -f | grep "Mode:"
# Should show mode changes when you switch modes
```

---

## Detection Data Flow

```
START DETECTION COMMAND
        ↓
[Pi Controller]
        ↓
controller.detection_active = True
        ↓
Camera captures frame
        ↓
[Yellow Crop Detector]
        ↓
HSV filtering → Find yellow regions
        ↓
Contour detection → Filter by size/confidence
        ↓
For each detection:
  1. Calculate pixel centroid
  2. Get drone telemetry (GPS, heading, altitude)
  3. [GeoLocation Calculator] → Convert pixel to GPS
  4. Assemble detection data
        ↓
        ├── [Socket.IO] → GCS Dashboard (if WiFi connected)
        │       ↓
        │   Real-time map update
        │
        └── [MAVLink STATUSTEXT] → Pixhawk TELEM2
                ↓
            Pixhawk forwards to TELEM1
                ↓
            Long-range radio to GCS
                ↓
            Mission Planner/QGC displays message
```

---

## Recommended Configuration for Competition

**Edit `/home/pi/rpi-connect/config.json`:**

```json
{
  "detection": {
    "enabled": true,
    "yellow_hsv_lower": [20, 80, 60],
    "yellow_hsv_upper": [32, 255, 255],
    "min_contour_area": 150,
    "confidence_threshold": 0.50,
    "auto_detect_in_auto_mode": true,  // ← Enable for autonomous detection
    "save_detection_images": true,
    "debug_mode": false
  },
  "pixhawk": {
    "enabled": true,
    "read_only": false,  // ← Must be false for MAVLink transmission
  },
  "mavlink_detection": {
    "enabled": true,  // ← Enable long-range detection transmission
    "send_metadata": true
  }
}
```

Then restart:
```bash
sudo systemctl restart pi-controller
```

---

## Quick Command Reference

```bash
# Start system
sudo systemctl start pi-controller

# View logs
sudo journalctl -u pi-controller -f

# Manual detection control (from GCS dashboard)
socket.emit('start_detection', { mission_id: 'mission_001' });
socket.emit('stop_detection', {});

# Check detection status
grep "Detection" /var/log/syslog

# Test detection directly
cd /home/pi/rpi-connect && python3 test_yellow_detection.py
```

---

## Summary

**Easiest Method (Recommended for Competition):**

1. ✅ Set `auto_detect_in_auto_mode: true` in config.json
2. ✅ Set `read_only: false` in pixhawk config
3. ✅ Start pi_controller: `sudo systemctl start pi-controller`
4. ✅ Arm drone and switch to AUTO mode
5. ✅ Detection starts automatically and sends via MAVLink!

No manual commands needed during flight!
