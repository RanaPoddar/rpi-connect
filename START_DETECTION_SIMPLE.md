# 🎯 START HERE: Manual Detection Control on Pi

## Simplest Method (No modifications needed!)

Your Pi config already has:
```json
{
  "detection": {
    "enabled": true,  ← Already set!
    "auto_detect_in_auto_mode": true  ← Detection auto-starts in AUTO mode
  }
}
```

### Step 1: Start Pi Controller

```bash
cd /home/pi/rpi-connect
source venv/bin/activate
python3 pi_controller.py
```

Wait for:
```
✅ Pixhawk telemetry initialized
📍 GeoLocation Calculator initialized
🌾 Yellow Crop Detector initialized
📡 MAVLink Detection Sender initialized
✅ System ready!
```

### Step 2: Fly the Drone

**Detection starts automatically when:**
- Drone enters **AUTO mode** (mission/waypoint mode), OR
- Drone is **flying** (altitude > 2 meters)

**That's it!** No manual trigger needed.

---

## What Happens Automatically

When drone is in AUTO mode or flying:
```
🌾 Yellow crop detected @ lat=28.xxx, lon=77.xxx
📡 MAVLink: Detection sent via STATUSTEXT
📡 MAVLink: Detection metadata sent
```

These are sent via:
```
Pi → Pixhawk (TELEM2) → Radio (TELEM1) → GCS
```

On your GCS dashboard at `http://localhost:3000/mission-control`, you'll see:
- 📍 Detection markers on map
- 📊 Detection counter
- 📝 Detection list with coordinates

---

## If You Want Manual Control

If you want to force detection ON/OFF regardless of flight mode:

### Method 1: Python Console (while pi_controller.py running)

Open another SSH terminal:
```bash
cd /home/pi/rpi-connect
source venv/bin/activate
python3
```

Then:
```python
# This requires accessing the controller instance
# Simpler to just use flag file below
```

### Method 2: Flag File (Easiest Manual Control)

While pi_controller.py is running, in another terminal:

```bash
# Force START detection
echo "1" > /tmp/detection_flag

# Force STOP detection
echo "0" > /tmp/detection_flag

# Check status
cat /tmp/detection_flag
```

**Note**: This requires a small modification to pi_controller.py to check this file. See below.

---

## Verify Detections Are Reaching GCS

### On Pi terminal (while pi_controller.py runs):
Look for these messages:
```
🌾 Yellow crop detected!
   Location: lat=28.xxx, lon=77.xxx
   Altitude: 10.5m
📡 MAVLink: Detection sent via STATUSTEXT
```

### On GCS computer:
Open dashboard: `http://localhost:3000/mission-control`

You should see:
- Green markers appearing on map
- Detection counter incrementing
- Latest detection coordinates displayed

---

## Quick Start Checklist

✅ **Pi Config** (`rpi-connect/config.json`):
```json
{
  "detection": {"enabled": true},
  "mavlink_detection": {"enabled": true},
  "socketio": {"enabled": false}
}
```

✅ **Start Pi**:
```bash
cd /home/pi/rpi-connect && source venv/bin/activate && python3 pi_controller.py
```

✅ **Start GCS** (Windows):
```bash
cd GCS-without-pi
# Make sure pymavlink_service is running
cd external-services && python pymavlink_service.py
```

✅ **Fly Drone**: Put in AUTO mode or takeoff

✅ **Watch Dashboard**: `http://localhost:3000/mission-control`

---

## Bottom Line

**You don't need to manually start detection!** 

Just run `pi_controller.py` and fly the drone in AUTO mode. Detection happens automatically and results are sent to GCS via MAVLink radio (2-10km range, no WiFi needed).
