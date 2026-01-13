# Manual Detection Control on Raspberry Pi

## SIMPLEST METHOD: Direct Python Commands

### 1. Start pi_controller.py in one terminal:
```bash
cd /home/pi/rpi-connect
source venv/bin/activate
python3 pi_controller.py
```

### 2. In another terminal (or SSH session), run Python:
```bash
cd /home/pi/rpi-connect
source venv/bin/activate
python3
```

### 3. Use these commands:
```python
# Import the module
import sys
sys.path.insert(0, '/home/pi/rpi-connect')

# Start detection
import os
os.system("pkill -f 'python3 pi_controller.py' -USR1")  # Send signal to start

# Or simpler - just check if detection file exists
with open('/tmp/detection_active', 'w') as f:
    f.write('1')  # Start detection

with open('/tmp/detection_active', 'w') as f:
    f.write('0')  # Stop detection
```

## BETTER METHOD: Edit pi_controller.py to check a flag file

Add this to your pi_controller.py detection loop:

```python
# Check for manual control file
if os.path.exists('/tmp/detection_start'):
    self.detection_active = True
    os.remove('/tmp/detection_start')
    print("🌾 Detection started manually via flag file")

if os.path.exists('/tmp/detection_stop'):
    self.detection_active = False
    os.remove('/tmp/detection_stop')
    print("🛑 Detection stopped manually via flag file")
```

Then from any terminal:
```bash
# Start detection
touch /tmp/detection_start

# Stop detection
touch /tmp/detection_stop
```

## EASIEST METHOD: Just set the flag in config and restart

Edit `config.json`:
```json
{
  "detection": {
    "enabled": true
  }
}
```

When `detection.enabled = true`, detection runs automatically whenever:
1. Mission is active (drone in AUTO mode), OR
2. Drone is flying (altitude > 2m)

**This is the recommended approach** - detection auto-starts during flight.

---

## Verify Detections Are Sent via MAVLink

Once pi_controller.py is running with detection active, you should see:
```
🌾 Yellow crop detected @ lat=28.xxx, lon=77.xxx
📡 MAVLink: Detection sent via STATUSTEXT
📡 MAVLink: Detection metadata sent
```

These messages go:
```
Pi → Pixhawk TELEM2 → Pixhawk TELEM1 → Radio → GCS
```

On GCS dashboard (http://localhost:3000/mission-control), you'll see detections appear in real-time.

---

## Quick Test Right Now

```bash
# On Pi
cd /home/pi/rpi-connect
source venv/bin/activate
python3 pi_controller.py

# Wait for "✅ System ready!"
# Detection will auto-start when drone is in AUTO mode or flying
```

The key settings in your config.json are already correct:
- ✅ `detection.enabled: true`
- ✅ `mavlink_detection.enabled: true`
- ✅ `socketio.enabled: false` (no WiFi needed)

Just run `pi_controller.py` and fly the drone - detections will automatically be sent to GCS via MAVLink radio.
