# MAVLink Detection Transmission - Troubleshooting Guide

## Problem
Raspberry Pi is not sending detection data to GCS via MAVLink telemetry for long-range operation.

---

## System Architecture

```
[Pi Camera] → [Yellow Detector] → [Detection Data]
                                        ↓
                                   [Pi Controller]
                                   ↙         ↘
                        [Socket.IO]      [MAVLink]
                              ↓               ↓
                        [GCS WiFi]      [TELEM2 → TELEM1]
                                              ↓
                                        [GCS Radio]
```

### Current Setup
- **Pi → Pixhawk:** TELEM2 (/dev/serial0) at 921600 baud
- **Pixhawk → GCS:** TELEM1 (radio telemetry)
- **Detection Sender:** Enabled in config.json
- **Message Type:** STATUSTEXT with custom prefixes

---

## Configuration Status

### ✅ What's Already Configured

1. **config.json:**
   ```json
   "mavlink_detection": {
     "enabled": true,
     "description": "Hybrid detection transmission - Socket.IO primary + MAVLink fallback",
     "send_metadata": true,
     "send_summary_interval": 30.0
   }
   ```

2. **Pixhawk Connection:**
   ```json
   "pixhawk": {
     "enabled": true,
     "connection_string": "/dev/serial0",
     "baud_rate": 921600,
     "read_only": true
   }
   ```

3. **Detection Code:** MAVLinkDetectionSender is implemented and integrated in pi_controller.py

---

## Common Issues & Solutions

### Issue 1: Pixhawk Connection Not Established

**Symptoms:**
- Pi log shows: "⚠️  MAVLink Detection Sender not available"
- No heartbeat from Pixhawk

**Checks:**
```bash
# On Raspberry Pi:
# 1. Check serial port exists
ls -l /dev/serial0

# 2. Check permissions
sudo usermod -a -G dialout pi
sudo usermod -a -G tty pi

# 3. Test Pixhawk connection
python3 -c "from pymavlink import mavutil; m = mavutil.mavlink_connection('/dev/serial0', baud=921600); m.wait_heartbeat(); print('Connected!')"

# 4. Check if other processes are using the port
sudo lsof /dev/serial0
```

**Solution:**
- Ensure Pi user has serial port permissions
- Verify TELEM2 is not being used by other processes
- Check physical connections (TX→RX, RX→TX, GND)

---

### Issue 2: MAVLink Sender Not Initialized

**Symptoms:**
- Pi log shows: "⚠️  Vehicle._master not available"
- Pixhawk telemetry works but detection sender doesn't

**Check Code Status:**
```python
# In pi_controller.py around line 229-246
# The sender initialization depends on:
# 1. MAVLINK_DETECTION_SENDER_AVAILABLE = True
# 2. self.pixhawk.vehicle exists
# 3. self.pixhawk.vehicle._master exists
```

**Solution:**
Add debugging to pi_controller.py initialization:

```python
# Around line 229, add debug prints:
if MAVLINK_DETECTION_SENDER_AVAILABLE and self.pixhawk.vehicle:
    print(f"📡 DEBUG: Vehicle object exists: {self.pixhawk.vehicle is not None}")
    print(f"📡 DEBUG: Has _master: {hasattr(self.pixhawk.vehicle, '_master')}")
    if hasattr(self.pixhawk.vehicle, '_master'):
        print(f"📡 DEBUG: _master value: {self.pixhawk.vehicle._master}")
```

---

### Issue 3: Messages Not Forwarded Through Pixhawk

**Problem:** Pixhawk may not forward STATUSTEXT from TELEM2 to TELEM1

**Solution - Update Pixhawk Parameters:**

Connect to Pixhawk with Mission Planner/QGC and set:

```
SERIAL2_PROTOCOL = 2    (MAVLink2)
SERIAL2_BAUD = 921      (921600 baud)
SR2_EXTRA1 = 10         (Attitude data rate)
SR2_EXTRA2 = 10         (VFR_HUD data rate)
SR2_EXTRA3 = 2          (Sensor data rate)
SR2_POSITION = 3        (GPS position rate)
SR2_RAW_SENS = 2        (Raw sensor rate)
SR2_RC_CHAN = 2         (RC channel rate)

# IMPORTANT: Allow STATUSTEXT forwarding
SERIAL2_OPTIONS = 0     (No special options)
LOG_BACKEND_TYPE = 1    (MAVLink)
```

**Verify in Mission Planner:**
- Go to CONFIG → Full Parameter List
- Search for SERIAL2 parameters
- Save and reboot Pixhawk

---

### Issue 4: STATUSTEXT Messages Being Filtered

**Problem:** GCS may filter out custom STATUSTEXT messages

**GCS Side Solution:**

#### For Mission Planner:
1. Go to CONFIG → Planner
2. Enable "Show STATUSTEXT messages"
3. Check STATUSTEXT tab in Flight Data screen

#### For QGroundControl:
1. Go to Application Settings
2. Console → Show all MAVLink messages
3. Filter for "STATUSTEXT"

#### For Custom GCS:
Add STATUSTEXT listener in your GCS code:

```javascript
// Node.js example (using node-mavlink)
master.on('STATUSTEXT', (msg) => {
  const text = msg.text.toString();
  
  // Parse detection messages
  if (text.startsWith('DET|')) {
    const parts = text.split('|');
    const detection = {
      id: parts[1],
      lat: parseFloat(parts[2]),
      lon: parseFloat(parts[3]),
      confidence: parseFloat(parts[4]),
      area: parseInt(parts[5])
    };
    console.log('Detection received:', detection);
  }
});
```

---

### Issue 5: Read-Only Mode Preventing Transmission

**Problem:** Pixhawk connection in "read_only" mode

**Check config.json:**
```json
"pixhawk": {
  "read_only": true,  // ← This prevents writing!
}
```

**Solution:**
Change to `read_only: false` if you need to send data:

```json
"pixhawk": {
  "enabled": true,
  "connection_string": "/dev/serial0",
  "baud_rate": 921600,
  "read_only": false,  // ← Allow writing
  "system_id": 255,
  "component_id": 191
}
```

⚠️ **Note:** Setting read_only=false means Pi can send MAVLink commands. Be careful not to interfere with GCS control!

---

## Testing Procedure

### Step 1: Verify Pixhawk Connection
```bash
cd /home/pi/rpi-connect
python3 -c "
from pymavlink import mavutil
import time

print('Connecting to Pixhawk...')
master = mavutil.mavlink_connection('/dev/serial0', baud=921600)
print('Waiting for heartbeat...')
master.wait_heartbeat()
print('✅ Connected! System ID:', master.target_system)

# Test sending STATUSTEXT
master.mav.statustext_send(
    mavutil.mavlink.MAV_SEVERITY_INFO,
    b'TEST: Pi to Pixhawk connection OK'
)
print('✅ Test message sent')
time.sleep(2)
"
```

**Expected:** No errors, prints "Connected!"

---

### Step 2: Test Detection Sender Directly
```bash
cd /home/pi/rpi-connect
python3 modules/mavlink_detection_sender.py
```

**Expected:** Should connect and send test detection message

---

### Step 3: Monitor Pi Logs in Real-Time
```bash
# Start pi_controller and watch for MAVLink messages
python3 pi_controller.py 2>&1 | grep -E "MAVLink|📡|Detection"
```

**Look for:**
- `✅ MAVLink Detection Sender initialized`
- `📡 MAVLink: Sent detection XXXX via telemetry`

---

### Step 4: Monitor GCS Side

#### Mission Planner:
1. Connect to drone
2. Open Messages tab (Ctrl+F → Messages)
3. Look for messages starting with "DET|"

#### QGroundControl:
1. Connect to drone
2. Open Analyze Tools → MAVLink Inspector
3. Filter for STATUSTEXT
4. Look for custom detection messages

---

## Message Format Reference

### Detection Message (DET)
```
Format: DET|ID|LAT|LON|CONF|AREA
Example: DET|mission_001_0001|40.712800|-74.006000|0.95|1732
Length: ~50 chars (STATUSTEXT limit)
```

### Detection Summary (DSTAT)
```
Format: DSTAT|TOTAL|ACTIVE|MISSION
Example: DSTAT|15|1|mission_001
```

### Image Metadata (IMG)
```
Format: IMG|ID|LAT|LON|TYPE|MISSION
Example: IMG|img_20260112_001|40.712800|-74.006000|periodic|mission_001
```

### System Stats (STAT)
```
Format: STAT|CPU|MEM|DISK|TEMP
Example: STAT|45.2|62.3|58.1|48.5
```

---

## Quick Fix Checklist

- [ ] Verify /dev/serial0 exists and accessible
- [ ] Set `read_only: false` in config.json
- [ ] Restart pi_controller service
- [ ] Check Pixhawk SERIAL2 parameters
- [ ] Enable STATUSTEXT display in GCS
- [ ] Test with mavlink_detection_sender.py
- [ ] Monitor logs for MAVLink initialization messages
- [ ] Verify physical TELEM2 connection (TX/RX/GND)

---

## Recommended Configuration Changes

Edit `/home/pi/rpi-connect/config.json`:

```json
{
  "pixhawk": {
    "enabled": true,
    "connection_string": "/dev/serial0",
    "baud_rate": 921600,
    "simulation_mode": false,
    "update_rate": 10.0,
    "read_only": false,  // ← CHANGE THIS
    "system_id": 255,
    "component_id": 191
  },
  "mavlink_detection": {
    "enabled": true,
    "send_metadata": true,
    "send_summary_interval": 30.0
  }
}
```

Then restart:
```bash
sudo systemctl restart pi-controller
# OR
python3 pi_controller.py
```

---

## Expected Behavior (When Working)

### Pi Side Logs:
```
✅ MAVLink Detection Sender initialized - Hybrid transmission enabled!
📡 MAVLink: Sent detection 0001 via telemetry
   ✅ [mission_001] Detection 0001 sent via BOTH channels
```

### GCS Side:
- STATUSTEXT messages appear with "DET|" prefix
- Can parse lat/lon/confidence from messages
- Detection data displayed on map

---

## Alternative: Use MAVLink COMMAND_LONG

If STATUSTEXT forwarding doesn't work, consider using COMMAND_LONG with custom param values:

```python
# Send detection as COMMAND_LONG DO_SET_PARAMETER
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER,
    0,  # confirmation
    lat,  # param1
    lon,  # param2
    confidence,  # param3
    area,  # param4
    0, 0, 0  # unused params
)
```

This approach is more reliable for forwarding through Pixhawk.

---

## Need More Help?

Check the pi_controller logs for specific error messages:
```bash
sudo journalctl -u pi-controller -f | grep -i mavlink
```

Or run in debug mode:
```bash
cd /home/pi/rpi-connect
python3 pi_controller.py --debug
```
