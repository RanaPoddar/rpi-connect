# Telemetry Network Architecture Solution

## Current Problem

**Issue**: When GCS is connected to Pixhawk Telem1, Raspberry Pi cannot connect to Telem2, and detections don't include GPS coordinates.

## Root Cause

You have a **multi-radio MAVLink network** that needs proper routing configuration. The Pixhawk acts as a central hub, but by default, MAVLink messages are not automatically forwarded between serial ports.

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                        IN-FLIGHT SETUP                      │
└─────────────────────────────────────────────────────────────┘

                    ┌─────────────────┐
                    │   PIXHAWK       │
                    │  (MAVLink Hub)  │
                    └────────┬────────┘
                             │
          ┌──────────────────┼──────────────────┐
          │                  │                  │
     TELEM 1            TELEM 2              USB (optional)
          │                  │                  │
          ▼                  ▼                  ▼
    ┌─────────┐      ┌─────────────┐    ┌──────────┐
    │  GCS    │      │ Raspberry   │    │ Laptop   │
    │ Radio   │      │    Pi       │    │ (debug)  │
    └─────────┘      └─────────────┘    └──────────┘
          │                  │
     Long Range         Local Serial
    (100m-40km)         (direct wire)
          │                  │
          ▼                  ▼
    ┌─────────┐      ┌─────────────┐
    │  GCS    │      │  Detection  │
    │ Laptop  │      │  + Images   │
    └─────────┘      └─────────────┘
          │
     WiFi/Internet
          │
          ▼
    ┌─────────┐
    │ Server  │
    │ 10.x.x  │
    └─────────┘
```

## Solution: Dual-Channel Communication

### Architecture Overview

```
Raspberry Pi (Detection)
    │
    ├─► Serial → Pixhawk TELEM2 ────────┐
    │                                   │
    └─► WiFi → Server (when in range)  │
                                        │
                                Pixhawk Hub
                                (forwards packets)
                                        │
                                        ▼
                              GCS Radio ← TELEM1
                                        │
                                        ▼
                                   GCS Laptop
                                (receives detections)
```

## Step-by-Step Configuration

### 1. Pixhawk Parameter Configuration

Connect to Pixhawk via Mission Planner/QGroundControl and set these parameters:

```
# TELEM1 (GCS Radio) - 915MHz/433MHz
SERIAL1_PROTOCOL = 2     # MAVLink2
SERIAL1_BAUD = 57       # 57600 baud (standard for telemetry radios)

# TELEM2 (Raspberry Pi)
SERIAL2_PROTOCOL = 2     # MAVLink2
SERIAL2_BAUD = 921       # 921600 baud (high-speed for local connection)

# USB (Optional - for debugging)
SERIAL0_PROTOCOL = 2     # MAVLink2
SERIAL0_BAUD = 115       # 115200 baud

# Enable message forwarding between all ports
SR1_POSITION = 2         # GPS position to TELEM1 @ 2Hz
SR1_EXTRA1 = 4           # Attitude to TELEM1 @ 4Hz
SR1_EXTRA2 = 2           # VFR_HUD to TELEM1 @ 2Hz
SR1_EXTRA3 = 2           # Other data to TELEM1 @ 2Hz

SR2_POSITION = 10        # GPS position to TELEM2 @ 10Hz (Pi needs high rate)
SR2_EXTRA1 = 10          # Attitude to TELEM2 @ 10Hz
SR2_EXTRA2 = 10          # VFR_HUD to TELEM2 @ 10Hz
SR2_EXTRA3 = 2           # Other data to TELEM2 @ 2Hz

# Message routing (critical for forwarding)
SERIALx_ROUTING = 1      # Enable routing for all SERIALx ports
```

**Important**: After changing parameters, **reboot Pixhawk**!

### 2. Raspberry Pi Hardware Connection

**Option A: TELEM2 Connection (Recommended)**
```
Pixhawk TELEM2        Raspberry Pi GPIO
─────────────────     ─────────────────
Pin 1 (VCC 5V)   →   NOT CONNECTED (Pi has own power)
Pin 2 (TX)       →   GPIO 15 (RXD)    /dev/serial0
Pin 3 (RX)       →   GPIO 14 (TXD)    /dev/serial0
Pin 4 (CTS)      →   NOT CONNECTED
Pin 5 (RTS)      →   NOT CONNECTED
Pin 6 (GND)      →   GND (Pin 6)
```

**Option B: USB Connection (Easier but slower)**
```
Pixhawk USB  →  USB cable  →  Raspberry Pi USB port
                            → /dev/ttyACM0
```

### 3. Enable Raspberry Pi Serial Port

On Raspberry Pi, run:
```bash
sudo raspi-config
# Navigate to: Interface Options → Serial Port
# - "Login shell accessible over serial?" → NO
# - "Serial port hardware enabled?" → YES
# Finish and reboot

# After reboot, verify:
ls -l /dev/serial0
# Should show: lrwxrwxrwx 1 root root 5 Jan  8 12:00 /dev/serial0 -> ttyS0
```

### 4. Update Pi Configuration

Update your `config.json`:

**For TELEM2 (GPIO Serial):**
```json
{
  "pixhawk": {
    "enabled": true,
    "connection_string": "/dev/serial0",
    "baud_rate": 921600,
    "simulation_mode": false,
    "update_rate": 10.0,
    "note": "TELEM2 - High-speed local connection for GPS"
  },
  "mavlink_detection": {
    "enabled": true,
    "send_metadata": true,
    "send_summary_interval": 30.0,
    "note": "Sends detections via Pixhawk to GCS radio"
  }
}
```

**For USB Connection:**
```json
{
  "pixhawk": {
    "enabled": true,
    "connection_string": "/dev/ttyACM0",
    "baud_rate": 115200,
    "simulation_mode": false,
    "update_rate": 10.0
  }
}
```

### 5. How Detection Data Flows

```
┌─────────────────────────────────────────────────────────────┐
│ DETECTION FLOW (Long Range Mission)                         │
└─────────────────────────────────────────────────────────────┘

1. DETECTION EVENT
   Camera → Yellow Crop Detected
                │
                ▼
   GeoLocation Calculator
        + GPS from Pixhawk (via Pi serial connection)
        + Camera parameters
        = Latitude/Longitude of crop

2. DUAL TRANSMISSION

   A. PRIMARY: Socket.IO (WiFi - when in range)
      Detection → Socket.IO → Server → Dashboard
      ✅ Full resolution image
      ✅ All metadata
      ✅ Real-time updates
      
   B. FALLBACK: MAVLink (Always active)
      Detection → Pi → Pixhawk TELEM2 → Pixhawk TELEM1 → GCS Radio → GCS
      ✅ Works at long range (40km+)
      ✅ GPS coordinates only (compact)
      ✅ Via STATUSTEXT messages

3. GCS RECEIVES
   Mission Planner/QGroundControl shows:
   - "DET|id123|12.345678|77.654321|0.89|1523"
   - Parsed: Detection at Lat 12.345678, Lon 77.654321
```

## Troubleshooting

### Pi Cannot Get GPS from Pixhawk

**Symptoms**: 
- Pi connects to Pixhawk
- No GPS coordinates in detections
- `telemetry['gps']['lat'] == 0.0`

**Solutions**:
```bash
# 1. Check if GPS messages are being received
python3 << EOF
from pymavlink import mavutil
master = mavutil.mavlink_connection('/dev/serial0', baud=921600)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Connected!")

print("Waiting for GPS (10 seconds)...")
msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=10)
if msg:
    print(f"GPS: {msg.lat/1e7}, {msg.lon/1e7}")
else:
    print("No GPS received - check SR2_POSITION parameter!")
EOF

# 2. Verify Pixhawk parameters
# In Mission Planner, check:
# - SR2_POSITION > 0 (should be 10)
# - GPS_TYPE != 0 (GPS is enabled)
# - Check GPS status shows 3D Fix
```

### GCS Not Receiving Detection Messages

**Check**:
1. GCS radio is connected to TELEM1
2. SR1_EXT_STAT > 0 (enables STATUSTEXT forwarding)
3. In Mission Planner → Data → Messages tab, look for "DET|" messages
4. MAVLink Inspector shows STATUSTEXT messages

### Both GCS and Pi Connected Issues

If both try to control the drone:
```json
// Pi config.json
{
  "pixhawk": {
    "read_only": true,  // Only read telemetry, don't send commands
    "system_id": 255,   // Use GCS system ID
    "component_id": 191 // Unique component ID
  }
}
```

## Testing Procedure

### Test 1: Local Serial Connection
```bash
cd ~/rpi-connect
python debug_pixhawk.py
# Should find connection and show GPS data
```

### Test 2: GPS in Detections
```bash
python pi_controller.py
# Make a detection
# Check log: Detection should show GPS coordinates
```

### Test 3: MAVLink Transmission
```bash
# Have GCS connected to TELEM1
# Make detection on Pi
# Check Mission Planner Messages tab for "DET|" messages
```

### Test 4: Long Range Test
```bash
# 1. Start mission with both WiFi and telemetry
# 2. Fly beyond WiFi range
# 3. Make detection
# 4. Should appear in GCS via telemetry radio
# 5. Fly back to WiFi range
# 6. Full image should sync via Socket.IO
```

## System ID Configuration

To avoid conflicts when multiple devices connect to Pixhawk:

```python
# In pixhawk_telemetry.py, update connect():
self.vehicle = connect(
    self.connection_string,
    baud=self.baud_rate,
    source_system=255,      # GCS system ID
    source_component=191,    # Camera/companion computer ID
    wait_ready=True
)
```

## Recommended Flight Configuration

**For competitions/missions**:

1. **Short Range (< 100m)**: WiFi only, disable Pixhawk connection
2. **Medium Range (100m-1km)**: Hybrid - WiFi primary, MAVLink backup
3. **Long Range (1km-40km)**: MAVLink primary, WiFi at takeoff/landing

```json
// Long range config
{
  "pixhawk": {"enabled": true, "connection_string": "/dev/serial0", "baud_rate": 921600},
  "mavlink_detection": {"enabled": true},
  "detection": {"auto_detect_in_auto_mode": true}
}
```

## Summary

✅ **Pi connects to TELEM2** for local GPS telemetry  
✅ **GCS connects to TELEM1** for flight control and monitoring  
✅ **Pixhawk forwards** detection messages from TELEM2 to TELEM1  
✅ **Dual transmission**: WiFi (full data) + MAVLink (coordinates only)  
✅ **No conflicts**: Different system/component IDs  

This architecture provides:
- 🎯 GPS-tagged detections
- 📡 Long-range communication
- 🔄 Automatic fallback
- 💪 Robust operation
