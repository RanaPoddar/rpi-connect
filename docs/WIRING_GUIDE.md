# Pixhawk Wiring Guide - Dual Telemetry Setup

## Hardware Connections

### Overview
```
                    ┌─────────────────────────────────┐
                    │         PIXHAWK                 │
                    │                                 │
                    │  [TELEM1]  [TELEM2]  [USB]      │
                    └─────┬─────────┬────────┬────────┘
                          │         │        │
                          │         │        └──────────┐
                          │         │                   │
                ┌─────────┘         └──────────┐        │
                │                              │        │
                ▼                              ▼        ▼
        ┌─────────────┐              ┌─────────────┐  ┌──────┐
        │  915MHz/    │              │ Raspberry   │  │Laptop│
        │  433MHz     │              │    Pi 4     │  │Debug │
        │  Radio      │              └─────────────┘  └──────┘
        └─────────────┘
                │
                │ Wireless
                │ (100m-40km)
                ▼
        ┌─────────────┐
        │  GCS Radio  │
        │  + Laptop   │
        └─────────────┘
```

## Pixhawk TELEM1 Port (GCS Radio)

**Use**: Long-range communication with Ground Control Station

### TELEM1 JST-GH 6-Pin Connector Pinout
```
   ┌─────────────────────┐
   │ Pixhawk TELEM1 Port │
   └─────────────────────┘
        │││││││
        123456
        
Pin 1: +5V     (Red)     - Power for radio (if needed)
Pin 2: TX      (Yellow)  - Transmit from Pixhawk → Radio RX
Pin 3: RX      (Green)   - Receive to Pixhawk ← Radio TX
Pin 4: CTS     (Grey)    - Flow control (optional)
Pin 5: RTS     (Grey)    - Flow control (optional)
Pin 6: GND     (Black)   - Ground
```

### Connection to 915MHz/433MHz Radio
```
Pixhawk TELEM1          Telemetry Radio
──────────────          ───────────────
Pin 1 (+5V)       →     VCC (if radio needs power)
Pin 2 (TX)        →     RX
Pin 3 (RX)        ←     TX
Pin 6 (GND)       →     GND

Settings:
- Baud Rate: 57600
- Air Speed: 64 kbps (configurable)
- Net ID: Match both radios
```

## Pixhawk TELEM2 Port (Raspberry Pi)

**Use**: High-speed local connection for GPS telemetry and detection data

### TELEM2 JST-GH 6-Pin Connector Pinout
```
   ┌─────────────────────┐
   │ Pixhawk TELEM2 Port │
   └─────────────────────┘
        │││││││
        123456
        
Pin 1: +5V     (Red)     - NOT CONNECTED (Pi has own power!)
Pin 2: TX      (Yellow)  - Transmit from Pixhawk → Pi RX
Pin 3: RX      (Green)   - Receive to Pixhawk ← Pi TX
Pin 4: CTS     (Grey)    - NOT CONNECTED
Pin 5: RTS     (Grey)    - NOT CONNECTED
Pin 6: GND     (Black)   - Ground to Pi
```

### Option A: Direct GPIO Connection (Recommended - Fastest)

```
Pixhawk TELEM2                Raspberry Pi GPIO Header
──────────────                ─────────────────────────
Pin 1 (+5V)                   NOT CONNECTED ⚠️
Pin 2 (TX) Yellow      →      GPIO 15 (Pin 10) - UART RXD
Pin 3 (RX) Green       ←      GPIO 14 (Pin 8)  - UART TXD
Pin 6 (GND) Black      →      GND (Pin 6, 9, 14, 20, 25, 30, 34, or 39)

⚠️ CRITICAL: Do NOT connect +5V! Pi GPIO is 3.3V but Pixhawk UART is 3.3V tolerant.
```

**Raspberry Pi GPIO Pinout Reference:**
```
        3.3V  [ 1] [ 2]  5V
   I2C SDA    [ 3] [ 4]  5V
   I2C SCL    [ 5] [ 6]  GND ← Connect Pixhawk GND here
   GPIO 4     [ 7] [ 8]  GPIO 14 (TXD) ← Connect Pixhawk RX
         GND  [ 9] [10]  GPIO 15 (RXD) ← Connect Pixhawk TX
  GPIO 17     [11] [12]  GPIO 18
  GPIO 27     [13] [14]  GND
  GPIO 22     [15] [16]  GPIO 23
        3.3V  [17] [18]  GPIO 24
  SPI MOSI    [19] [20]  GND
  SPI MISO    [21] [22]  GPIO 25
  SPI SCLK    [23] [24]  SPI CE0
         GND  [25] [26]  SPI CE1
  ...
```

**Enable Serial Port on Pi:**
```bash
sudo raspi-config
# Interface Options → Serial Port
# - Login shell: NO
# - Serial hardware: YES
# Reboot

# Verify
ls -l /dev/serial0
# Should show: /dev/serial0 -> ttyS0 (or ttyAMA0 on older Pi)
```

### Option B: USB Connection (Easier - Slower)

```
Pixhawk USB Port  →  USB-A to Micro-USB Cable  →  Raspberry Pi USB Port
                                                   Device: /dev/ttyACM0
```

**Pros:**
- ✅ Easy to connect/disconnect
- ✅ No GPIO wiring needed
- ✅ 3.3V/5V level shifting handled automatically

**Cons:**
- ❌ Slower (max 115200 baud vs 921600 on GPIO)
- ❌ Takes up USB port
- ❌ Cable can be accidentally disconnected

## Complete System Wiring

```
┌──────────────────────────────────────────────────────────────┐
│                    DETECTION DRONE                           │
│                                                              │
│  ┌─────────────┐              ┌──────────────────┐          │
│  │  Pixhawk    │   TELEM2     │  Raspberry Pi 4  │          │
│  │             │◄─────────────►│                  │          │
│  │  ┌────┐     │   Serial/    │  ┌────────────┐  │          │
│  │  │GPS │     │   USB        │  │ HQ Camera  │  │          │
│  │  └────┘     │   921600     │  └────────────┘  │          │
│  │             │              │                  │          │
│  │  [TELEM1]   │              │  WiFi: 10.x.x.x  │          │
│  └──────┬──────┘              └──────────────────┘          │
│         │                                │                  │
│         │ 57600                          │ Socket.IO        │
│         │                                │                  │
└─────────┼────────────────────────────────┼──────────────────┘
          │                                │
          │ 915MHz Radio                   │ WiFi/LTE
          │ (100m-40km)                    │ (100m max)
          │                                │
          ▼                                ▼
  ┌──────────────┐              ┌─────────────────┐
  │ Ground Radio │              │  Server         │
  │              │              │  10.x.x.x:3000  │
  │  [USB]       │              └─────────────────┘
  └──────┬───────┘                        │
         │                                │
         │ USB                            │ HTTP/WS
         │                                │
         ▼                                ▼
  ┌──────────────────────────────────────────────┐
  │         GCS Laptop                           │
  │                                              │
  │  ┌──────────────────┐  ┌─────────────────┐  │
  │  │ Mission Planner  │  │  Web Dashboard  │  │
  │  │ QGroundControl   │  │  (Browser)      │  │
  │  └──────────────────┘  └─────────────────┘  │
  └──────────────────────────────────────────────┘
```

## Power Considerations

### Pixhawk Power
- **Primary**: Battery (via power module)
- **Backup**: USB (if connected)
- **Telemetry Ports**: Can provide 5V @500mA for radios

### Raspberry Pi Power
- **Primary**: Separate power supply (5V/3A recommended)
- **DO NOT**: Power Pi from Pixhawk TELEM port
- **Reason**: Pi draws too much current (~2.5A under load)

### Recommended Power Setup
```
Battery
  │
  ├─► Power Module ─► Pixhawk
  │
  └─► 5V BEC/Step-down ─► Raspberry Pi (5V/3A)
```

## Cable Requirements

### Custom TELEM2 to GPIO Cable

**Option 1: Pre-made JST-GH to Dupont**
- Search: "Pixhawk JST-GH to Dupont female cable"
- Connect to Pi GPIO pins

**Option 2: DIY Cable**
- JST-GH 6-pin connector (Pixhawk side)
- Dupont female connectors (Pi GPIO side)
- 3 wires: TX, RX, GND
- Optional: use different colors for easy identification

### Wire Gauge
- **Signal wires (TX/RX)**: 26-28 AWG
- **Ground**: 22-24 AWG
- **Length**: Keep < 30cm for high-speed serial (921600)

## Verification Checklist

### Pre-Flight Checks

```
☐ GCS Radio connected to TELEM1
☐ GCS Radio powered and paired with ground radio
☐ Raspberry Pi connected to TELEM2 or USB
☐ Raspberry Pi powered separately
☐ Pi serial port enabled (/dev/serial0 exists)
☐ Pixhawk parameters set (SR1_*, SR2_*, SERIAL*_BAUD)
☐ Pixhawk has GPS 3D fix
☐ Pi can read GPS from Pixhawk
☐ GCS receives telemetry at ground station
☐ WiFi connection to server (if in range)
```

### Quick Test Commands

**On Raspberry Pi:**
```bash
# 1. Check serial port
ls -l /dev/serial0

# 2. Test Pixhawk connection
python debug_pixhawk.py

# 3. Test GPS reception
python3 << EOF
from pymavlink import mavutil
m = mavutil.mavlink_connection('/dev/serial0', baud=921600)
m.wait_heartbeat()
print("Connected!")
msg = m.recv_match(type='GPS_RAW_INT', blocking=True, timeout=10)
if msg:
    print(f"GPS: {msg.lat/1e7}, {msg.lon/1e7}, Sats: {msg.satellites_visible}")
EOF

# 4. Start pi_controller
python pi_controller.py
```

## Troubleshooting

### No GPS Coordinates in Detections

**Symptom**: Detections show `lat: 0.0, lon: 0.0`

**Check**:
```bash
# 1. Verify Pi is receiving GPS
python3 << EOF
from pymavlink import mavutil
master = mavutil.mavlink_connection('/dev/serial0', baud=921600)
master.wait_heartbeat()
for i in range(10):
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    if msg:
        print(f"GPS: {msg.lat/1e7}, {msg.lon/1e7}")
    else:
        print("No GPS message - check SR2_POSITION parameter!")
EOF

# 2. Check Pixhawk parameter
# In Mission Planner: SR2_POSITION should be > 0 (recommended: 10)
```

### GCS Not Receiving Detection Messages

**Check**:
1. In Mission Planner: Data → Messages tab
2. Look for messages starting with "DET|"
3. Check SR1_EXT_STAT > 0 (enables STATUSTEXT forwarding)

### Serial Port Permission Denied

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER

# Logout and login for changes to take effect
# Or reboot
```

### Wrong Device Name

```bash
# Find all serial devices
ls -l /dev/tty* | grep -E "ACM|USB|AMA|S0"

# Common names:
# /dev/ttyACM0  - USB connection
# /dev/serial0  - GPIO UART (newer Pi)
# /dev/ttyAMA0  - GPIO UART (older Pi)
# /dev/ttyS0    - GPIO UART (some configurations)
```

## Safety Notes

⚠️ **NEVER**:
- Connect Pixhawk +5V to Pi GPIO (will damage Pi!)
- Use Pixhawk as primary power for Pi
- Connect both Pixhawk USB and TELEM simultaneously (choose one)

✅ **ALWAYS**:
- Use separate power supplies for Pixhawk and Pi
- Connect ground (GND) between devices
- Test connections on the ground before flight
- Have backup communication (WiFi + telemetry)

## Support

For issues:
1. Check `/docs/TELEMETRY_SOLUTION.md`
2. Run `python debug_pixhawk.py`
3. Run `python setup_pixhawk_params.py /dev/serial0 921600`
