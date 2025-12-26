# Pixhawk Telemetry Integration Guide

## Overview

This system integrates Pixhawk flight controller telemetry with the NIDAR drone detection and monitoring system. The Raspberry Pi connected to the Pixhawk retrieves real-time telemetry data and transmits it to the dashboard via Socket.IO.

## Architecture

```
┌─────────────────┐          ┌──────────────────┐          ┌─────────────┐
│  Pixhawk FC     │  MAVLink │  Raspberry Pi    │ Socket.IO│   Server    │
│  (Flight Ctrl)  │◄─────────┤  (Pi Controller) │◄─────────┤  (Node.js)  │
└─────────────────┘          └──────────────────┘          └─────────────┘
                                      │                            │
                                      │                            │
                                      └────────────────────────────┤
                                                                   │
                                                          ┌────────▼────────┐
                                                          │   Dashboard     │
                                                          │   (React)       │
                                                          └─────────────────┘
```

## Components

### 1. Pixhawk Telemetry Module (`modules/pixhawk_telemetry.py`)

A modular Python class that handles:
- Connection to Pixhawk via MAVLink protocol
- Real-time telemetry data retrieval
- Background thread for continuous updates
- Simulation mode for testing without hardware
- Thread-safe data access

**Key Features:**
- GPS coordinates (latitude, longitude, altitude)
- Attitude (roll, pitch, yaw)
- Velocity (ground speed, air speed, 3D velocity)
- Battery status (voltage, current, level)
- Flight mode and armed status
- System health (EKF status, GPS fix quality)

### 2. Pi Controller Integration (`pi_controller.py`)

Enhanced to include:
- Pixhawk telemetry initialization on startup
- Automatic telemetry transmission via Socket.IO
- Command handlers for Pixhawk status and reconnection
- Clean shutdown procedures

### 3. Server Socket Events (`server/index.js`)

New socket events:
- `drone_telemetry` - Receives telemetry from Raspberry Pi
- `drone_telemetry_update` - Broadcasts telemetry to dashboard
- `request_telemetry` - Dashboard requests telemetry update
- `pixhawk_status_update` - Pixhawk connection status changes
- `reconnect_pixhawk` - Trigger Pixhawk reconnection

### 4. Dashboard Component (`DroneTelemetry.jsx`)

React component displaying:
- Real-time GPS position with fix quality
- Flight status (mode, armed state, heading)
- Attitude visualization (roll, pitch, yaw)
- Velocity information
- Battery status with visual indicator
- Connection status badge

## Installation

### On Raspberry Pi

1. **Install DroneKit and PyMAVLink:**
   ```bash
   cd /home/ranap/Documents/NIDARNEW/rpi-connect
   pip install -r requirements.txt
   ```

2. **Configure Pixhawk connection:**
   Edit `config.json`:
   ```json
   {
     "pixhawk": {
       "enabled": true,
       "connection_string": "/dev/ttyACM0",
       "baud_rate": 57600,
       "simulation_mode": false,
       "update_rate": 1.0
     }
   }
   ```

3. **Connect Pixhawk to Raspberry Pi:**
   - USB connection: `/dev/ttyACM0` or `/dev/ttyUSB0`
   - Serial (UART): `/dev/ttyAMA0` or `/dev/serial0`
   - TCP/UDP: `tcp:127.0.0.1:5760` or `udp:127.0.0.1:14550`

4. **Set up permissions (if needed):**
   ```bash
   sudo usermod -a -G dialout $USER
   sudo chmod 666 /dev/ttyACM0
   ```

### On Server

No additional installation required. Socket events are already configured.

### On Dashboard

Import and use the DroneTelemetry component:
```jsx
import DroneTelemetry from './components/DroneTelemetry';

// In your dashboard component
<DroneTelemetry socket={socket} piId="detection_drone_pi_pushpak" />
```

## Configuration

### Pixhawk Configuration Options

```json
{
  "pixhawk": {
    "enabled": true,              // Enable/disable Pixhawk telemetry
    "connection_string": "/dev/ttyACM0",  // Serial port or network address
    "baud_rate": 57600,           // Baud rate for serial connection
    "simulation_mode": false,     // Use simulated data for testing
    "update_rate": 1.0            // Telemetry update frequency (Hz)
  }
}
```

### Connection String Examples

- **USB Serial:** `/dev/ttyACM0`, `/dev/ttyUSB0`
- **UART Serial:** `/dev/ttyAMA0`, `/dev/serial0`
- **TCP:** `tcp:192.168.1.100:5760`
- **UDP:** `udp:127.0.0.1:14550`
- **SITL (Simulation):** `tcp:127.0.0.1:5760`

## Usage

### Starting the System

1. **Start Server:**
   ```bash
   cd /home/ranap/Documents/NIDARNEW/server
   npm start
   ```

2. **Start Pi Controller:**
   ```bash
   cd /home/ranap/Documents/NIDARNEW/rpi-connect
   python3 pi_controller.py
   ```

3. **Start Dashboard:**
   ```bash
   cd /home/ranap/Documents/NIDARNEW/dashboard/nidar-dashboard
   npm run dev
   ```

### Testing with Simulation Mode

For testing without actual Pixhawk hardware:

1. Set `"simulation_mode": true` in `config.json`
2. The system will generate realistic simulated telemetry data
3. Useful for development and testing dashboard components

### Testing the Pixhawk Module Standalone

```bash
cd /home/ranap/Documents/NIDARNEW/rpi-connect
python3 modules/pixhawk_telemetry.py
```

This will run a test program that connects to Pixhawk and displays telemetry in the terminal.

## Telemetry Data Structure

```javascript
{
  "connected": true,
  "timestamp": "2025-12-26T10:30:45.123Z",
  "gps": {
    "lat": 40.712800,
    "lon": -74.006000,
    "alt": 50.5,
    "relative_alt": 50.5,
    "satellites": 12,
    "fix_type": 3,
    "hdop": 0.8
  },
  "attitude": {
    "roll": 0.05,
    "pitch": 0.02,
    "yaw": 1.57
  },
  "velocity": {
    "vx": 2.5,
    "vy": 1.0,
    "vz": 0.5,
    "groundspeed": 5.2,
    "airspeed": 5.5
  },
  "battery": {
    "voltage": 12.4,
    "current": 15.3,
    "level": 85,
    "remaining": 85
  },
  "flight_mode": "GUIDED",
  "armed": true,
  "system_status": "ACTIVE",
  "ekf_ok": true,
  "heading": 90
}
```

## Troubleshooting

### Pixhawk Not Connecting

1. **Check physical connection:**
   ```bash
   ls -l /dev/ttyACM* /dev/ttyUSB*
   ```

2. **Check permissions:**
   ```bash
   sudo chmod 666 /dev/ttyACM0
   ```

3. **Verify MAVLink communication:**
   ```bash
   sudo apt-get install screen
   screen /dev/ttyACM0 57600
   ```
   You should see MAVLink messages.

4. **Check DroneKit installation:**
   ```bash
   python3 -c "import dronekit; print(dronekit.__version__)"
   ```

### No Telemetry on Dashboard

1. Check server logs for socket connections
2. Verify Pi Controller is connected to server
3. Check browser console for Socket.IO errors
4. Ensure `request_telemetry` events are being sent

### Simulation Mode Not Working

1. Ensure `simulation_mode: true` in config.json
2. Check Pi Controller logs for initialization messages
3. Verify no actual Pixhawk connection attempts

## Socket.IO Events Reference

### From Pi to Server

| Event | Description | Data |
|-------|-------------|------|
| `pi_register` | Register Pi with server | `{pi_id, timestamp}` |
| `drone_telemetry` | Send telemetry data | `{pi_id, telemetry, timestamp}` |
| `pixhawk_status` | Pixhawk connection status | `{pi_id, status, timestamp}` |
| `command_result` | Result of command execution | `{pi_id, command, result}` |

### From Server to Pi

| Event | Description | Data |
|-------|-------------|------|
| `request_telemetry` | Request telemetry update | `{timestamp}` |
| `execute_command` | Execute command on Pi | `{command, args}` |
| `reconnect_pixhawk` | Reconnect to Pixhawk | `{}` |

### From Server to Dashboard

| Event | Description | Data |
|-------|-------------|------|
| `drone_telemetry_update` | Broadcast telemetry | `{pi_id, telemetry, timestamp}` |
| `pixhawk_status_update` | Pixhawk status change | `{pi_id, status, timestamp}` |
| `pi_connected` | Pi connected to server | `{id, name, status}` |
| `pi_disconnected` | Pi disconnected | `{piId, name}` |

## Performance Considerations

- **Update Rate:** Default 1 Hz (adjustable in config)
- **Network Bandwidth:** ~1-2 KB per telemetry update
- **CPU Usage:** Minimal (<5% on Raspberry Pi 4)
- **Latency:** Typically <100ms from Pixhawk to dashboard

## Future Enhancements

- [ ] Historical telemetry data logging
- [ ] Flight path visualization on map
- [ ] Telemetry alerts and warnings
- [ ] Multiple drone support
- [ ] Telemetry data export (CSV, JSON)
- [ ] Advanced analytics and statistics
- [ ] Geofencing and boundary alerts
- [ ] Mission planning integration

## Support

For issues or questions:
1. Check logs: `tail -f /var/log/syslog` (on Pi)
2. Review Socket.IO connection status
3. Verify Pixhawk firmware compatibility
4. Test with simulation mode first

## License

Part of the NIDAR Agricultural Drone Detection System.
