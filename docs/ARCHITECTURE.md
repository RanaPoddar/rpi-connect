# System Architecture - Pixhawk Telemetry Integration

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         DETECTION DRONE                              │
│  ┌────────────────────┐              ┌──────────────────────┐       │
│  │  Pixhawk Flight    │   MAVLink    │   Raspberry Pi 4     │       │
│  │    Controller      │◄─────────────┤                      │       │
│  │                    │  (Serial/USB)│  Pi Controller       │       │
│  │  - GPS Module      │              │  + Telemetry Module  │       │
│  │  - IMU/Sensors     │              │  + Camera            │       │
│  │  - Battery Monitor │              │                      │       │
│  └────────────────────┘              └──────────────────────┘       │
│                                                │                     │
└────────────────────────────────────────────────┼─────────────────────┘
                                                  │
                                    Socket.IO (WiFi/LTE)
                                                  │
                                                  ▼
                        ┌─────────────────────────────────────┐
                        │         SERVER (Node.js)            │
                        │                                     │
                        │  ┌──────────────────────────────┐   │
                        │  │  Socket.IO Event Handler     │   │
                        │  │  - drone_telemetry           │   │
                        │  │  - request_telemetry         │   │
                        │  │  - pixhawk_status_update     │   │
                        │  └──────────────────────────────┘   │
                        │                                     │
                        └─────────────────────────────────────┘
                                        │
                                        │ WebSocket
                                        ▼
                        ┌─────────────────────────────────────┐
                        │      DASHBOARD (React + Vite)       │
                        │                                     │
                        │  ┌──────────────────────────────┐   │
                        │  │  DroneTelemetry Component    │   │
                        │  │  - GPS Position Display      │   │
                        │  │  - Flight Status             │   │
                        │  │  - Battery Monitor           │   │
                        │  │  - Attitude Display          │   │
                        │  └──────────────────────────────┘   │
                        │                                     │
                        └─────────────────────────────────────┘
```

## Data Flow

```
1. TELEMETRY ACQUISITION
   Pixhawk ──MAVLink──> Raspberry Pi
                        ├─ GPS Data
                        ├─ Attitude
                        ├─ Battery
                        └─ Flight Status

2. DATA TRANSMISSION
   Raspberry Pi ──Socket.IO──> Server
                                 │
                    Event: 'drone_telemetry'
                    Data: {
                      pi_id: "...",
                      telemetry: {...},
                      timestamp: "..."
                    }

3. DATA BROADCAST
   Server ──Socket.IO──> Dashboard
                          │
         Event: 'drone_telemetry_update'
         Data: {
           pi_id: "...",
           telemetry: {...},
           timestamp: "..."
         }

4. DISPLAY UPDATE
   Dashboard
   └─ DroneTelemetry Component
      ├─ Parse telemetry data
      ├─ Update UI state
      └─ Render visualizations
```

## Module Structure

```
rpi-connect/
│
├─ modules/
│  ├─ __init__.py
│  └─ pixhawk_telemetry.py ─────┐
│                                │
│                                │  Provides:
│                                │  - PixhawkTelemetry class
│                                │  - connect()
│                                │  - start_telemetry_updates()
│                                │  - get_telemetry()
│                                │  - disconnect()
│                                │
├─ pi_controller.py ◄────────────┘
│  │
│  │  Uses PixhawkTelemetry to:
│  │  1. Connect to Pixhawk
│  │  2. Start background updates
│  │  3. Transmit via Socket.IO
│  │
│  └─ Sends to Server
│
config.json ────────────────────────┐
  {                                 │
    "pixhawk": {                    │  Configuration
      "enabled": true,              │  for connection
      "connection_string": "...",   │  and behavior
      "simulation_mode": true       │
    }                               │
  }                                 │
                                    ▼
                         Applied to PixhawkTelemetry
```

## Component Hierarchy (Dashboard)

```
App.jsx
├─ useState(socket)
├─ useEffect() → Initialize Socket.IO
│
└─ <DroneTelemetry socket={socket} piId="..." />
   │
   ├─ useState(telemetry)
   ├─ useState(connected)
   ├─ useState(lastUpdate)
   │
   ├─ useEffect() → Setup Socket listeners
   │  ├─ socket.on('drone_telemetry_update')
   │  ├─ socket.on('pixhawk_status_update')
   │  └─ setInterval() → Request updates
   │
   └─ Render:
      ├─ Connection Status Badge
      ├─ Flight Status Section
      ├─ GPS Position Section
      ├─ Attitude Section
      ├─ Velocity Section
      └─ Battery Section
```

## Socket.IO Event Flow

```
Dashboard                 Server                 Raspberry Pi
    │                       │                         │
    ├──────────────────────►│                         │
    │  connect              │                         │
    │                       │                         │
    │                       │◄────────────────────────┤
    │                       │  pi_register            │
    │                       │                         │
    │                       │  (Pi connects)          │
    │                       │                         │
    │                       │◄────────────────────────┤
    │                       │  drone_telemetry        │
    │                       │  {telemetry data}       │
    │                       │                         │
    │◄──────────────────────┤                         │
    │  drone_telemetry_update                         │
    │  {telemetry data}     │                         │
    │                       │                         │
    │  (Display updates)    │                         │
    │                       │                         │
    ├──────────────────────►│                         │
    │  request_telemetry    │                         │
    │                       │                         │
    │                       ├────────────────────────►│
    │                       │  request_telemetry      │
    │                       │                         │
    │                       │◄────────────────────────┤
    │                       │  drone_telemetry        │
    │                       │  {latest data}          │
    │                       │                         │
    │◄──────────────────────┤                         │
    │  drone_telemetry_update                         │
    │                       │                         │
    │  (Updates continue every second...)            │
```

## Telemetry Data Structure

```
telemetry = {
  "connected": boolean,
  "timestamp": "ISO8601 string",
  
  "gps": {
    "lat": float,          // Latitude (-90 to 90)
    "lon": float,          // Longitude (-180 to 180)
    "alt": float,          // Altitude MSL (meters)
    "relative_alt": float, // Altitude AGL (meters)
    "satellites": int,     // Number of satellites
    "fix_type": int,       // 0-6 (0=No fix, 3=3D fix)
    "hdop": float          // Horizontal dilution of precision
  },
  
  "attitude": {
    "roll": float,         // Roll angle (radians)
    "pitch": float,        // Pitch angle (radians)
    "yaw": float           // Yaw angle (radians)
  },
  
  "velocity": {
    "vx": float,           // X velocity (m/s)
    "vy": float,           // Y velocity (m/s)
    "vz": float,           // Z velocity (m/s)
    "groundspeed": float,  // Ground speed (m/s)
    "airspeed": float      // Air speed (m/s)
  },
  
  "battery": {
    "voltage": float,      // Battery voltage (V)
    "current": float,      // Current draw (A)
    "level": int,          // Battery level (%)
    "remaining": int       // Remaining capacity (%)
  },
  
  "flight_mode": string,   // "STABILIZE", "GUIDED", etc.
  "armed": boolean,        // True if armed
  "system_status": string, // "ACTIVE", "STANDBY", etc.
  "ekf_ok": boolean,       // Extended Kalman Filter status
  "heading": int           // Heading (0-359 degrees)
}
```

## Connection Options

```
OPTION 1: USB Connection (Recommended)
┌─────────────┐   USB Cable   ┌──────────────┐
│  Pixhawk    │───────────────│ Raspberry Pi │
│  USB Port   │               │  USB Port    │
└─────────────┘               └──────────────┘
Device: /dev/ttyACM0 or /dev/ttyUSB0

OPTION 2: UART Serial
┌─────────────┐   UART/Serial   ┌──────────────┐
│  Pixhawk    │─────────────────│ Raspberry Pi │
│  TELEM Port │  TX/RX/GND      │  GPIO Pins   │
└─────────────┘                 └──────────────┘
Device: /dev/ttyAMA0 or /dev/serial0

OPTION 3: Network (Simulation/SITL)
┌─────────────┐   TCP/UDP    ┌──────────────┐
│  SITL or    │──────────────│ Raspberry Pi │
│  Companion  │  Localhost   │  or Computer │
└─────────────┘              └──────────────┘
Address: tcp:127.0.0.1:5760 or udp:127.0.0.1:14550
```

## Deployment Architecture

```
GROUND STATION                    DETECTION DRONE
┌──────────────────┐             ┌────────────────────────┐
│                  │             │  ┌──────────────────┐  │
│  Dashboard       │             │  │  Pixhawk         │  │
│  (Web Browser)   │             │  │  Flight Ctrl     │  │
│                  │             │  └────────┬─────────┘  │
└─────────┬────────┘             │           │            │
          │                      │  ┌────────▼─────────┐  │
          │ WiFi/Internet        │  │  Raspberry Pi 4  │  │
          │                      │  │  - Pi Controller │  │
          ▼                      │  │  - Telemetry Mod │  │
┌──────────────────┐             │  │  - Camera        │  │
│                  │             │  └────────┬─────────┘  │
│  Server          │◄────────────┼───────────┘            │
│  (Node.js)       │  WiFi/LTE   │                        │
│                  │             │                        │
└──────────────────┘             └────────────────────────┘
      Airport/                        In Flight
      Control Room                    (Real-time telemetry)
```

## Performance Characteristics

```
Component          | Latency    | Update Rate | CPU Usage | Bandwidth
-------------------|------------|-------------|-----------|----------
Pixhawk → Pi       | <10ms      | 1-10 Hz     | <2%       | <1 KB/s
Pi → Server        | 20-100ms   | 1 Hz        | <3%       | 1-2 KB/s
Server → Dashboard | 10-50ms    | Real-time   | <1%       | 1-2 KB/s
End-to-End         | 50-200ms   | 1 Hz        | <5%       | 2-4 KB/s
```

## State Management

```
Raspberry Pi State:
├─ self.pixhawk (PixhawkTelemetry instance)
├─ self.pixhawk_enabled (bool)
├─ telemetry_thread (background thread)
└─ telemetry_data (dict, thread-safe)

Server State:
├─ connectedPis (Map)
│  └─ [piId]: {
│       droneTelemetry: {...}
│     }
└─ io.sockets (active connections)

Dashboard State:
├─ socket (Socket.IO connection)
├─ telemetry (current telemetry data)
├─ connected (connection status)
└─ lastUpdate (timestamp)
```

This architecture provides a robust, scalable, and real-time telemetry system for monitoring drones in flight!
