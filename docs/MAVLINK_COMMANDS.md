# MAVLink Command Interface Documentation

## Overview
The MAVLink command interface is now integrated into the Pi Controller, allowing remote control of the drone via Socket.IO commands from the dashboard or server.

## Available Commands

### Basic Control

#### ARM
**Socket.IO Event:** `drone_arm`
```javascript
socket.emit('drone_arm', {
  pi_id: 'pi_001',
  args: {
    force: false  // Optional: skip pre-arm checks (dangerous!)
  }
});
```

#### DISARM
**Socket.IO Event:** `drone_disarm`
```javascript
socket.emit('drone_disarm', {
  pi_id: 'pi_001',
  args: {
    force: false  // Optional: force disarm even in flight (dangerous!)
  }
});
```

#### Set Flight Mode
**Socket.IO Event:** `drone_set_mode`
```javascript
socket.emit('drone_set_mode', {
  pi_id: 'pi_001',
  args: {
    mode: 'GUIDED'  // GUIDED, AUTO, LOITER, RTL, LAND, etc.
  }
});
```

### Flight Commands

#### Takeoff
**Socket.IO Event:** `drone_takeoff`
```javascript
socket.emit('drone_takeoff', {
  pi_id: 'pi_001',
  args: {
    altitude: 10.0,  // meters
    timeout: 60      // seconds (optional)
  }
});
```

#### Land
**Socket.IO Event:** `drone_land`
```javascript
socket.emit('drone_land', {
  pi_id: 'pi_001',
  args: {
    timeout: 60  // seconds (optional)
  }
});
```

#### Go To Location
**Socket.IO Event:** `drone_goto`
```javascript
socket.emit('drone_goto', {
  pi_id: 'pi_001',
  args: {
    lat: 40.7128,      // latitude
    lon: -74.0060,     // longitude
    altitude: 15.0,    // meters (optional)
    groundspeed: 5.0,  // m/s (optional)
    timeout: 120       // seconds (optional)
  }
});
```

### Mission Management

#### Upload Mission
**Socket.IO Event:** `drone_upload_mission`
```javascript
socket.emit('drone_upload_mission', {
  pi_id: 'pi_001',
  args: {
    waypoints: [
      {lat: 40.7128, lon: -74.0060, alt: 10},
      {lat: 40.7129, lon: -74.0061, alt: 10},
      {lat: 40.7130, lon: -74.0062, alt: 10, action: 'land'}
    ],
    takeoff_alt: 10.0  // Optional: auto-takeoff altitude
  }
});
```

#### Start Mission
**Socket.IO Event:** `drone_start_mission`
```javascript
socket.emit('drone_start_mission', {
  pi_id: 'pi_001'
});
```

#### Clear Mission
**Socket.IO Event:** `drone_clear_mission`
```javascript
socket.emit('drone_clear_mission', {
  pi_id: 'pi_001'
});
```

### Emergency Commands

#### Return to Launch (RTL)
**Socket.IO Event:** `drone_rtl`
```javascript
socket.emit('drone_rtl', {
  pi_id: 'pi_001'
});
```

#### Emergency Stop
**Socket.IO Event:** `drone_emergency_stop`
```javascript
socket.emit('drone_emergency_stop', {
  pi_id: 'pi_001'
});
```

### Status

#### Get Drone Status
**Socket.IO Event:** `drone_get_status`
```javascript
socket.emit('drone_get_status', {
  pi_id: 'pi_001'
});
```

## Command Results

All commands emit a `drone_command_result` event with the following structure:

```javascript
{
  pi_id: 'pi_001',
  command: 'arm',
  result: {
    success: true,
    message: 'Vehicle armed successfully',
    timestamp: '2025-12-27T19:00:00.000Z'
  },
  timestamp: '2025-12-27T19:00:00.000Z'
}
```

## Using via Terminal Commands

You can also use the commands through the `execute_command` Socket.IO event:

```javascript
socket.emit('execute_command', {
  pi_id: 'pi_001',
  command: 'arm',
  args: {}
});
```

Available commands:
- `arm` - ARM the vehicle
- `disarm` - DISARM the vehicle
- `set_mode` - Change flight mode
- `takeoff` - Takeoff to altitude
- `land` - Land the vehicle
- `goto_location` - Navigate to GPS coordinates
- `upload_mission` - Upload waypoint mission
- `start_mission` - Execute mission
- `clear_mission` - Clear mission
- `get_mission_progress` - Get mission status
- `rtl` - Return to launch
- `emergency_stop` - Emergency land

## Example Workflow

### Complete Mission Execution

```javascript
// 1. ARM the drone
socket.emit('drone_arm', {pi_id: 'pi_001', args: {}});

// 2. Takeoff to 10 meters
socket.emit('drone_takeoff', {pi_id: 'pi_001', args: {altitude: 10.0}});

// 3. Navigate to location
socket.emit('drone_goto', {
  pi_id: 'pi_001',
  args: {lat: 40.7128, lon: -74.0060, altitude: 10.0}
});

// 4. Land
socket.emit('drone_land', {pi_id: 'pi_001', args: {}});

// Vehicle will auto-disarm after landing
```

### Autonomous Mission

```javascript
// 1. ARM the drone
socket.emit('drone_arm', {pi_id: 'pi_001', args: {}});

// 2. Upload mission
socket.emit('drone_upload_mission', {
  pi_id: 'pi_001',
  args: {
    waypoints: [
      {lat: 40.7128, lon: -74.0060, alt: 10},
      {lat: 40.7129, lon: -74.0061, alt: 10},
      {lat: 40.7130, lon: -74.0062, alt: 10}
    ],
    takeoff_alt: 10.0
  }
});

// 3. Start mission (will auto-takeoff and execute)
socket.emit('drone_start_mission', {pi_id: 'pi_001'});

// Mission executes autonomously!
```

## Safety Notes

1. **Always verify GPS lock** before arming
2. **Check battery levels** before flight
3. **Know your airspace** and follow regulations
4. **Have emergency RTL ready** at all times
5. **Test in simulation** mode first
6. **Monitor telemetry** continuously during flight

## Simulation Mode

For testing without real hardware, enable simulation mode in `config.json`:

```json
{
  "pixhawk": {
    "enabled": true,
    "simulation_mode": true
  }
}
```

This allows you to test all commands safely without a real drone.
