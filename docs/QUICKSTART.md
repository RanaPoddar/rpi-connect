# Quick Setup Guide - Pixhawk Telemetry System

## Prerequisites

- Raspberry Pi (tested on Pi 4)
- Pixhawk flight controller (optional - can test with simulation mode)
- Node.js server running
- React dashboard

## Installation Steps

### 1. Raspberry Pi Setup

```bash
# Navigate to rpi-connect directory
cd /home/ranap/Documents/NIDARNEW/rpi-connect

# Install Python dependencies (including DroneKit)
pip install -r requirements.txt

# Or install manually:
pip install python-socketio[client]==5.10.0
pip install dronekit==2.9.2
pip install pymavlink==2.4.37
pip install psutil opencv-python numpy pillow picamera2
```

### 2. Configure Pixhawk Connection

Edit `config.json`:

```json
{
  "server_url": "http://YOUR_SERVER_IP:3000",
  "pi_id": "detection_drone_pi_pushpak",
  "pixhawk": {
    "enabled": true,
    "connection_string": "/dev/ttyACM0",
    "baud_rate": 57600,
    "simulation_mode": true,
    "update_rate": 1.0
  }
}
```

**For Testing (No Hardware):**
- Set `"simulation_mode": true`

**For Real Pixhawk:**
- Set `"simulation_mode": false`
- Adjust `connection_string` based on your connection:
  - USB: `/dev/ttyACM0` or `/dev/ttyUSB0`
  - UART: `/dev/ttyAMA0` or `/dev/serial0`
  - Network: `tcp:127.0.0.1:5760` or `udp:127.0.0.1:14550`

### 3. Test Pixhawk Module (Optional)

```bash
cd /home/ranap/Documents/NIDARNEW/rpi-connect

# Test the telemetry module standalone
python3 modules/pixhawk_telemetry.py

# You should see simulated telemetry updates
# Press Ctrl+C to stop
```

### 4. Server Setup

No changes needed! The socket events are already configured in `server/index.js`.

```bash
# Just start the server
cd /home/ranap/Documents/NIDARNEW/server
npm start
```

### 5. Dashboard Setup

```bash
cd /home/ranap/Documents/NIDARNEW/dashboard/nidar-dashboard

# Install socket.io-client if not already installed
npm install socket.io-client

# Start development server
npm run dev
```

### 6. Integrate DroneTelemetry Component

See `INTEGRATION_EXAMPLE.jsx` for a complete example.

**Quick Integration:**

```jsx
// In your App.jsx or main dashboard component
import { useState, useEffect } from 'react';
import { io } from 'socket.io-client';
import DroneTelemetry from './components/DroneTelemetry';

function App() {
  const [socket, setSocket] = useState(null);

  useEffect(() => {
    const newSocket = io('http://localhost:3000', {
      transports: ['websocket'],
      reconnection: true
    });
    setSocket(newSocket);
    return () => newSocket.close();
  }, []);

  return (
    <div>
      {/* Your existing components */}
      
      {/* Add DroneTelemetry component */}
      <DroneTelemetry 
        socket={socket} 
        piId="detection_drone_pi_pushpak" 
      />
    </div>
  );
}
```

## Running the System

### Option 1: Testing with Simulation (No Hardware)

**Terminal 1 - Server:**
```bash
cd /home/ranap/Documents/NIDARNEW/server
npm start
```

**Terminal 2 - Pi Controller (with simulation):**
```bash
cd /home/ranap/Documents/NIDARNEW/rpi-connect
# Make sure simulation_mode: true in config.json
python3 pi_controller.py
```

**Terminal 3 - Dashboard:**
```bash
cd /home/ranap/Documents/NIDARNEW/dashboard/nidar-dashboard
npm run dev
```

**Open browser:** http://localhost:5173

You should see:
- Pi Controller connecting to server
- Simulated telemetry data appearing on dashboard
- Real-time updates every second

### Option 2: With Real Pixhawk Hardware

**Same as above, but:**
1. Connect Pixhawk to Raspberry Pi via USB
2. Set `"simulation_mode": false` in config.json
3. Verify connection: `ls -l /dev/ttyACM*`
4. Run pi_controller.py

## Verification

### Check Pi Controller Connection

In Pi Controller terminal, you should see:
```
🚁 Initializing Pixhawk telemetry...
🔌 Running in SIMULATION mode - generating simulated telemetry
✅ Pixhawk telemetry initialized
📡 Started telemetry updates at 1.0 Hz
Connected to server at http://localhost:3000
```

### Check Server Logs

In Server terminal, you should see:
```
📡 Raspberry Pi connected: Detection Pi (detection_drone_pi_pushpak)
📡 Telemetry [detection_drone_pi_pushpak]: GPS(40.712800, -74.006000), Mode: STABILIZE
```

### Check Dashboard

On the dashboard, you should see a card titled "🚁 Drone Telemetry" with:
- Connection status badge (green "Connected" or blue "Simulation")
- Real-time GPS coordinates
- Flight mode and armed status
- Battery level with progress bar
- Attitude (roll, pitch, yaw)
- Velocity information
- Updates every second

## Troubleshooting

### "Module not found: pixhawk_telemetry"
```bash
# Make sure you're in rpi-connect directory
cd /home/ranap/Documents/NIDARNEW/rpi-connect
python3 pi_controller.py
```

### "DroneKit not available"
```bash
pip install dronekit pymavlink
```

### "Permission denied: /dev/ttyACM0"
```bash
sudo chmod 666 /dev/ttyACM0
# Or permanently:
sudo usermod -a -G dialout $USER
sudo reboot
```

### Dashboard not receiving telemetry
1. Check browser console for Socket.IO connection
2. Verify server URL in dashboard code
3. Check server is running and accessible
4. Verify Pi Controller is connected (check server logs)

### No telemetry updates
1. Check `pixhawk.enabled: true` in config.json
2. Verify Pi Controller logs show telemetry initialization
3. Request telemetry manually from browser console:
```javascript
socket.emit('request_telemetry', { pi_id: 'detection_drone_pi_pushpak' });
```

## File Locations

**Raspberry Pi:**
- Main controller: `/home/ranap/Documents/NIDARNEW/rpi-connect/pi_controller.py`
- Telemetry module: `/home/ranap/Documents/NIDARNEW/rpi-connect/modules/pixhawk_telemetry.py`
- Config: `/home/ranap/Documents/NIDARNEW/rpi-connect/config.json`

**Server:**
- Socket events: `/home/ranap/Documents/NIDARNEW/server/index.js`

**Dashboard:**
- Component: `/home/ranap/Documents/NIDARNEW/dashboard/nidar-dashboard/src/components/DroneTelemetry.jsx`
- Example: `/home/ranap/Documents/NIDARNEW/dashboard/nidar-dashboard/INTEGRATION_EXAMPLE.jsx`

## Next Steps

1. ✅ Test with simulation mode first
2. ✅ Verify telemetry appears on dashboard
3. ✅ Connect real Pixhawk hardware
4. ✅ Disable simulation mode
5. ✅ Test with real telemetry
6. 🚀 Deploy to drone!

## Support Documentation

- [PIXHAWK_TELEMETRY.md](PIXHAWK_TELEMETRY.md) - Complete technical documentation
- [INTEGRATION_SUMMARY.md](INTEGRATION_SUMMARY.md) - System overview
- [INTEGRATION_EXAMPLE.jsx](../dashboard/nidar-dashboard/INTEGRATION_EXAMPLE.jsx) - Dashboard integration example

## Success Checklist

- [ ] Pi Controller starts without errors
- [ ] Pixhawk telemetry module initializes
- [ ] Pi connects to server
- [ ] Server logs show telemetry messages
- [ ] Dashboard displays DroneTelemetry component
- [ ] Telemetry data updates in real-time
- [ ] GPS coordinates change over time (in simulation)
- [ ] Battery level shows correctly
- [ ] Connection status badge displays

Once all items are checked, the system is working correctly! 🎉
