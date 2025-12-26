# Raspberry Pi Remote Control

Remote control and monitoring system for Raspberry Pi with camera control, system monitoring, and **Pixhawk flight controller telemetry**.

## Features

- 🎥 **Camera Control**: Live streaming, image capture, camera testing
- 💻 **System Monitoring**: CPU temperature, usage stats, memory, disk
- 🚁 **Drone Telemetry**: Real-time telemetry from Pixhawk flight controller
- 🌐 **Remote Control**: Execute commands, reboot, restart services
- 🔌 **Socket.IO**: Real-time bidirectional communication

## New: Pixhawk Telemetry Integration

The system now supports real-time telemetry from Pixhawk flight controllers:

- GPS coordinates and altitude
- Attitude (roll, pitch, yaw)
- Flight mode and armed status
- Battery voltage and current
- Velocity and heading
- System health indicators

📖 **For detailed documentation, see [PIXHAWK_TELEMETRY.md](PIXHAWK_TELEMETRY.md)**

## Installation

### On Raspberry Pi

1. Clone this repository to your Pi:
```bash
cd /home/pi
git clone <repository-url> rpi-connect
cd rpi-connect
```

2. Run setup script:
```bash
chmod +x setup.sh
./setup.sh
```

3. Configure server connection:
```bash
nano config.json
```
Set your server URL and Pi ID.

4. Test camera:
```bash
python3 test_camera.py
```

## Usage

### Manual Start
```bash
python3 pi_controller.py
```

### Service Control
```bash
# Start service
sudo systemctl start pi-controller

# Stop service
sudo systemctl stop pi-controller

# Enable on boot
sudo systemctl enable pi-controller

# Check status
sudo systemctl status pi-controller

# View logs
sudo journalctl -u pi-controller -f
```

## Configuration

Edit `config.json`:

```json
{
  "server_url": "http://YOUR_SERVER_IP:3000",
  "pi_id": "pi_001",
  "camera": {
    "enabled": true,
    "resolution": [1920, 1080],
    "framerate": 30,
    "quality": 85
  }
}
```

## Available Commands

Commands sent from server:

- `camera_test` - Test camera availability
- `capture_image` - Capture single image
- `reboot` - Reboot Pi
- `shutdown` - Shutdown Pi
- `restart_service` - Restart a system service

## Socket.IO Events

### Emitted by Pi

- `pi_register` - Register Pi with server
- `system_stats` - Send system statistics
- `camera_frame` - Stream video frame
- `command_result` - Result of executed command
- `stream_status` - Streaming status update
- `pong` - Response to ping

### Received by Pi

- `execute_command` - Execute a command
- `request_stats` - Request system stats
- `start_stream` - Start camera stream
- `stop_stream` - Stop camera stream
- `ping` - Ping request

## Troubleshooting

### Camera not working
```bash
# Enable camera
sudo raspi-config nonint do_camera 0
sudo reboot

# Test camera
raspistill -o test.jpg
```

### Service not starting
```bash
# Check logs
sudo journalctl -u pi-controller -n 50

# Check permissions
ls -la /home/pi/rpi-connect/

# Test manually
python3 pi_controller.py
```

### Connection issues
- Verify server URL in config.json
- Check firewall settings
- Ensure server is running
- Check network connectivity: `ping SERVER_IP`

## Requirements

- Raspberry Pi (tested on Pi 4)
- Raspberry Pi Camera Module
- Python 3.7+
- Network connection

## Security Notes

- Use HTTPS/WSS in production
- Implement authentication on server
- Restrict command execution
- Use VPN for remote access
- Keep system updated

## License

MIT License
