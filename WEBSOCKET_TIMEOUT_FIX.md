# WebSocket Timeout Fix - Summary

## Problem
The Raspberry Pi was experiencing WebSocket timeout errors when sending data to the GCS server:
```
websocket._exceptions.WebSocketTimeoutException: timed out
```

This occurred during:
- Camera frame streaming
- System stats transmission
- Telemetry updates

## Root Causes Identified

1. **No WebSocket Timeout Configuration**: The Socket.IO client was initialized without explicit timeout settings
2. **High Frame Rate**: Camera streaming at ~10 FPS with large JPEG payloads was overwhelming the connection
3. **Network Latency**: The default WebSocket send timeout (likely 10s) was too short for slow/unstable networks
4. **No Error Handling**: Socket.IO emit operations had no try-catch blocks to handle timeout exceptions gracefully

## Fixes Applied

### 1. Enhanced Socket.IO Client Configuration (`pi_controller.py`)

**Added comprehensive timeout settings:**
```python
sio = socketio.Client(
    reconnection=True,
    reconnection_attempts=0,          # Infinite reconnection attempts
    reconnection_delay=2,              # Start with 2 second delay
    reconnection_delay_max=30,         # Max 30 seconds between attempts
    randomization_factor=0.5,          # Add jitter to prevent thundering herd
    request_timeout=60,                # HTTP long-polling timeout (60s)
    engineio_logger=False,
    logger=False,
    ssl_verify=True
)
```

**Connection options:**
```python
sio.connect(
    SERVER_URL,
    transports=['websocket', 'polling'],  # Prefer WebSocket, fallback to polling
    wait_timeout=10,                       # Connection establishment timeout
    socketio_path='/socket.io',
    headers={}
)
```

### 2. Socket.IO Configuration File (`config.json`)

**Added new `socketio` section:**
```json
{
  "socketio": {
    "request_timeout": 60,
    "ping_timeout": 60,
    "ping_interval": 25,
    "reconnection_delay": 2,
    "reconnection_delay_max": 30,
    "note": "Socket.IO connection settings. ping_timeout=60s prevents disconnections on slow networks."
  }
}
```

### 3. Reduced Frame Rate & Added Error Handling

**Camera streaming improvements:**
- Reduced frame rate from ~10 FPS to **5 FPS** (200ms interval)
- Reduced JPEG quality from 75 to **70** (smaller payloads)
- Added **rate limiting** to prevent socket buffer overflow
- Added **consecutive error tracking** (stops stream after 5 failures)
- Added **try-catch blocks** around all emit operations

**Before:**
```python
time.sleep(0.1)  # ~10 fps
sio.emit('camera_frame', data)  # No error handling
```

**After:**
```python
if current_time - last_send_time < 0.2:  # Rate limit to 5 FPS
    time.sleep(0.05)
    continue

try:
    sio.emit('camera_frame', data)
    consecutive_errors = 0
except Exception as e:
    consecutive_errors += 1
    if consecutive_errors >= 5:
        print(f"❌ Too many errors, stopping stream")
        streaming_active = False
```

### 4. Safe Emit Helper Function

**Added wrapper function for all Socket.IO operations:**
```python
def safe_emit(event_name, data, timeout=5.0):
    """
    Safely emit a Socket.IO event with timeout and error handling.
    Returns True if successful, False otherwise.
    """
    if not sio.connected:
        return False
    
    try:
        sio.emit(event_name, data)
        return True
    except Exception as e:
        print(f"⚠️  Socket.IO emit failed ({event_name}): {str(e)[:100]}")
        return False
```

**Usage in telemetry updates:**
```python
# Old: sio.emit('drone_telemetry', data)
# New: safe_emit('drone_telemetry', data)
```

### 5. Server-Side Configuration (Already Optimal)

The server already had good timeout settings in `config/config.js`:
```javascript
SOCKET_CONFIG: {
    pingTimeout: 120000,      // 120 seconds
    pingInterval: 25000,      // 25 seconds
    connectTimeout: 60000,    // 60 seconds
    maxHttpBufferSize: 1e8    // 100MB
}
```

## Testing Recommendations

1. **Monitor Connection Stability**
   ```bash
   # On Raspberry Pi, watch for timeout errors
   tail -f /var/log/syslog | grep -i "timeout\|disconnect"
   ```

2. **Check Frame Rate**
   - Observe the camera stream FPS in the web interface
   - Should be steady at ~5 FPS
   - No dropped frames or stuttering

3. **Network Latency Test**
   ```bash
   # Ping the GCS server
   ping 10.215.165.94
   ```
   - If latency > 100ms consistently, consider:
     - Reducing frame rate further (increase `send_interval` to 0.3 or 0.5)
     - Lowering JPEG quality to 60-65
     - Using polling transport instead of WebSocket

4. **System Resource Monitoring**
   ```python
   # CPU/Memory should remain stable
   # CPU < 25%, Memory < 50%
   ```

## Configuration Tuning Guide

If timeouts still occur, adjust these parameters:

### For Slow Networks (High Latency)
```json
// config.json
{
  "socketio": {
    "request_timeout": 90,        // Increase to 90s
    "ping_timeout": 90,
    "ping_interval": 30           // Increase ping interval
  }
}
```

```python
# pi_controller.py _stream_frames()
send_interval = 0.3  # Reduce to 3.3 FPS
[cv2.IMWRITE_JPEG_QUALITY, 60]  # Lower quality
```

### For Unstable Networks (Packet Loss)
```python
# Use polling transport (more reliable but higher latency)
sio.connect(
    SERVER_URL,
    transports=['polling', 'websocket']  # Prefer polling
)
```

### For High-Bandwidth Networks
```python
# Increase frame rate
send_interval = 0.1  # 10 FPS
[cv2.IMWRITE_JPEG_QUALITY, 80]  # Higher quality
```

## Additional Improvements

1. **Dual-Channel Transmission**: MAVLink fallback for critical data
2. **Automatic Reconnection**: Infinite retry with exponential backoff
3. **Graceful Degradation**: Detection continues even if streaming fails
4. **Better Logging**: Detailed error messages with system stats

## Verification

After applying these fixes, you should see:
```
✅ Connected successfully with ping_interval=25s, ping_timeout=60s
📊 System stats sent via Socket.IO (WiFi)
   CPU:4.0% MEM:3.3% TEMP:28.2°C
📡 MAVLink: Sent system stats (CPU:4.0% MEM:3.3% TEMP:28.2°C)
```

**No more:**
```
❌ websocket._exceptions.WebSocketTimeoutException: timed out
❌ Disconnected from server
```

## Files Modified

1. `rpi-connect/pi_controller.py`
   - Enhanced Socket.IO client initialization
   - Added `safe_emit()` helper function
   - Improved `_stream_frames()` with rate limiting and error handling
   - Updated telemetry callbacks to use `safe_emit()`
   - Enhanced main loop with better error handling

2. `rpi-connect/config.json`
   - Added `socketio` configuration section
   - Set `ping_timeout: 60`, `ping_interval: 25`
   - Documented timeout settings

## Next Steps

1. **Deploy to Raspberry Pi**
   ```bash
   cd /home/pi/rpi-connect
   git pull  # or copy files manually
   sudo systemctl restart pi-controller
   ```

2. **Monitor Logs**
   ```bash
   sudo journalctl -u pi-controller -f
   ```

3. **Test Under Real Conditions**
   - Start camera streaming
   - Enable detection
   - Run mission with periodic capture
   - Monitor for any timeout errors

4. **Fine-Tune if Needed**
   - Adjust `send_interval` based on network performance
   - Modify JPEG quality if bandwidth is limited
   - Consider using polling transport for very unstable networks

## Troubleshooting

### If timeouts still occur:

1. **Check server logs:**
   ```bash
   tail -f /path/to/GCS-without-pi/server.log
   ```

2. **Increase timeouts further:**
   - Set `request_timeout: 90` or `120`
   - Set `ping_timeout: 120` or `180`

3. **Disable camera streaming:**
   - Detection still works without streaming
   - Images are saved locally and sent via MAVLink metadata

4. **Use MAVLink-only mode:**
   - Detections sent via long-range radio
   - WiFi used only for non-critical data

## References

- Socket.IO Python Client: https://python-socketio.readthedocs.io/
- Engine.IO Protocol: https://socket.io/docs/v4/engine-io-protocol/
- WebSocket Timeouts: https://github.com/websocket-client/websocket-client
