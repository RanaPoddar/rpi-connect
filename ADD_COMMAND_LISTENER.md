# Pi MAVLink Command Listener Fix

## Add this to pi_controller.py in the message_listener_loop()

```python
# Around line 1075 in message_listener_loop(), add after the STATUSTEXT handler:

elif msg_type == 'COMMAND_LONG':
    # Handle MAVLink commands from GCS (long-range control via F10 radio)
    command = msg.command
    
    if command == 42000:  # Start detection command
        self.detection_active = True
        print("🌾 Detection started via MAVLink command 42000")
        self.mavlink_detection_sender.send_stats(
            total_detections=self.detection_count,
            active_status='ACTIVE',
            mission_id=self.current_mission_id or 'MANUAL'
        )
        
        # Send ACK back to GCS
        try:
            self.vehicle.mav.command_ack_send(
                command,  # command that was received
                mavutil.mavlink.MAV_RESULT_ACCEPTED,  # result
                0,  # progress
                0,  # result_param2
                self.vehicle.target_system,  # target_system
                self.vehicle.target_component  # target_component
            )
            print("   ✅ Sent COMMAND_ACK (ACCEPTED)")
        except Exception as e:
            print(f"   ⚠️ Failed to send ACK: {e}")
    
    elif command == 42001:  # Stop detection command
        self.detection_active = False
        print("🛑 Detection stopped via MAVLink command 42001")
        self.mavlink_detection_sender.send_stats(
            total_detections=self.detection_count,
            active_status='INACTIVE',
            mission_id=self.current_mission_id or 'MANUAL'
        )
        
        # Send ACK back to GCS
        try:
            self.vehicle.mav.command_ack_send(
                command,
                mavutil.mavlink.MAV_RESULT_ACCEPTED,
                0, 0,
                self.vehicle.target_system,
                self.vehicle.target_component
            )
            print("   ✅ Sent COMMAND_ACK (ACCEPTED)")
        except Exception as e:
            print(f"   ⚠️ Failed to send ACK: {e}")
    
    else:
        # Unknown command - send UNSUPPORTED
        try:
            self.vehicle.mav.command_ack_send(
                command,
                mavutil.mavlink.MAV_RESULT_UNSUPPORTED,
                0, 0,
                self.vehicle.target_system,
                self.vehicle.target_component
            )
            print(f"   ⚠️ Unknown MAVLink command: {command} (sent UNSUPPORTED)")
        except Exception as e:
            pass
```

## Where to Add

1. Open: `rpi-connect/pi_controller.py`
2. Find: `message_listener_loop()` function (around line 1048)
3. Locate the message type checking section:
   ```python
   if msg_type == 'HEARTBEAT':
       # ... heartbeat handling
   elif msg_type == 'STATUSTEXT':
       # ... statustext handling
   ```
4. **Add the COMMAND_LONG handler after STATUSTEXT**

## Test After Adding

### On Pi Terminal:
```bash
# Restart Pi controller
cd ~/rpi-connect
python3 pi_controller.py
```

### On GCS:
1. Open Mission Control: `http://localhost:3000/mission_control.html`
2. Click "Start Detection (D1)"
3. **Watch Pi logs for**: `🌾 Detection started via MAVLink command 42000`
4. **Watch GCS logs for**: `📡 Sent MAVLink command: Start Detection`

### Expected Flow:
```
Mission Control Button Click
  → POST /drone/1/pi/start_detection
  → PyMAVLink sends COMMAND_LONG(42000)
  → F10 Radio transmits
  → Pixhawk forwards to Pi (ttyAMA0)
  → Pi receives COMMAND_LONG
  → Pi sets detection_active = True
  → Pi sends COMMAND_ACK
  → Radio transmits ACK back
  → GCS receives ACK confirmation
```

## Quick Reference

| Command | ID | Action |
|---------|-----|---------|
| START_DETECTION | 42000 | Sets `detection_active = True` |
| STOP_DETECTION | 42001 | Sets `detection_active = False` |

## Important Notes

- ✅ Commands work up to 10km range (F10 RC range)
- ✅ No WiFi needed  
- ✅ ACK confirms command received
- ⚠️ Ensure Pi MAVLink connection is stable
- ⚠️ Check `read_only = true` still allows command reception (it should - commands are just data)

---

*Created: January 13, 2026*
