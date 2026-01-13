#!/bin/bash
# Pi Quick Test Script - Run on Raspberry Pi

echo "================================================"
echo "  Pi Restart & Test (Telemetry-Only Mode)"
echo "================================================"
echo ""

echo "1. Stopping old Pi controller..."
pkill -f pi_controller.py
sleep 2

echo ""
echo "2. Starting Pi controller..."
cd ~/rpi-connect
python3 pi_controller.py &
PI_PID=$!

echo ""
echo "3. Waiting for startup (15 seconds)..."
sleep 5

echo ""
echo "Expected logs:"
echo "  ✅ Running in telemetry-only mode"
echo "  🚁 Initializing Pixhawk telemetry..."
echo "  ✅ Pixhawk telemetry initialized"
echo "  📡 MAVLink command handler registered (42000/42001)"
echo "  📡 Command callback registered for detection control"
echo ""
echo "You should NOT see:"
echo "  ❌ MAVLink receive error"
echo "  ❌ Attempting to use a port that is not open"
echo ""

sleep 10

echo "4. Checking process..."
if ps -p $PI_PID > /dev/null; then
    echo "✅ Pi controller running (PID: $PI_PID)"
else
    echo "❌ Pi controller crashed!"
    exit 1
fi

echo ""
echo "5. Press Ctrl+C to stop monitoring..."
echo ""
tail -f /dev/null
