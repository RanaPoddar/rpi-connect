#!/bin/bash
# Simple Detection Control Scripts for Raspberry Pi

echo "Creating detection control scripts..."

# Start detection script
cat > ~/start_detection.sh << 'EOF'
#!/bin/bash
echo "1" > /tmp/detection_flag
echo "🌾 Detection START signal sent"
echo "If pi_controller.py is running, detection should start"
EOF
chmod +x ~/start_detection.sh

# Stop detection script
cat > ~/stop_detection.sh << 'EOF'
#!/bin/bash
echo "0" > /tmp/detection_flag
echo "🛑 Detection STOP signal sent"
echo "If pi_controller.py is running, detection should stop"
EOF
chmod +x ~/stop_detection.sh

# Status script
cat > ~/detection_status.sh << 'EOF'
#!/bin/bash
if [ -f /tmp/detection_flag ]; then
    FLAG=$(cat /tmp/detection_flag)
    if [ "$FLAG" = "1" ]; then
        echo "📊 Detection Status: 🟢 ACTIVE"
    else
        echo "📊 Detection Status: 🔴 INACTIVE"
    fi
else
    echo "📊 Detection Status: ⚪ Unknown (flag file not found)"
fi

# Check if pi_controller is running
if pgrep -f "python3.*pi_controller.py" > /dev/null; then
    echo "✅ pi_controller.py is running"
else
    echo "❌ pi_controller.py is NOT running"
fi
EOF
chmod +x ~/detection_status.sh

echo "✅ Scripts created in home directory:"
echo "   ~/start_detection.sh"
echo "   ~/stop_detection.sh"
echo "   ~/detection_status.sh"
echo ""
echo "Usage:"
echo "   ./start_detection.sh   - Start detection"
echo "   ./stop_detection.sh    - Stop detection"
echo "   ./detection_status.sh  - Check status"
