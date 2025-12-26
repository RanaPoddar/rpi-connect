#!/bin/bash
# Setup script for Raspberry Pi Controller

echo "=========================================="
echo "Raspberry Pi Controller Setup"
echo "=========================================="

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/device-tree/model 2>/dev/null; then
    echo "Warning: This doesn't appear to be a Raspberry Pi"
    read -p "Continue anyway? (y/n) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Update system
echo ""
echo "Updating system packages..."
sudo apt-get update

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
pip3 install -r requirements.txt

# Enable camera
echo ""
echo "Enabling camera..."
sudo raspi-config nonint do_camera 0

# Create systemd service
echo ""
echo "Creating systemd service..."
sudo tee /etc/systemd/system/pi-controller.service > /dev/null <<EOF
[Unit]
Description=Raspberry Pi Controller Service
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$(pwd)
ExecStart=/usr/bin/python3 $(pwd)/pi_controller.py
Restart=always
RestartSec=10
Environment="SERVER_URL=http://localhost:3000"
Environment="PI_ID=pi_001"

[Install]
WantedBy=multi-user.target
EOF

# Reload systemd
sudo systemctl daemon-reload

echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "1. Edit config.json to set your server URL"
echo "2. Test camera: python3 test_camera.py"
echo "3. Start service: sudo systemctl start pi-controller"
echo "4. Enable on boot: sudo systemctl enable pi-controller"
echo "5. Check status: sudo systemctl status pi-controller"
echo ""
