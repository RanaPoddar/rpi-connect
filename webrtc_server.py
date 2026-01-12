#!/usr/bin/env python3
"""
WebRTC Stream Server for Raspberry Pi
Handles WebRTC signaling and peer connections
"""

import asyncio
import json
import logging
import fractions
import os
from aiohttp import web
import socketio
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaPlayer
from av import VideoFrame
import numpy as np
import cv2
import time

# Import detection modules
try:
    from modules.yellow_crop_detector import YellowCropDetector
    from modules.mavlink_detection_sender import MAVLinkDetectionSender
    from pymavlink import mavutil
    DETECTION_AVAILABLE = True
except ImportError as e:
    DETECTION_AVAILABLE = False
    print(f"Detection modules not available: {e}")

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize Socket.IO server
sio = socketio.AsyncServer(
    async_mode='aiohttp',
    cors_allowed_origins='*',
    ping_timeout=60,
    ping_interval=25
)

# Store active peer connections and detection state
peer_connections = {}
detection_enabled = False
detector = None
mavlink_connection = None
detection_sender = None

class PiCameraTrack(VideoStreamTrack):
    """
    Custom video track from Raspberry Pi camera with optional detection overlay
    """
    
    def __init__(self, enable_detection=False):
        super().__init__()
        self.camera = None
        self.is_running = False
        self._timestamp = 0
        self._start_time = time.time()
        self.enable_detection = enable_detection
        self.frame_skip = 0  # Skip every other frame for better performance
        
    async def start(self):
        """Start camera capture"""
        try:
            # Try to open Pi camera
            self.camera = cv2.VideoCapture(0)
            
            # Set camera properties - reduced resolution for better streaming
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.camera.set(cv2.CAP_PROP_FPS, 20)
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self.camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            
            self.is_running = True
            self._start_time = time.time()
            logger.info("Camera started successfully (640x480@20fps)")
            return True
        except Exception as e:
            logger.error(f"Failed to start camera: {e}")
            return False
    
    async def recv(self):
        """Receive next video frame"""
        if not self.camera or not self.is_running:
            # Return blank frame if camera not available
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            new_frame = VideoFrame.from_ndarray(frame, format='bgr24')
            new_frame.pts = self._timestamp
            new_frame.time_base = fractions.Fraction(1, 20)
            self._timestamp += 1
            await asyncio.sleep(1/20)  # 20 FPS
            return new_frame
        
        # Read frame from camera
        ret, frame = self.camera.read()
        
        if not ret or frame is None:
            logger.warning("Failed to read frame from camera")
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
        else:
            # Apply detection overlay if enabled
            if self.enable_detection and detector and detection_enabled:
                try:
                    detections = detector.detect(frame)
                    if detections:
                        for det in detections:
                            x, y, w, h = det.bbox
                            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                            cv2.putText(frame, f"{det.confidence:.2f}", (x, y-5),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                except Exception as e:
                    logger.error(f"Detection error: {e}")
        
        # Convert to VideoFrame
        new_frame = VideoFrame.from_ndarray(frame, format='bgr24')
        new_frame.pts = self._timestamp
        new_frame.time_base = fractions.Fraction(1, 20)
        self._timestamp += 1
        
        # Maintain frame rate
        await asyncio.sleep(1/20)  # 20 FPS
        
        return new_frame
    
    def stop(self):
        """Stop camera capture"""
        self.is_running = False
        if self.camera:
            self.camera.release()
            self.camera = None
        logger.info("Camera stopped")

# Socket.IO event handlers
@sio.event
async def connect(sid, environ):
    """Client connected"""
    logger.info(f"Client connected: {sid}")
    await sio.emit('connection_status', {'status': 'connected'}, room=sid)

@sio.event
async def disconnect(sid):
    """Client disconnected"""
    logger.info(f"Client disconnected: {sid}")
    
    # Clean up peer connection if exists
    if sid in peer_connections:
        pc = peer_connections[sid]
        await pc.close()
        del peer_connections[sid]

@sio.on('offer')
async def handle_offer(sid, data):
    """Handle WebRTC offer from client"""
    logger.info(f"Received offer from {sid}")
    
    try:
        # Create peer connection
        pc = RTCPeerConnection()
        peer_connections[sid] = pc
        
        # Create camera track
        video_track = PiCameraTrack()
        await video_track.start()
        
        # Add track to peer connection
        pc.addTrack(video_track)
        
        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            logger.info(f"Connection state: {pc.connectionState}")
            
            if pc.connectionState == "failed" or pc.connectionState == "closed":
                video_track.stop()
                if sid in peer_connections:
                    del peer_connections[sid]
        
        # Set remote description
        offer = RTCSessionDescription(
            sdp=data['sdp'],
            type=data['type']
        )
        await pc.setRemoteDescription(offer)
        
        # Create answer
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)
        
        # Send answer to client
        await sio.emit('answer', {
            'sdp': pc.localDescription.sdp,
            'type': pc.localDescription.type
        }, room=sid)
        
        logger.info(f"Sent answer to {sid}")
        
    except Exception as e:
        logger.error(f"Error handling offer: {e}")
        await sio.emit('error', {'message': str(e)}, room=sid)

@sio.on('ice_candidate')
async def handle_ice_candidate(sid, data):
    """Handle ICE candidate from client"""
    logger.info(f"Received ICE candidate from {sid}")
    
    try:
        if sid in peer_connections:
            pc = peer_connections[sid]
            candidate = data.get('candidate')
            
            if candidate:
                await pc.addIceCandidate(candidate)
                logger.info("Added ICE candidate")
    
    except Exception as e:
        logger.error(f"Error handling ICE candidate: {e}")

@sio.on('start_stream')
async def handle_start_stream(sid, data):
    """Start streaming"""
    logger.info(f"Start stream request from {sid}")
    await sio.emit('stream_status', {'status': 'ready'}, room=sid)

@sio.on('stop_stream')
async def handle_stop_stream(sid, data):
    """Stop streaming"""
    logger.info(f"Stop stream request from {sid}")
    
    if sid in peer_connections:
        pc = peer_connections[sid]
        await pc.close()
        del peer_connections[sid]
    
    await sio.emit('stream_status', {'status': 'stopped'}, room=sid)

@sio.on('toggle_detection')
async def handle_toggle_detection(sid, data):
    """Toggle detection on/off"""
    global detection_enabled
    
    if not DETECTION_AVAILABLE:
        await sio.emit('detection_status', {
            'enabled': False,
            'error': 'Detection modules not available'
        }, room=sid)
        return
    
    detection_enabled = data.get('enabled', False)
    logger.info(f"Detection {'enabled' if detection_enabled else 'disabled'}")
    
    await sio.emit('detection_status', {
        'enabled': detection_enabled,
        'message': f"Detection {'enabled' if detection_enabled else 'disabled'}"
    }, room=sid)

@sio.on('toggle_mavlink_detection')
async def handle_toggle_mavlink_detection(sid, data):
    """Toggle MAVLink detection transmission"""
    global detection_sender
    
    if not DETECTION_AVAILABLE or not detection_sender:
        await sio.emit('mavlink_status', {
            'enabled': False,
            'error': 'MAVLink detection not available'
        }, room=sid)
        return
    
    enabled = data.get('enabled', False)
    detection_sender.enabled = enabled
    logger.info(f"MAVLink detection {'enabled' if enabled else 'disabled'}")
    
    await sio.emit('mavlink_status', {
        'enabled': enabled,
        'message': f"MAVLink detection {'enabled' if enabled else 'disabled'}"
    }, room=sid)

# HTTP routes
async def index(request):
    """Serve index page"""
    html = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Pi WebRTC Stream</title>
        <style>
            body {
                font-family: Arial, sans-serif;
                max-width: 1200px;
                margin: 0 auto;
                padding: 20px;
                background: #1a1a1a;
                color: #fff;
            }
            h1 { color: #4CAF50; }
            #video-container {
                position: relative;
                width: 100%;
                max-width: 1280px;
                margin: 20px auto;
            }
            video {
                width: 100%;
                height: auto;
                background: #000;
                border-radius: 8px;
            }
            .controls {
                text-align: center;
                margin: 20px 0;
            }
            button {
                padding: 12px 24px;
                margin: 5px;
                font-size: 16px;
                border: none;
                border-radius: 4px;
                cursor: pointer;
                transition: all 0.3s;
            }
            .btn-primary {
                background: #4CAF50;
                color: white;
            }
            .btn-primary:hover { background: #45a049; }
            .btn-danger {
                background: #f44336;
                color: white;
            }
            .btn-danger:hover { background: #da190b; }
            .status {
                padding: 10px;
                margin: 10px 0;
                border-radius: 4px;
                text-align: center;
            }
            .status.connected { background: #4CAF50; }
            .status.disconnected { background: #f44336; }
            .status.connecting { background: #ff9800; }
            .detection-panel {
                background: #2a2a2a;
                padding: 20px;
                border-radius: 8px;
                margin: 20px 0;
            }
            .detection-panel h3 {
                margin-top: 0;
                color: #4CAF50;
            }
            .switch-container {
                display: flex;
                align-items: center;
                margin: 10px 0;
                gap: 10px;
            }
            .switch {
                position: relative;
                display: inline-block;
                width: 50px;
                height: 24px;
            }
            .switch input {
                opacity: 0;
                width: 0;
                height: 0;
            }
            .slider {
                position: absolute;
                cursor: pointer;
                top: 0;
                left: 0;
                right: 0;
                bottom: 0;
                background-color: #ccc;
                transition: .4s;
                border-radius: 24px;
            }
            .slider:before {
                position: absolute;
                content: "";
                height: 18px;
                width: 18px;
                left: 3px;
                bottom: 3px;
                background-color: white;
                transition: .4s;
                border-radius: 50%;
            }
            input:checked + .slider {
                background-color: #4CAF50;
            }
            input:checked + .slider:before {
                transform: translateX(26px);
            }
        </style>
    </head>
    <body>
        <h1>🎥 Raspberry Pi WebRTC Stream</h1>
        
        <div id="status" class="status disconnected">Disconnected</div>
        
        <div id="video-container">
            <video id="video" autoplay playsinline></video>
        </div>
        
        <div class="controls">
            <button class="btn-primary" onclick="startStream()">Start Stream</button>
            <button class="btn-danger" onclick="stopStream()">Stop Stream</button>
        </div>
        
        <div class="detection-panel">
            <h3>🌾 Detection & Telemetry</h3>
            
            <div class="switch-container">
                <label class="switch">
                    <input type="checkbox" id="detectionToggle" onchange="toggleDetection()">
                    <span class="slider"></span>
                </label>
                <label for="detectionToggle">Enable Yellow Crop Detection</label>
            </div>
            
            <div class="switch-container">
                <label class="switch">
                    <input type="checkbox" id="mavlinkToggle" onchange="toggleMavlinkDetection()">
                    <span class="slider"></span>
                </label>
                <label for="mavlinkToggle">Send Detection via MAVLink Telemetry</label>
            </div>
            
            <div id="detectionStatus" style="margin-top: 10px; padding: 5px; font-size: 14px;"></div>
        </div>
        
        <script src="/socket.io/socket.io.js"></script>
        <script>
            const socket = io();
            let peerConnection = null;
            const video = document.getElementById('video');
            const status = document.getElementById('status');
            
            // ICE servers configuration
            const config = {
                iceServers: [
                    { urls: 'stun:stun.l.google.com:19302' },
                    { urls: 'stun:stun1.l.google.com:19302' }
                ]
            };
            
            // Update status
            function updateStatus(message, type) {
                status.textContent = message;
                status.className = 'status ' + type;
            }
            
            // Socket.IO events
            socket.on('connect', () => {
                console.log('Connected to server');
                updateStatus('Connected to server', 'connected');
            });
            
            socket.on('disconnect', () => {
                console.log('Disconnected from server');
                updateStatus('Disconnected', 'disconnected');
            });
            
            socket.on('answer', async (data) => {
                console.log('Received answer');
                try {
                    await peerConnection.setRemoteDescription(new RTCSessionDescription(data));
                    console.log('Remote description set');
                } catch (error) {
                    console.error('Error setting remote description:', error);
                }
            });
            
            socket.on('stream_status', (data) => {
                console.log('Stream status:', data);
                if (data.status === 'stopped') {
                    updateStatus('Stream stopped', 'disconnected');
                }
            });
            
            // Start streaming
            async function startStream() {
                try {
                    updateStatus('Starting stream...', 'connecting');
                    
                    // Create peer connection
                    peerConnection = new RTCPeerConnection(config);
                    
                    // Handle incoming stream
                    peerConnection.ontrack = (event) => {
                        console.log('Received remote track');
                        video.srcObject = event.streams[0];
                        updateStatus('Streaming', 'connected');
                    };
                    
                    // Handle ICE candidates
                    peerConnection.onicecandidate = (event) => {
                        if (event.candidate) {
                            socket.emit('ice_candidate', { candidate: event.candidate });
                        }
                    };
                    
                    // Handle connection state
                    peerConnection.onconnectionstatechange = () => {
                        console.log('Connection state:', peerConnection.connectionState);
                        if (peerConnection.connectionState === 'connected') {
                            updateStatus('Streaming', 'connected');
                        } else if (peerConnection.connectionState === 'failed') {
                            updateStatus('Connection failed', 'disconnected');
                        }
                    };
                    
                    // Create offer
                    const offer = await peerConnection.createOffer();
                    await peerConnection.setLocalDescription(offer);
                    
                    // Send offer to server
                    socket.emit('offer', {
                        sdp: offer.sdp,
                        type: offer.type
                    });
                    
                } catch (error) {
                    console.error('Error starting stream:', error);
                    updateStatus('Error: ' + error.message, 'disconnected');
                }
            }
            
            // Stop streaming
            function stopStream() {
                if (peerConnection) {
                    peerConnection.close();
                    peerConnection = null;
                }
                video.srcObject = null;
                socket.emit('stop_stream');
                updateStatus('Stream stopped', 'disconnected');
            }
            
            // Toggle detection
            function toggleDetection() {
                const enabled = document.getElementById('detectionToggle').checked;
                socket.emit('toggle_detection', { enabled: enabled });
            }
            
            // Toggle MAVLink detection
            function toggleMavlinkDetection() {
                const enabled = document.getElementById('mavlinkToggle').checked;
                socket.emit('toggle_mavlink_detection', { enabled: enabled });
            }
            
            // Listen for detection status
            socket.on('detection_status', (data) => {
                const statusDiv = document.getElementById('detectionStatus');
                if (data.error) {
                    statusDiv.textContent = '❌ ' + data.error;
                    statusDiv.style.color = '#f44336';
                } else {
                    statusDiv.textContent = '✓ ' + data.message;
                    statusDiv.style.color = '#4CAF50';
                }
            });
            
            // Listen for MAVLink status
            socket.on('mavlink_status', (data) => {
                const statusDiv = document.getElementById('detectionStatus');
                if (data.error) {
                    statusDiv.textContent = '❌ ' + data.error;
                    statusDiv.style.color = '#f44336';
                } else {
                    statusDiv.textContent = '📡 ' + data.message;
                    statusDiv.style.color = '#4CAF50';
                }
            });
        </script>
    </body>
    </html>
    """
    return web.Response(text=html, content_type='text/html')

def init_detection_system():
    """Initialize detection system if available"""
    global detector, mavlink_connection, detection_sender
    
    if not DETECTION_AVAILABLE:
        logger.warning("Detection system not available")
        return
    
    try:
        # Load config
        config_path = os.path.join(os.path.dirname(__file__), 'config.json')
        if os.path.exists(config_path):
            with open(config_path, 'r') as f:
                config = json.load(f)
        else:
            config = {}
        
        # Initialize detector
        detector = YellowCropDetector(config.get('detection', {}))
        logger.info("✓ Yellow crop detector initialized")
        
        # Initialize MAVLink connection if enabled
        pixhawk_config = config.get('pixhawk', {})
        if pixhawk_config.get('enabled', False):
            try:
                connection_string = pixhawk_config.get('connection_string', '/dev/serial0')
                baud_rate = pixhawk_config.get('baud_rate', 921600)
                mavlink_connection = mavutil.mavlink_connection(connection_string, baud=baud_rate)
                mavlink_connection.wait_heartbeat(timeout=5)
                
                # Initialize detection sender
                mavlink_enabled = config.get('mavlink_detection', {}).get('enabled', True)
                detection_sender = MAVLinkDetectionSender(mavlink_connection, enabled=mavlink_enabled)
                logger.info("✓ MAVLink detection sender initialized")
            except Exception as e:
                logger.warning(f"MAVLink not available: {e}")
    except Exception as e:
        logger.error(f"Failed to initialize detection system: {e}")

# Create aiohttp application
app = web.Application()
sio.attach(app)

# Add routes
app.router.add_get('/', index)

def main():
    """Run the server"""
    # Initialize detection system
    init_detection_system()
    
    port = 8080
    logger.info(f"Starting WebRTC server on port {port}")
    logger.info(f"Detection available: {DETECTION_AVAILABLE}")
    logger.info(f"Open http://localhost:{port} in your browser")
    
    web.run_app(app, host='0.0.0.0', port=port)

if __name__ == '__main__':
    main()
