#!/usr/bin/env python3
"""
WebRTC Stream Server for Raspberry Pi
Handles WebRTC signaling and peer connections
"""

import asyncio
import json
import logging
from aiohttp import web
import socketio
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaPlayer
from av import VideoFrame
import numpy as np
import cv2
import time

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

# Store active peer connections
peer_connections = {}

class PiCameraTrack(VideoStreamTrack):
    """
    Custom video track from Raspberry Pi camera
    """
    
    def __init__(self):
        super().__init__()
        self.camera = None
        self.is_running = False
        
    async def start(self):
        """Start camera capture"""
        try:
            # Try to open Pi camera
            self.camera = cv2.VideoCapture(0)
            
            # Set camera properties
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            self.camera.set(cv2.CAP_PROP_FPS, 30)
            
            self.is_running = True
            logger.info("Camera started successfully")
            return True
        except Exception as e:
            logger.error(f"Failed to start camera: {e}")
            return False
    
    async def recv(self):
        """Receive next video frame"""
        pts, time_base = await self.next_timestamp()
        
        if not self.camera or not self.is_running:
            # Return blank frame if camera not available
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
            new_frame = VideoFrame.from_ndarray(frame, format='bgr24')
            new_frame.pts = pts
            new_frame.time_base = time_base
            return new_frame
        
        ret, frame = self.camera.read()
        
        if not ret:
            # Return blank frame on error
            frame = np.zeros((720, 1280, 3), dtype=np.uint8)
        
        # Convert to VideoFrame
        new_frame = VideoFrame.from_ndarray(frame, format='bgr24')
        new_frame.pts = pts
        new_frame.time_base = time_base
        
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
        </script>
    </body>
    </html>
    """
    return web.Response(text=html, content_type='text/html')

# Create aiohttp application
app = web.Application()
sio.attach(app)

# Add routes
app.router.add_get('/', index)

def main():
    """Run the server"""
    port = 8080
    logger.info(f"Starting WebRTC server on port {port}")
    logger.info(f"Open http://localhost:{port} in your browser")
    
    web.run_app(app, host='0.0.0.0', port=port)

if __name__ == '__main__':
    main()
