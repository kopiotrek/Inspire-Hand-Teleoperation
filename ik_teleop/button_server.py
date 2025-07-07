#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float32
from flask import Flask, render_template_string, request
import threading
import signal
import sys
import time

# ROS Node Setup
rospy.init_node('button_web_server')
pub = rospy.Publisher('/activate_button', Bool, queue_size=1)
release_pub = rospy.Publisher('/release_button', Bool, queue_size=1)  # <-- New publisher

# Flask Web Server Setup
app = Flask(__name__)
button_state = False
grasp_enabled = False
grasp_start_time = None
grasp_above = False
release_state = False
current_grasp_value = 0.0
grasp_ignore_timestamp = 0
grasp_ignore_time_limit = 3

HTML_PAGE = '''
<!DOCTYPE html>
<html>
<head>
    <title>ROS Toggle Button</title>
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <style>
        body {
            font-family: sans-serif;
            text-align: center;
            margin-top: 100px;
            background-color: #f2f2f2;
        }
        h1 {
            font-size: 48px;
            margin-bottom: 60px;
        }
        .status {
            font-size: 64px;
            margin-bottom: 40px;
            color: {{ 'green' if grasped else 'darkred' }};
        }
        button {
            font-size: 48px;
            padding: 40px 80px;
            color: white;
            border: none;
            border-radius: 16px;
            cursor: pointer;
            margin: 20px;
            width: 80%%;
            max-width: 600px;
        }
        #toggleBtn {
            background-color: {{ '#28a745' if state else '#007BFF' }};
            opacity: {{ '1.0' if enabled else '0.5' }};
            cursor: {{ 'pointer' if enabled else 'not-allowed' }};
        }
        #releaseBtn {
            background-color: #dc3545;
        }
    </style>
</head>
<body>
    <div class="status" id="graspStatus">{{ 'GRASPED' if grasped else 'NOT GRASPED' }}</div>
    
    <button id="toggleBtn" onclick="toggleButton()" {{ 'disabled' if not enabled else '' }}>
        {{ 'ON' if state else 'OFF' }}
    </button>

    <button id="releaseBtn" onclick="releaseButton()">
        RELEASE
    </button>

    <p style="font-size: 24px; color: #333;" id="graspValue">Current grasp signal: {{ '%.3f'|format(grasp_value) }}</p>

    <script>
        function updateStatus() {
            fetch('/status')
                .then(response => response.json())
                .then(updateUI);
        }

        function toggleButton() {
            fetch('/toggle', { method: 'POST' })
                .then(response => response.json())
                .then(updateUI);
        }

        function releaseButton() {
            fetch('/release', { method: 'POST' })
                .then(response => response.json())
                .then(updateUI);
        }

        function updateUI(data) {
            document.getElementById('graspStatus').innerText = data.grasped ? 'GRASPED' : 'NOT GRASPED';
            document.getElementById('graspValue').innerText = `Current grasp signal: ${data.grasp_value.toFixed(3)}`;
            
            const toggleBtn = document.getElementById('toggleBtn');
            toggleBtn.disabled = !data.enabled;
            toggleBtn.style.cursor = data.enabled ? 'pointer' : 'not-allowed';
            toggleBtn.style.opacity = data.enabled ? '1.0' : '0.5';
            toggleBtn.style.backgroundColor = data.state ? '#28a745' : '#007BFF';
            toggleBtn.innerText = data.state ? 'ON' : 'OFF';

        }

        setInterval(updateStatus, 1000);
    </script>
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML_PAGE, state=button_state, enabled=grasp_enabled,
                                  grasped=grasp_enabled, grasp_value=current_grasp_value)

@app.route('/toggle', methods=['POST'])
def toggle():
    global button_state
    if grasp_enabled:
        button_state = not button_state
        pub.publish(Bool(data=button_state))
    return get_status_dict()

@app.route('/release', methods=['POST'])
def release():
    global button_state, grasp_enabled, grasp_ignore_timestamp, release_state

    print("[DEBUG] /release endpoint triggered")  # Add this
    release_pub.publish(Bool(data=True))
    button_state = False
    grasp_enabled = False
    release_state = True
    grasp_ignore_timestamp = time.time()

    return get_status_dict()


@app.route('/status')
def status():
    return get_status_dict()

def get_status_dict():
    return {
        'state': button_state,
        'enabled': grasp_enabled,
        'grasped': grasp_enabled,
        'grasp_value': current_grasp_value
    }

def grasp_callback(msg):
    global grasp_enabled, current_grasp_value, grasp_ignore_timestamp, release_state
    current_grasp_value = msg.data

    # Ignore grasp signal for 5 seconds after release
    if time.time() - grasp_ignore_timestamp < grasp_ignore_time_limit:
        return  # Ignore during the cooldown period

    # Now, allow re-enabling
    if msg.data > 0.8:
        grasp_enabled = True
        release_state = False  # Reset release state once grasp is re-enabled


rospy.Subscriber('/kth_franka_plant/in/grasp_signal', Float32, grasp_callback)

def run_flask():
    app.run(host='0.0.0.0', port=5000, use_reloader=False)

def shutdown_handler(sig, frame):
    print("Shutting down...")
    sys.exit(0)

if __name__ == '__main__':
    signal.signal(signal.SIGINT, shutdown_handler)
    flask_thread = threading.Thread(target=run_flask)
    flask_thread.daemon = True
    flask_thread.start()
    rospy.spin()
