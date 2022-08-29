#! /usr/bin/env
from flask import Flask, render_template
from flask_socketio import SocketIO
import rospy
from geometry_msgs.msg import Twist

cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret'
socketio = SocketIO(app)

@socketio.on('cmd_vel')
def handle_cmd_vel(msg):
    print('received message: ' + str(msg))
    twist_msg = Twist()
    twist_msg.linear.x = msg['x']
    twist_msg.angular.z = msg['theta']
    cmd_vel_pub.publish(twist_msg)

@app.route('/')
def index():
    return """
    <html>
        <head>
            <script src="//cdnjs.cloudflare.com/ajax/libs/socket.io/2.2.0/socket.io.js" integrity="sha256-yr4fRk/GU1ehYJPAs8P4JlTgu0Hdsp4ZKrx8bDEDC3I=" crossorigin="anonymous"></script>
            <script src="//github.com/bobboteck/JoyStick/releases/download/v1.1.4/joy.min.js"></script>
        </head>
        <body>
            <div id="joyDiv" style="width:200px;height:200px;margin-bottom:20px;"></div>
            <script type="text/javascript">
             var joy = new JoyStick('joyDiv');
             var socket = io();
             socket.on('connect', function() {
                 setInterval(function() {
                     var x = joy.GetX() / 100.0;
                     var th = joy.GetY() / 100.0;
                     socket.emit('cmd_vel', {x: x, theta: th});
                 }, 50);
             });
            </script>
        </body>
    </html>
    """

if __name__ == '__main__':
    rospy.init_node('simple_socketio_example', anonymous=True, disable_signals=True)
    socketio.run(app,host='18.20.192.50')
