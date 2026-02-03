import time
import roslibpy

def string_callback(msg):
    print(f'Received message: {msg["data"]}')

def imu_callback(msg):
    print(f"gryo data: {msg['angular_velocity']}")

def odom_callback(msg):
    print(f"Position data: {msg['pose']['pose']['position']}")
    print(f"Orientation data: {msg['pose']['pose']['orientation']}")

id = 86
client = roslibpy.Ros(host=f'10.24.6.{id}', port=9090)
client.connect()

subscriber = roslibpy.Topic(client, '/chatter', 'std_msgs/String')
subscriber.subscribe(string_callback)

imu_subscriber = roslibpy.Topic(client, f'/create_{id}/imu', 'sensor_msgs/Imu')
# imu_subscriber.subscribe(imu_callback)

odom_subscriber = roslibpy.Topic(client, f'/create_{id}/odom', 'nav_msgs/Odometry')
# odom_subscriber.subscribe(odom_callback)

cmd_vel_subscriber = roslibpy.Topic(client, f'/create_{id}/cmd_vel', 'geometry_msgs/Twist')
cmd_vel_subscriber.subscribe(lambda msg: print(f"Received cmd_vel: {msg}"))

client.run_forever()