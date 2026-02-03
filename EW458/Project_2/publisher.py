import time
import roslibpy

id = 86
client = roslibpy.Ros(host=f'10.24.6.{id}', port=9090)
client.run()

talker = roslibpy.Topic(client, '/chatter', 'std_msgs/String')

cmd_vel_pub = roslibpy.Topic(client, f'/create_{id}/cmd_vel', 'geometry_msgs/Twist')

while client.is_connected:
    #talker.publish(roslibpy.Message({'data': 'Hello, this is Andrew!'}))
    #print('Published message to /chatter topic')

    twist = roslibpy.Message({
        'linear': {
            'x': 0.5,
            'y': 0.0,
            'z': 0.0
        },
        'angular': {
            'x': 0.0,
            'y': 0.0,
            'z': 1
        }
    })
    cmd_vel_pub.publish(twist)
    print('Published movement command to /cmd_vel topic')
    time.sleep(0.1)

talker.unadvertise()
cmd_vel_pub.unadvertise()
client.terminate()