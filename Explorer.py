# #!/usr/bin/env python
# import rospy
# from geometry_msgs.msg import Twist
# import sys
# import select
# import termios
# import tty

# def publish_velocity(pub, linear_velocity, angular_velocity, duration):
#     twist = Twist()
#     twist.linear.x = linear_velocity
#     twist.angular.z = angular_velocity

#     # Publish the velocity command
#     pub.publish(twist)

#     # Sleep for the specified duration
#     rospy.sleep(duration)

#     # Stop the Turtlebot
#     twist.linear.x = 0.0
#     twist.angular.z = 0.0
#     pub.publish(twist)

# def get_key():
#     tty.setraw(sys.stdin.fileno())
#     select.select([sys.stdin], [], [], 0)
#     key = sys.stdin.read(1)
#     termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
#     return key

# if __name__ == '__main__':
#     rospy.init_node('turtlebot_controller', anonymous=True)
#     pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

#     # Save terminal settings
#     settings = termios.tcgetattr(sys.stdin)

#     linear_velocity = 0.2  # Adjust the linear velocity as desired
#     angular_velocity = 0.5  # Adjust the angular velocity as desired
#     duration = 0.5  # Adjust the duration of each movement as desired

#     try:
#         while not rospy.is_shutdown():
#             key = get_key()

#             if key == 'w':
#                 publish_velocity(pub, linear_velocity, 0.0, duration)
#             elif key == 's':
#                 publish_velocity(pub, -linear_velocity, 0.0, duration)
#             elif key == 'a':
#                 publish_velocity(pub, 0.0, angular_velocity, duration)
#             elif key == 'd':
#                 publish_velocity(pub, 0.0, -angular_velocity, duration)
#             elif key == 'q':
#                 break

#     except rospy.ROSInterruptException:
#         pass

#     finally:
#         # Restore terminal settings
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

def publish_velocity(pub, linear_velocity, angular_velocity):
    twist = Twist()
    twist.linear.x = linear_velocity
    twist.angular.z = angular_velocity
    pub.publish(twist)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    rospy.init_node('turtlebot_controller', anonymous=True)
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)

    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)

    linear_velocity = 0.2  # Adjust the linear velocity as desired
    angular_velocity = 0.5  # Adjust the angular velocity as desired

    try:
        while not rospy.is_shutdown():
            key = get_key(settings)

            if key == 'w':
                publish_velocity(pub, linear_velocity, 0.0)
            elif key == 's':
                publish_velocity(pub, -linear_velocity, 0.0)
            elif key == 'a':
                publish_velocity(pub, 0.0, angular_velocity)
            elif key == 'd':
                publish_velocity(pub, 0.0, -angular_velocity)
            elif key == 'q':
                break
            else:
                publish_velocity(pub, 0.0, 0.0)

            rospy.sleep(0.1)  # Adjust the sleep duration for smoother control

    except rospy.ROSInterruptException:
        pass

    finally:
        # Stop the robot and restore terminal settings
        publish_velocity(pub, 0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
