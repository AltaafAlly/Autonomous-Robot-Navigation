import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetModelState
import math
from path_mapping import get_path

class TurtlebotMover:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.current_pose = (0, 0, 0)  # (x, y, theta)
        self.distance_tolerance = 0.1 # a small distance to consider as the goal reached

        self.MAX_LINEAR_ACCELERATION = 0.02  # Change this as needed
        self.MAX_LINEAR_VELOCITY = 1.0  # Change this as needed
        self.prev_linear_velocity = 0.0

        self.get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)

        # Initialize PID variables for angular velocity
        self.last_error_theta = 0  # Error at the previous time step
        self.integral_theta = 0  # Integral of error so far

    def move_to_goal(self, goal, speed=10, Kp=1, Ki=0.5, Kd=0.8):
        # Initialize Twist message
        move_cmd = Twist()
        
        # Calculate difference between current pose and goal
        delta_x = goal[0] - self.current_pose[0]
        delta_y = goal[1] - self.current_pose[1]
        goal_theta = math.atan2(delta_y, delta_x)
        
        # Proportional controller for linear velocity
        distance_to_goal = math.sqrt(delta_x**2 + delta_y**2)
        
        # If we are close to the goal, stop
        if distance_to_goal < self.distance_tolerance:
            return move_cmd

        # Proportional controller for angular velocity
        delta_theta = goal_theta - self.current_pose[2]
        delta_theta = (delta_theta + math.pi) % (2 * math.pi) - math.pi # Normalize to be between -pi and pi
        
        # PID controller for angular velocity
        self.integral_theta += delta_theta
        derivative_theta = delta_theta - self.last_error_theta
        move_cmd.angular.z = Kp * delta_theta + Ki * self.integral_theta + Kd * derivative_theta
        self.last_error_theta = delta_theta  # Update last error for next time step

        # If we are not oriented towards the goal within a tolerance, rotate only
        if abs(delta_theta) > self.distance_tolerance:
            pass  # Angular velocity is already set by PID controller
        else:  # Else, we can move forward
            target_linear_velocity = min(speed * distance_to_goal, speed)
            # Apply velocity ramping
            delta_v = target_linear_velocity - self.prev_linear_velocity
            if abs(delta_v) > self.MAX_LINEAR_ACCELERATION:
                target_linear_velocity = self.prev_linear_velocity + math.copysign(self.MAX_LINEAR_ACCELERATION, delta_v)
            self.prev_linear_velocity = target_linear_velocity
            move_cmd.linear.x = target_linear_velocity

        return move_cmd

    def odom_callback(self, msg):
        # Update current pose using Gazebo's get_model_state service
        resp = self.get_model_state('mobile_base', '')  # Replace with your robot's model name
        position = resp.pose.position
        orientation = resp.pose.orientation
        qx, qy, qz, qw = orientation.x, orientation.y, orientation.z, orientation.w
        euler = euler_from_quaternion([qx, qy, qz, qw])
        self.current_pose = (position.x, position.y, euler[2])

    def get_gazebo_model_state(self):
        try:
            resp = self.get_model_state('mobile_base', '')  # Replace with your robot's model name
            return resp.pose
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


def main():
    rospy.init_node('turtlebot_mover')

    mover = TurtlebotMover()
    x = input("x: ")
    y = input("y: ")
    actual_model_state = mover.get_gazebo_model_state()
    print("Actual model state: {}".format(actual_model_state))
    
    goal = [int(x), int(y)] # print the position of the postbox
   
    origin = mover.current_pose
    
    shortest_path = get_path(origin, goal)
    
    rate = rospy.Rate(10)  # 10 Hz

    try:
        # Now traverse the shortest path
        for step in shortest_path:
            goal_x, goal_y = step  # Unpack coordinates
            goal = [goal_x, goal_y]

            while not rospy.is_shutdown():
                move_cmd = mover.move_to_goal(goal)
                mover.pub.publish(move_cmd)

                # If the current position is within the tolerance of the goal, break the loop and move on to the next goal.
                if math.sqrt((goal_x - mover.current_pose[0])**2 + (goal_y - mover.current_pose[1])**2) < mover.distance_tolerance:
                    break
                rate.sleep()
                
    except rospy.ROSInterruptException:
        pass

if __name__ == "__main__":
    main()