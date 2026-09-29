# name and student number: [Sean Ge] [66574351]
 
# do not forget to document well (docstrings, code comments, ...) 
 
# include needed imports 
from copy import deepcopy
import rclpy 
from rclpy.node import Node 
 
import numpy as np  # optional 
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry 
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive 

class WallFollow(Node):
    """
    Implement Wall Following on the car
    """

    def __init__(self):
        super().__init__('wall_follow_node')

        self.current_speed = 0.0
        self.L_min = 1.0
        self.L_max = 3.0
        self.preview_time = 0.3 # future length-------------------------> adjust this value as needed

        # TODO: set PID gains
        self.kp = 1
        self.kd = 0 #----------------------------------------> adjust this value as needed
        self.ki = 0

        # TODO: store history
        self.integral = 0

        self.prev_error = None
        self.error = 0

        self.prev_time = 0
        self.dt = 0

        self.D_desired = 0.51  # desired distance to the wall-------------------------> adjust this value as needed

        self.beam_angle = 25  # angle to the left front-------------------------> adjust this value as needed

        # TODO: create subscribers and publishers
        
        # Send braking commands to the simulator. 
        self.publisher_ = self.create_publisher( 
            AckermannDriveStamped, 
            '/drive_from_wFollow', 
            10 
        ) 

        # Receive the lidar distance readings. 
        self.scan_subscription = self.create_subscription( 
            LaserScan, 
            '/scan', 
            self.scan_callback,  # choose a descriptive method name for the callback 
            10 
        )

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10
        ) 

    def odom_callback(self, msg):
        """Store the measured forward speed in m/s."""

        self.current_speed = float(msg.twist.twist.linear.x)

    def scan_callback(self, msg):
        """
        Callback function for the LIDAR scan data.
        This function is called every time a new LIDAR scan is received.

        Args:
            msg: LaserScan message containing the LIDAR data
        """

        #-------------------time-------------------------
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.prev_time == 0:
            self.prev_time = current_time
            return

        self.dt = current_time - self.prev_time
        self.prev_time = current_time

        if self.dt <= 0.0: return

        #-------------------get 2 ray range-------------------------
        a = self.get_range(msg, self.beam_angle)  # get the range at beam_angle degrees -----> Left Front
        b = self.get_range(msg, 90)  # get the range at 90 degrees -----> Straight Left

        #prevent nan for a,b in the future calculation
        if not (np.isfinite(a) and np.isfinite(b)):
            self.get_logger().warn(f"Invalid scan: a={a}, b={b}")
            stop_msg = AckermannDriveStamped()
            stop_msg.drive.speed = 0.0
            self.publisher_.publish(stop_msg)
            self.prev_error = None
            return


        #-------------------calculate error & PID-------------------------
        theta = np.radians(90.0 - self.beam_angle) #angle between a,b

    
        angle_alpha = np.arctan2(a * np.cos(theta) - b, a * np.sin(theta)) #angle difference

        self.L = float(np.clip(
        self.L_min + self.preview_time * abs(self.current_speed),
        self.L_min,
        self.L_max))

        D = b * np.cos(angle_alpha)  # distance to the wall
        D_future = D + self.L * np.sin(angle_alpha)  # distance to the wall in the future

        self.error = self.D_desired - D_future  # calculate the error to the wall

        derivative = 0
        if self.prev_error != None:
            derivative = (self.error - self.prev_error) / self.dt  # calculate the derivative of the error

        u = (self.kp * self.error + self.kd * derivative + self.ki * self.integral)  # calculate the control input using PID controller formula

        self.prev_error = self.error  # update the previous error for the next iteration


        #transer u into actual steering angle and speed
        steering_angle = -u
        steering_angle = float(np.clip(steering_angle, -0.4, 0.4))#-------------------------------------> adjust this value as needed

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.drive.steering_angle = steering_angle
        abs_steering = abs(steering_angle)

        if abs_steering < 0.08:
            speed = 6.0
        elif abs_steering < 0.18:
            speed = 3.0
        else:
            speed = 1.5

        msg.drive.speed = speed  #--------------------------------------> adjust this value as needed 

        #trouble shooting
        # self.get_logger().info(
        # f"D={D:.2f}, alpha={np.degrees(angle_alpha):.1f}, "
        # f"error={self.error:.2f}, steer={steering_angle:.3f}",
        # throttle_duration_sec=0.2)                    

        self.publisher_.publish(msg)


    #-------------------------------------------------------------------helper functions-------------------------------------------------------------    
    def get_range(self, msg, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle.
        Make sure you take care of NaNs etc.

        Args:
            msg: LaserScan message containing the LIDAR data
            angle: between angle_min and angle_max of the LIDAR

        Returns:
            range: range measurement in meters at the given angle
        """
        ranges = np.array(msg.ranges)
        if len(ranges) == 0: return np.nan

        target_angle = np.radians(angle)
        angle_array = (msg.angle_min + np.arange(len(ranges)) * msg.angle_increment)

        min_difference = np.inf
        closest_index = 0

        for index in range(len(angle_array)):
            difference = abs(angle_array[index] - target_angle)

            if difference < min_difference:
                min_difference = difference
                closest_index = index

            # Get the distance measured by the selected ray.
        range_measurement = ranges[closest_index]

        # Reject non-finite and out-of-range measurements.
        if (np.isfinite(range_measurement)):
            return range_measurement
        
        else:
            return np.nan
                

def main(args=None): 
    rclpy.init(args=args) 
    wall_follow_node = WallFollow() 
    rclpy.spin(wall_follow_node) 
    wall_follow_node.destroy_node() 
    rclpy.shutdown() 
 
 
if __name__ == '__main__': 
    main()    