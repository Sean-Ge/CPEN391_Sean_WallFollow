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
 
 
class SafetyNode(Node): 
    """A basic TTC-based AEB node that checks the car's forward corridor.""" 
 
    def __init__(self): 
        super().__init__('safety_node') 
 
        # Start at zero and update the speed when odometry arrives. 
        self.speed = 0.0 
 
        # Brake if the minimum TTC is below 0.7 seconds. 
        self.ttc_threshold = 0.3
 
        # Use the car width and leave extra space on each side. 
        self.car_width = 0.341 
        self.margin = 0.05 
 
        # create the required publisher, subscriber ... 
        # use the following as a starting point and modify as needed ... 

        self.requested_drive = None
        self.braking = False
 
        # Send braking commands to the simulator. 
        self.publisher_ = self.create_publisher( 
            AckermannDriveStamped, 
            'drive', 
            10 
        ) 
 
        # Receive the lidar distance readings. 
        self.scan_subscription = self.create_subscription( 
            LaserScan, 
            'scan', 
            self.scan_callback,  # choose a descriptive method name for the callback 
            10 
        ) 
 
        # Receive the car's current velocity. 
        self.odom_subscription = self.create_subscription( 
            Odometry, 
            'ego_racecar/odom', 
            self.odom_callback,  # choose a descriptive method name for the callback 
            10 
        )

        # Receive driving requests from wall-follow.
        self.wall_follow_subscription = self.create_subscription(
            AckermannDriveStamped,
            '/drive_from_wFollow',
            self.wall_follow_callback,
            10
        )
 
        # ... 
 
    # add methods as needed below here 
 
    def odom_callback(self, data): 
        """Update the car's forward speed from odometry.""" 
 
        self.speed = data.twist.twist.linear.x 

    
    def wall_follow_callback(self, msg):
        self.requested_drive = msg

 
    def scan_callback(self, data): 
        """Check TTC for obstacle points inside the forward corridor.""" 
 
        # Convert the lidar readings into an array. 
        ranges = np.array(data.ranges) 
 
        # Skip an empty scan. 
        if len(ranges) == 0: 
            return 
 
 
        # Find the actual angle of each lidar ray. 
        angles = (data.angle_min + np.arange(len(ranges)) * data.angle_increment) 
 
        # Replace unusable readings with NaN before doing calculations. 
        ranges = np.where(
                np.isfinite(ranges)
            & (ranges >= data.range_min)
            & (ranges <= data.range_max),
                ranges,
                np.nan
            )
 
        # Find each detected point's position relative to the lidar. 
        # Assume the lidar faces forward and is centered across the car. 
        forward_distance = ranges * np.cos(angles) 
        side_distance = ranges * np.sin(angles) 
 
        # Include a small clearance on both sides of the car. 
        half_width = self.car_width / 2.0 + self.margin 
 
        # Project the car's forward speed onto each lidar ray. 
        approaching_speed = self.speed * np.cos(angles) 
 
        # Keep the basic TTC estimate: distance / approaching speed. 
        ttc = ranges / np.maximum(approaching_speed, 0.0001) 
 
        # Only check points ahead and within the car's width plus clearance. 
        # Ignore points outside this corridor by setting their TTC to infinity. 
        ttc = np.where((forward_distance > 0.0) & (np.abs(side_distance) <= half_width),  ttc,  np.inf) 
 
        # Ignore directions where we are not approaching. 
        ttc = np.where(approaching_speed > 0.0001,  ttc,  np.inf) 
 
        # Ignore NaN and infinite results. 
        ttc = np.where(np.isfinite(ttc), ttc, np.inf) 
 
        # Find the smallest TTC. 
        minimum_ttc = np.min(ttc) 
 
        # Brake when the smallest TTC is below the threshold. 
        # Latch the brake once danger is detected.
        if minimum_ttc < self.ttc_threshold:
            self.braking = True

        # wall follow call might come after scan
        if self.requested_drive is None:
            return


        output_msg = AckermannDriveStamped()
        output_msg.header.stamp = self.get_clock().now().to_msg()
        output_msg.drive = deepcopy(self.requested_drive.drive)#deepcopy to prevent modifying the message

        if self.braking:
            output_msg.drive.speed = 0.0

        self.publisher_.publish(output_msg) 
                    
 
 
def main(args=None): 
    rclpy.init(args=args) 
    safety_node = SafetyNode() 
    rclpy.spin(safety_node) 
    safety_node.destroy_node() 
    rclpy.shutdown() 
 
 
if __name__ == '__main__': 
    main()