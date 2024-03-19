import rclpy
from rclpy.node import Node
from autoware_auto_vehicle_msgs.msg import (
    VelocityReport,
    SteeringReport,
    ControlModeReport,
    GearReport
)
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, qos_profile_sensor_data

class StatusPublisher(Node):
    def __init__(self):
        super().__init__('r2ware_sensing_node')

        # Subscriber for clock topic
        self.clock_sub = self.create_subscription(Time, '/clock', self.clock_callback, 10)
        self.latest_clock_msg = None

        # Define QoS profile
        qos_profile = QoSProfile(depth=1, reliability=QoSProfile.Reliability.RELIABLE,
                                 durability=QoSProfile.Durability.VOLATILE)

        # Publishers
        self.velocity_publisher = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', qos_profile)
        self.steering_publisher = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', qos_profile)
        self.control_mode_publisher = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode', qos_profile)
        self.gear_publisher = self.create_publisher(GearReport, '/vehicle/status/gear_status', qos_profile)

    def clock_callback(self, msg):
        self.latest_clock_msg = msg
        self.publish_messages()

    def publish_messages(self):
        if self.latest_clock_msg is not None:
            current_time = self.latest_clock_msg
            self.publish_velocity_report(current_time)
            self.publish_steering_report(current_time)
            self.publish_control_mode_report(current_time)
            self.publish_gear_report(current_time)

    def publish_velocity_report(self, current_time):
        velocity_msg = VelocityReport()
        velocity_msg.header.stamp = current_time
        velocity_msg.header.frame_id = "base_link"
        velocity_msg.longitudinal_velocity = 0.0
        velocity_msg.lateral_velocity = 0.0
        velocity_msg.heading_rate = 0.0
        # Add logic to populate velocity_msg with data
        self.velocity_publisher.publish(velocity_msg)

    def publish_steering_report(self, current_time):
        steering_msg = SteeringReport()
        steering_msg.stamp = current_time
        steering_msg.steering_tire_angle = 0
        # Add logic to populate steering_msg with data
        self.steering_publisher.publish(steering_msg)

    def publish_control_mode_report(self, current_time):
        control_mode_msg = ControlModeReport()
        control_mode_msg.stamp = current_time
        control_mode_msg.mode = 1
        # Add logic to populate control_mode_msg with data
        self.control_mode_publisher.publish(control_mode_msg)

    def publish_gear_report(self, current_time):
        gear_msg = GearReport()
        gear_msg.header.stamp = current_time
        gear_msg.report = 22

        # Add logic to populate gear_msg with data
        self.gear_publisher.publish(gear_msg)

def main(args=None):
    rclpy.init(args=args)

    status_publisher = StatusPublisher()

    rclpy.spin(status_publisher)

    status_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





'''
Category 	    Topic 	                            Message type 	                                frame_id 	Hz 	QoS
Velocity 	    /vehicle/status/velocity_status 	autoware_auto_vehicle_msgs/VelocityReport 	    base_line 	30 	Reliable,Volatile,Keep last/1
Steering 	    /vehicle/status/steering_status 	autoware_auto_vehicle_msgs/SteeringReport 	    - 	        30 	Reliable,Volatile,Keep last/1
X Control Mode 	/vehicle/status/control_mode 	    autoware_auto_vehicle_msgs/ControlModeReport 	- 	        30 	Reliable,Volatile,Keep last/1
X Gear 	        /vehicle/status/gear_status 	    autoware_auto_vehicle_msgs/GearReport 	        - 	        30 	Reliable,Volatile,Keep last/1
'''