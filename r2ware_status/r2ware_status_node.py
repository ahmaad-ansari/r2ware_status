import rclpy
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import (
    ControlModeCommand,
    GearCommand,
    HazardLightsCommand,
    TurnIndicatorsCommand,
    ControlModeReport,
    GearReport,
    HazardLightsReport,
    SteeringReport,
    TurnIndicatorsReport,
    VelocityReport
)

class StatusSubscriberPublisher(Node):
    def __init__(self):
        super().__init__('r2ware_status_node')

        # Subscribers
        self.control_cmd_sub = self.create_subscription(AckermannControlCommand, '/control/command/control_cmd', self.control_cmd_callback, 10)
        self.control_mode_request_sub = self.create_subscription(ControlModeCommand, '/control/control_mode_request', self.control_mode_request_callback, 10)
        self.gear_cmd_sub = self.create_subscription(GearCommand, '/control/command/gear_cmd', self.gear_cmd_callback, 10)
        self.hazard_lights_cmd_sub = self.create_subscription(HazardLightsCommand, '/control/command/hazard_lights_cmd', self.hazard_lights_cmd_callback, 10)
        self.turn_indicators_cmd_sub = self.create_subscription(TurnIndicatorsCommand, '/control/command/turn_indicators_cmd', self.turn_indicators_cmd_callback, 10 )

        # Publishers
        self.control_mode_publisher = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode', 10)
        self.gear_status_publisher = self.create_publisher(GearReport, '/vehicle/status/gear_status', 10)
        self.hazard_lights_status_publisher = self.create_publisher(HazardLightsReport, '/vehicle/status/hazard_lights_status', 10)
        self.steering_status_publisher = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10)
        self.turn_indicators_status_publisher = self.create_publisher(TurnIndicatorsReport, '/vehicle/status/turn_indicators_status', 10)
        self.velocity_status_publisher = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 10)

    def control_cmd_callback(self, msg):
        # Extract relevant data from msg
        steering_angle = msg.lateral.steering_tire_angle
        longitudinal_velocity = msg.longitudinal.speed
        lateral_velocity = 0.0  # Since lateral velocity is not provided in the message
        heading_rate = 0.0  # Heading rate is not provided in the message

        # Create SteeringReport message
        steering_msg = SteeringReport()
        steering_msg.steering_tire_angle = steering_angle

        # Create VelocityReport message
        velocity_msg = VelocityReport()
        velocity_msg.header.frame_id = "base_link"
        velocity_msg.longitudinal_velocity = longitudinal_velocity
        velocity_msg.lateral_velocity = lateral_velocity
        velocity_msg.heading_rate = heading_rate

        # Publish messages
        self.steering_status_publisher.publish(steering_msg)
        self.velocity_status_publisher.publish(velocity_msg)

    def control_mode_request_callback(self, msg):
        # Extract relevant data from msg
        mode = msg.mode  # The mode received in the message
        print(msg)

        # Create ControlModeReport message
        control_mode_msg = ControlModeReport()
        control_mode_msg.mode = mode

        # Publish message
        self.control_mode_publisher.publish(control_mode_msg)

    def gear_cmd_callback(self, msg):
        # Extract relevant data from msg
        command = msg.command  # Gear command received in the message

        # Create GearReport message
        gear_report_msg = GearReport()
        gear_report_msg.report = command

        # Publish message
        self.gear_status_publisher.publish(gear_report_msg)

    def hazard_lights_cmd_callback(self, msg):
        # Extract relevant data from msg
        command = msg.command  # Hazard lights command received in the message

        # Create HazardLightsReport message
        hazard_lights_report_msg = HazardLightsReport()
        hazard_lights_report_msg.report = command

        # Publish message
        self.hazard_lights_status_publisher.publish(hazard_lights_report_msg)

    def turn_indicators_cmd_callback(self, msg):
        # Extract relevant data from msg
        command = msg.command  # Turn indicators command received in the message

        # Create TurnIndicatorsReport message
        turn_indicators_report_msg = TurnIndicatorsReport()
        turn_indicators_report_msg.report = command

        # Publish message
        self.turn_indicators_status_publisher.publish(turn_indicators_report_msg)

def main(args=None):
    rclpy.init(args=args)

    status_subscriber_publisher = StatusSubscriberPublisher()

    rclpy.spin(status_subscriber_publisher)

    status_subscriber_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
