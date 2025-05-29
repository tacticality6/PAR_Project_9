import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from par_project_9_interfaces.msg import Delivery, Deliveries, Package, Packages, Marker
from par_project_9_interfaces.srv import MarkerConfirmation

class DeliveryTrackingNode(Node):
    def __init__(self):
        super().__init__("delivery_tracking_node")

        #parameter declarations

        #onboard package variables
        self.packageOnboardTopic = self.declare_parameter("packages_onboard_topic","packages_onboard").get_parameter_value().string_value
        self.packageOnboardFreq = self.declare_parameter("package_onboard_repeat_freq", 10.0).get_parameter_value().double_value

        #completed deliveries logger
        self.completedDeliveriesTopic = self.declare_parameter("completed_deliveries_topic", "completed_deliveries").get_parameter_value().string_value
        self.completedDeliveriesFreq = self.declare_parameter("completed_deliveries_repeat_freq", 10.0).get_parameter_value().double_value

        #service called when marker touched
        self.touchedMarkerServiceName = self.declare_parameter("touched_marker_service_name", "touched_marker").get_parameter_value().string_value

        #publishers, services and callbacks

        self.touchedMarkerService = self.create_service(
            self.touchedMarkerServiceName, 
            MarkerConfirmation, 
            self.touched_marker_callback, 
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.onboardRepeatTimer = self.create_timer(
            self.packageOnboardFreq, 
            self.pub_onboard_callback
        )

        self.completedRepeatTimer = self.create_timer(
            self.completedDeliveriesFreq,
            self.pub_completed_callback
        )

        self.onboard_pub = self.create_publisher(
            Packages,
            self.packageOnboardTopic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        
        self.completed_pub = self.create_publisher(
            Deliveries,
            self.completedDeliveriesTopic,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        #local variables

        self.packages_onboard = []
        self.completed_deliveries = []

    
    def touched_marker_callback(self, request, response):
        response.success = False

        is_pickup = request.marker.is_pickup
        marker_id = request.marker.id
        dest_id = request.marker.dest_id

        if is_pickup:
            if not self.is_package_onboard(marker_id):
                response.success = True
                package_msg = Package()
                package_msg.package_id = marker_id
                package_msg.destination_id = dest_id
            
                package_msg.pickup_time = self.get_clock().now().to_msg()

                self.packages_onboard.append(package_msg)
                
        else:
            for msg in self.packages_onboard[:]:
                if msg.destination_id == dest_id:
                    response.success = True

                    delivery_msg = Delivery()
                    delivery_msg.package_id = msg.package_id
                    delivery_msg.destination_id = dest_id

                    now = self.get_clock().now()

                    delivery_msg.time_completed = now.to_msg()

                    pickup_time = Time.from_msg(msg.pickup_time)

                    elapsed_time = now - pickup_time
                    
                    delivery_msg.trip_time = elapsed_time.to_msg()

                    self.completed_deliveries.append(delivery_msg)

                    self.packages_onboard.remove(msg)
            self.pub_completed_callback()

        
        if response.success:
            self.pub_onboard_callback() 
        
        return response

    
    def is_package_onboard(self, marker_id):
        for msg in self.packages_onboard:
            if msg.package_id == marker_id:
                return True
        return False
            


    def pub_onboard_callback(self):
        msg = Packages()
        msg.packages = self.packages_onboard
        self.onboard_pub.publish(msg)

    def pub_completed_callback(self):
        msg = Deliveries()
        msg.deliveries = self.completed_deliveries
        self.onboard_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    delivery_tracking_node = DeliveryTrackingNode()

    try:
        rclpy.spin(delivery_tracking_node)
    except KeyboardInterrupt:
        pass

    delivery_tracking_node.destroy_node()
    rclpy.shutdown()