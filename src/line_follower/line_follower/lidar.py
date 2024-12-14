import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class LidarProcessingNode(Node):
    def __init__(self):
        super().__init__('lidar_processing_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',  # Change to your LiDAR topic name if different
            self.lidar_callback,
            10)
        self.parking_spot_publisher = self.create_publisher(Bool, '/parking_spot_available', 10)
        self.modified_scan_publisher = self.create_publisher(LaserScan, '/modified_scan', 10)
        self.get_logger().info("LiDAR Processing Node initialized.")

    def lidar_callback(self, msg):
        # Process LiDAR data to find objects on the right side (e.g., parked cars)
        print(len(msg.ranges))
        right_side_distances = self.extract_right_side(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_max)
        print(right_side_distances)
        modified_scan_msg = LaserScan()
        modified_scan_msg.ranges = right_side_distances
        self.modified_scan_publisher.publish(modified_scan_msg)
        print(right_side_distances[1])
        print(len(right_side_distances))
        self.get_logger().info('AAA')

        # Detect clusters that correspond to parked cars
        parked_cars = self.detect_parked_cars(right_side_distances)

        # Check for a gap between two parked cars large enough for the car to park
        if len(parked_cars) >= 2:
            gap_found = self.find_parking_gap(parked_cars)
            parking_spot_msg = Bool()
            parking_spot_msg.data = gap_found
            self.parking_spot_publisher.publish(parking_spot_msg)

    def extract_right_side(self, ranges, angle_min, angle_increment, range_max):
        """
        Extract distances from the LiDAR data that correspond to the right side of the car
        within the specified angle range (0.75π to -0.75π) and within 1 meter range.
        """
        # Calculate angle indices for 0.75π to -0.75π (90 degrees right side)
        angle_start =math.pi * 0.2
        angle_end =math.pi * 0.5
        print(angle_start)
        print(math.cos(angle_start))
        print(angle_end)
        print(math.cos(angle_end))

        # Convert angles to index based on angle_min and angle_increment
        start_index = int((angle_start - angle_min) / angle_increment)
        end_index = int((angle_end - angle_min) / angle_increment)
        print(start_index)
        print(end_index)
        j=min(start_index, end_index)-1

        # Extract and filter ranges for the right side within 1 meter distance
        right_side_distances = [
            round(ranges[i]*math.cos(angle_start+((i-j)*angle_increment)),2) if ranges[i] <= 5.0 else round(ranges[i],2) #range_max
            for i in range(min(start_index, end_index), max(start_index, end_index))
        ]
        return right_side_distances

    def detect_parked_cars(self, distances):
        """
        Detect clusters in the LiDAR data that represent parked cars based on distance continuity.
        """
        clusters = []
        current_cluster = []
        threshold = 0.05  # Distance threshold for clustering in meters

        for i, distance in enumerate(distances):
            # Only consider distances within a reasonable range to filter out noise
            if distance < 10.0:  # Adjust the distance range based on your environment
                if len(current_cluster) == 0 or abs(distance - distances[i-1]) < threshold:
                    current_cluster.append(distance)
                else:
                    clusters.append(current_cluster)
                    current_cluster = [distance]

        if current_cluster:
            clusters.append(current_cluster)

        # Filter clusters that are large enough to represent parked cars
        parked_cars = [cluster for cluster in clusters if len(cluster) > 10]  # Adjust size threshold as needed
        return parked_cars

    def find_parking_gap(self, parked_cars):
        """
        Calculate the gap between the last two detected parked cars to check if it's large enough.
        """
        # Take the last two parked cars
        last_car = parked_cars[-1]
        second_last_car = parked_cars[-2]

        # Calculate the gap size in meters
        gap_size = abs(last_car[0] - second_last_car[-1])
        car_length = 0.67  # Car length in meters

        self.get_logger().info(f"Detected gap size: {gap_size:.2f} meters")

        # Return True if the gap is large enough for the car
        return gap_size > car_length

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
