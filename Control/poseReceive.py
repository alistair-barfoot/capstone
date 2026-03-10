#Script to listen to the ROS2 pose data and write it to a file.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import os

class PoseFileWriter(Node):
    def __init__(self):
        super().__init__('pose_file_writer')
        
        # --- Config ---
        self.topic_name = '/pos_rot_left'
        self.file_path = "latest_pose.txt"
        
        # --- Subscriber ---
        self.subscription = self.create_subscription(
            PoseStamped,
            self.topic_name,
            self.listener_callback,
            10)
        
        # --- File Setup ---
        #Open the file in 'w' (write) mode. 
        self.f = open(self.file_path, 'w')
        self.get_logger().info(f"Writing pose data to {self.file_path}...")

    def listener_callback(self, msg):
        try:
            # Format: x, y, z, qx, qy, qz, qw
            data = (f"{msg.pose.position.x:.5f},"
                    f"{msg.pose.position.y:.5f},"
                    f"{msg.pose.position.z:.5f},"
                    f"{msg.pose.orientation.x:.5f},"
                    f"{msg.pose.orientation.y:.5f},"
                    f"{msg.pose.orientation.z:.5f},"
                    f"{msg.pose.orientation.w:.5f}\n")
            time = self.get_clock().now().to_msg()
            #print(f"{data} at {time}") 
            # 1. Go to the start of the file
            self.f.seek(0)
            # 2. Overwrite existing data
            self.f.write(data)
            # 3. Truncate ensures no leftover characters if the new line is shorter
            self.f.truncate()
            # 4. Flush forces the data from Python's buffer to the OS immediately
            self.f.flush()
            
        except Exception as e:
            self.get_logger().error(f"Write error: {e}")

    def destroy_node(self):
        self.f.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PoseFileWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()