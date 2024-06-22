import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo  # Import CameraInfo
from std_srvs.srv import SetBool  # Import the SetBool service
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectCollectionNode(Node):
    def __init__(self):
        super().__init__('object_collection_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.subscription_info = self.create_subscription(
            CameraInfo,  # Subscription to CameraInfo
            'camera/camera_info',  # Adjust this topic according to your actual topic name
            self.camera_info_callback,
            10)
        self.detected_objects = []
        # Create a client to call the gripper control service
        self.gripper_control_client = self.create_client(SetBool, 'control_gripper')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.detected_objects = self.detect_objects(cv_image)
            
            # Display the image
            cv2.imshow("Drone Camera", cv_image)
            cv2.waitKey(1)
            
            for obj in self.detected_objects:
                self.approach_object(obj)
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def detect_objects(self, image):
        detected_objects = []
        try:
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Define the lower and upper bounds for the red color in HSV space
            lower_red_1 = np.array([0, 50, 50])
            upper_red_1 = np.array([10, 255, 255])
            lower_red_2 = np.array([170, 50, 50])
            upper_red_2 = np.array([180, 255, 255])

            # Create masks for the two red ranges
            mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
            mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)

            # Combine the masks
            mask = cv2.bitwise_or(mask1, mask2)
           
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                x, y, w, h = cv2.boundingRect(cnt)
                detected_objects.append((x, y, w, h))
                cv2.rectangle(image, (x, y), (x+w, y+h), (0, 255, 0), 2)
                self.get_logger().info(f"Detected object at x: {x}, y: {y}, width: {w}, height: {h}")

        except Exception as e:
            self.get_logger().error(f"Error in detect_objects: {e}")
        return detected_objects

    def approach_object(self, obj):
        try:
            obj_x, obj_y, obj_w, obj_h = obj
            desired_x = obj_x + obj_w / 2.0
            desired_y = obj_y + obj_h / 2.0
            
            # Calculate distance
            distance = self.calculate_distance(desired_x, desired_y)
            
            # Handle object if within grabbing distance
            if distance < 10:  # Adjust this threshold as needed
                self.grab_object(obj)
            else:
                self.get_logger().info(f"Approaching object: Desired position - ({desired_x}, {desired_y}), Distance - {distance:.2f} units")
        except Exception as e:
            self.get_logger().error(f"Error in approach_object: {e}")

    def calculate_distance(self, obj_x, obj_y):
        try:
            # Assuming the drone's position is known
            drone_x = 0  # Update with actual drone position
            drone_y = 0  # Update with actual drone position
            distance = np.sqrt((obj_x - drone_x)**2 + (obj_y - drone_y)**2)
            return distance
        except Exception as e:
            self.get_logger().error(f"Error in calculate_distance: {e}")
            return float('inf')  # Return a large number to indicate error

    def grab_object(self, obj):
        try:
            # Call the gripper control service to grab the object
            request = SetBool.Request()
            request.data = True  # Set to True to close the gripper
            future = self.gripper_control_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info("Object grabbed successfully")
                else:
                    self.get_logger().error("Failed to grab object")
            else:
                self.get_logger().error("Service call failed: Did not receive a response")
        except Exception as e:
            self.get_logger().error(f"Error in grab_object: {e}")

    def camera_info_callback(self, msg):
        try:
            # Extract camera information from the message
            height = msg.height
            width = msg.width
            distortion_model = msg.distortion_model
            D = msg.D  # Distortion coefficients
            K = msg.K  # Intrinsic camera matrix
            R = msg.R  # Rectification matrix
            P = msg.P  # Projection matrix

            # You can process the camera information here as needed
            # For example, you can print it or use it for camera calibration
            self.get_logger().info(f"Received camera info - Height: {height}, Width: {width}, Distortion model: {distortion_model}")

        except Exception as e:
            self.get_logger().error(f"Error in camera_info_callback: {e}")
        

def main(args=None):
    rclpy.init(args=args)
    object_collection_node = ObjectCollectionNode()
    try:
        rclpy.spin(object_collection_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        object_collection_node.get_logger().error(f"Unexpected exception: {e}")
    finally:
        try:
            if rclpy.ok():
                object_collection_node.destroy_node()
                rclpy.shutdown()
            cv2.destroyAllWindows()
        except Exception as e:
            object_collection_node.get_logger().error(f"Error during shutdown: {e}")
            cv2.destroyAllWindows()  # Close OpenCV windows on shutdown

if __name__ == '__main__':
    main()
