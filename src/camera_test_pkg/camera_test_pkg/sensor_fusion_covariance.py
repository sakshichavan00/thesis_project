# importing the required modules
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import Float32
import tf_transformations
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import cv2
import numpy as np

# imports for YOLO model
from ultralytics import YOLO
from .sort import Sort  # Ensure the SORT implementation is accessible in your PYTHONPATH

# Load YOLO model
model = YOLO("/home/sakshi/Desktop/IRP/ws_dartec/test_custom_panther_ws/src/test_custom_panther/models/best.pt")

class IntegratedCameraNode(Node):
    def __init__(self):
        super().__init__('integrated_camera_node')

        self.publisher_pose = self.create_publisher(PoseWithCovarianceStamped, 'robot_pose', 10)
        self.publisher_speed = self.create_publisher(Float32, 'robot_speed', 10)  # New publisher for speed
        self.tracker = Sort()
        self.specific_classes = ["person", "cat", "dog", "chair", "bottle", "car", "bowl", "cup", "panther_robot"]
        self.class_object_ids = {class_label: 1 for class_label in self.specific_classes}
        self.prev_positions = {}
        self.active_camera = 1  # Start with camera 1 as the active camera

        self.got_camera_info1 = False
        self.got_camera_info2 = False

        self.camera_model_1 = PinholeCameraModel()
        self.camera_model_2 = PinholeCameraModel()
        self.bridge = CvBridge()

        self.original_width = None
        self.original_height = None
        self.new_width = 800
        self.new_height = None

        self.no_detection_duration = 0.1 # Time duration to wait before switching cameras (in seconds)
        self.last_detection_time = self.get_clock().now()
        self.bot_detected = False
        self.detection_data = None
        self.bot_in_cam_1 = False
        self.bot_in_cam_2 = False


        # Subscriptions for Camera 1
        self.camera_info_sub_1 = self.create_subscription(
            CameraInfo,
            '/world/default/model/top_down_camera/link/link/sensor/camera1/camera_info',
            self.camera_info_callback1,
            10
        )
        self.subscription_1 = self.create_subscription(
            Image,
            '/world/default/model/top_down_camera/link/link/sensor/camera1/image',
            self.listener_callback1,
            10
        )

        # Subscriptions for Camera 2
        self.camera_info_sub_2 = self.create_subscription(
            CameraInfo,
            '/world/default/model/side_camera/link/link/sensor/camera2/camera_info',
            self.camera_info_callback2,
            10
        )
        self.subscription_2 = self.create_subscription(
            Image,
            '/world/default/model/side_camera/link/link/sensor/camera2/image',
            self.listener_callback2,
            10
        )

        # camera extrinsic parameters for camera 1
        # self.camera_position_1 = np.array([10, 0, 20])
        # camera_orientation_1 = [0, 1.5708, 0]  # in Roll pitch yaw (xyz)
        # self.camera_position_1 = np.array([10, 0, 20])
        self.camera_position_1 = np.array([6, 25, 8])

        # camera_orientation_1 = [0, 2.0944, 0]  # in Roll pitch yaw (xyz)
        camera_orientation_1 = [0, 2.0944, 0]  # in Roll pitch yaw (xyz)

        camera_rotation_matrix_1 = tf_transformations.euler_matrix(camera_orientation_1[0], camera_orientation_1[1], camera_orientation_1[2], 'sxyz')
        self.transformation_matrix_1 = np.identity(4)
        self.transformation_matrix_1[:3, :3] = camera_rotation_matrix_1[:3, :3]
        self.transformation_matrix_1[:3, 3] = self.camera_position_1
        self.transformation_matrix_inv_1 = np.linalg.inv(self.transformation_matrix_1)

        # camera extrinsic parameters for camera 2
        # self.camera_position_2 = np.array([0, 0, 20])
        self.camera_position_2 = np.array([10, 11, 9])

        camera_orientation_2 = [0, 2.0944, 0]  # in Roll pitch yaw (xyz)
        # camera_orientation_2 = [0, 1.5708, 0]  # in Roll pitch yaw (xyz)

        camera_rotation_matrix_2 = tf_transformations.euler_matrix(camera_orientation_2[0], camera_orientation_2[1], camera_orientation_2[2], 'sxyz')
        self.transformation_matrix_2 = np.identity(4)
        self.transformation_matrix_2[:3, :3] = camera_rotation_matrix_2[:3, :3]
        self.transformation_matrix_2[:3, 3] = self.camera_position_2
        self.transformation_matrix_inv_2 = np.linalg.inv(self.transformation_matrix_2)

        # define 2 variable to store camera image, make them black array of size 800x450
        self.disp_image_1 = np.zeros((450, 800, 3), dtype=np.uint8)
        self.disp_image_2 = np.zeros((450, 800, 3), dtype=np.uint8)

        # erros for various locations of the robot on the image, based on a 8x5 grid 
        self.x_variances = np.array([
            [0.5, 0.2, 0, 0.1, 0.2, 0, 0.2, 0.4],
            [1.1, 0.7, 0.4, 0.3, 0.3, 0.4, 0.65, 1.1],
            [2.4, 1.6, 1.2, 1, 1, 1.2, 1.6, 2.2],
            [3.6, 3.2, 2.7, 2.4, 2.4, 2.7, 3.1, 3.7],
            [6.3, 5.6, 5.2, 4.8, 4.8, 5.1, 5.6, 6]
        ])
        self.y_variances = np.array([
            [0.94, 0.22, 0, 0.04, 0, 0.1, 0.28, 0.84],
            [1.3, 0.6, 0.2, 0.03, 0.07, 0.3, 0.63, 1.3],
            [2.3, 1.2, 0.6, 0.2, 0.2, 0.6, 1.1, 2.23],
            [3.1, 1.8, 0.95, 0.36, 0.2, 1, 1.8, 3.01],
            [4.34, 2.8, 1.6, 0.5, 0.4, 1.7, 2.7, 3.9]
        ])

        # create a timer to call the pose publish methods every 0.1 seconds
        self.create_timer(0.1, self.publish_pose)



    def camera_info_callback1(self, msg):
        # self.get_logger().info(f"Got Camera 1 Info")
        self.got_camera_info1 = True
        self.camera_model_1.fromCameraInfo(msg)



    def camera_info_callback2(self, msg):
        # self.get_logger().info(f"Got Camera 2 Info")
        self.got_camera_info2 = True
        self.camera_model_2.fromCameraInfo(msg)

    

    # when a new image is availabe in camera - 1
    def listener_callback1(self, msg):
        # check if time has elapsed since last detection
        duration = self.get_clock().now() - self.last_detection_time
        if (duration.nanoseconds / 1e9 >= self.no_detection_duration) and (not self.bot_in_cam_2):
            self.detection_data = self.process_image(msg, 1)
            self.get_logger().info(f"Detection Data [cam 1]: {self.detection_data[0]}")
            self.get_logger().info(f"Detection Data [cam 1]: {self.detection_data[0] == None}")
            if self.detection_data[0]:
                self.last_detection_time = self.get_clock().now()
                self.bot_detected = True
                # set the bool to indicate that the bot is in camera 1, and no need to process camera 2 frames
                self.bot_in_cam_1 = True
            else:
                self.bot_detected = False
                self.bot_in_cam_1 = False


    # when a new image is availabe in camera - 2
    def listener_callback2(self, msg):
        # check if time has elapsed since last detection
        duration = self.get_clock().now() - self.last_detection_time
        if (duration.nanoseconds / 1e9 >= self.no_detection_duration) and (not self.bot_in_cam_1):
            # process the image, do detections
            self.detection_data = self.process_image(msg, 2)
            self.get_logger().info(f"Detection Data [cam 1]: {self.detection_data[0]}")
            self.get_logger().info(f"Detection Data [cam 2]: {self.detection_data[0] == None}")
            if self.detection_data[0]:
                self.last_detection_time = self.get_clock().now()
                self.bot_detected = True
                # set the bool to indicate that the bot is in camera 2, and no need to process camera 1 frames
                self.bot_in_cam_2 = True
            else:
                self.bot_detected = False
                self.bot_in_cam_2 = False
                

    def show_images(self):
        
        # draw the grid on the image
        rows = 5
        cols = 8
        image_width = 800
        image_height = 450
        cell_width = image_width // cols
        cell_height = image_height // rows
        # Draw the grid
        for i in range(1, cols):
            # Vertical lines
            cv2.line(self.disp_image_1, (i * cell_width, 0), (i * cell_width, image_height), (0, 0, 0), 1)
            cv2.line(self.disp_image_2, (i * cell_width, 0), (i * cell_width, image_height), (0, 0, 0), 1)
        for j in range(1, rows):
            # Horizontal lines
            cv2.line(self.disp_image_1, (0, j * cell_height), (image_width, j * cell_height), (0, 0, 0), 1)
            cv2.line(self.disp_image_2, (0, j * cell_height), (image_width, j * cell_height), (0, 0, 0), 1)
        
        # display the images
        cv2.imshow("Camera Image - 1", self.disp_image_1)
        cv2.imshow("Camera Image - 2", self.disp_image_2)
        cv2.waitKey(1)


    # calcuate all the camera to world coordinate transformation
    def do_transform(self, u, v, camera_id):
        camera_model = self.camera_model_1 if camera_id == 1 else self.camera_model_2
        transformation_matrix = self.transformation_matrix_1 if camera_id == 1 else self.transformation_matrix_2

        image_point = (u, v)
        camera_point = camera_model.projectPixelTo3dRay(image_point)

        # Transform the ray to the world coordinates if necessary
        camera_point = [coord * 21 for coord in camera_point]
        camera_point_temp = np.array([camera_point[2], -camera_point[0], -camera_point[1]])

        camera_point_temp_frame = np.array([camera_point_temp[0], camera_point_temp[1], camera_point_temp[2], 1])
        world_point = np.dot(transformation_matrix, camera_point_temp_frame)

        world_point[2] = 0

        # self.get_logger().info(f'camera coordinates: [{camera_point_temp[0]}, {camera_point_temp[1]}, {camera_point_temp[2]}]')
        # self.get_logger().info(f'world coordinates: [{world_point[0]}, {world_point[1]}, {world_point[2]}]')

        return world_point



    def publish_pose(self):
        if self.bot_detected:
            # Get the position of the robot in the world coordinates (using the do transform method)
            world_coords = self.detection_data[3]
            center_x = self.detection_data[1]
            center_y = self.detection_data[2]
            # call the get covaraince function to get the covariance matrix
            covariance_matrix = self.get_covariance(center_x, center_y)
            # store the time of the detection
            current_time = self.get_clock().now().to_msg()


            # publish the pose with the covariance stamped messsage
            robot_pose = PoseWithCovarianceStamped()
            robot_pose.header.stamp = current_time
            robot_pose.header.frame_id = "world"
            robot_pose.pose.pose.position.x = world_coords[0]
            robot_pose.pose.pose.position.y = world_coords[1]
            robot_pose.pose.pose.position.z = world_coords[2]
            robot_pose.pose.covariance = covariance_matrix.flatten().tolist()

            self.publisher_pose.publish(robot_pose)

            # show the images
            self.show_images()

            # wait for the next detection
            self.bot_detected = False



    # do the machine learning model prediction
    def process_image(self, msg, camera_id):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            # self.get_logger().error(f"Error converting image: {e}")
            return []

        self.original_height, self.original_width = cv_image.shape[:2]
        # self.get_logger().info(f"Original Image Dimensions: {self.original_width}x{self.original_height}")

        self.new_height = int((self.new_width / self.original_width) * self.original_height)
        resized_image = cv2.resize(cv_image, (self.new_width, self.new_height))
        # self.get_logger().info(f"Resized Image Dimensions: {self.new_width}x{self.new_height}")

        results = model.predict(resized_image)
        detections = []

        if results:
            result = results[0]
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                class_label = model.names[class_id]

                if class_label == "panther_robot":
                    cords = box.xyxy[0].tolist()
                    cords = [round(x) for x in cords]
                    conf = round(box.conf[0].item(), 2)
                    detections.append(cords + [conf])

                    x1, y1, x2, y2, conf = detections[0]
                    center_x, center_y = (x1 + x2) // 2, (y1 + y2) // 2
                    orig_center_x = int(center_x * self.original_width / self.new_width)
                    orig_center_y = int(center_y * self.original_height / self.new_height)
                    
                    world_coords = self.do_transform(orig_center_x, orig_center_y, camera_id)
                    self.get_logger().info(f"World Coordinates: {world_coords}")

                    x1, y1, x2, y2 = cords
                    cv2.rectangle(resized_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(resized_image, f"{class_label} Conf:{conf}", (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    if camera_id == 1:
                        self.disp_image_1 = np.copy(resized_image)
                    else:
                        self.disp_image_2 = np.copy(resized_image)

                    return detections, center_x, center_y, world_coords 
                    
        return None, None
        


    # calcualte the variance to use as per the location of the robot in the image
    # using the grid structure define,
    # the erros in the pose of the robot in each grid was calcuated manually
    def get_covariance(self, bb_center_x, bb_center_y, img_width=800, img_height=450, grid_x=8, grid_y=5):
       
        # Calculate grid dimensions
        grid_width = img_width / grid_x
        grid_height = img_height / grid_y

        # Determine the grid cell indices
        grid_i = int(bb_center_y // grid_height)
        grid_j = int(bb_center_x // grid_width)

        # Extract the variances for the respective grid cell
        var_x = self.x_variances[grid_i, grid_j]
        var_y = self.y_variances[grid_i, grid_j]

        # Create a 6x6 covariance matrix initialized to zero
        covariance_matrix = np.zeros((6, 6))

        # Set the variances in the corresponding positions (x, y)
        covariance_matrix[0, 0] = var_x
        covariance_matrix[1, 1] = var_y

        return covariance_matrix




def main(args=None):
    rclpy.init(args=args)
    node = IntegratedCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()