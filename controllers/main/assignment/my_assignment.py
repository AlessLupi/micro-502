import numpy as np
import time
import cv2

# The available ground truth state measurements can be accessed by calling sensor_data[item]. All values of "item" are provided as defined in main.py within the function read_sensors. 
# The "item" values that you may later retrieve for the hardware project are:
# "x_global": Global X position
# "y_global": Global Y position
# "z_global": Global Z position
# 'v_x": Global X velocity
# "v_y": Global Y velocity
# "v_z": Global Z velocity
# "ax_global": Global X acceleration
# "ay_global": Global Y acceleration
# "az_global": Global Z acceleration (With gravtiational acceleration subtracted)
# "roll": Roll angle (rad)
# "pitch": Pitch angle (rad)
# "yaw": Yaw angle (rad)
# "q_x": X Quaternion value
# "q_y": Y Quaternion value
# "q_z": Z Quaternion value
# "q_w": W Quaternion value

# A link to further information on how to access the sensor data on the Crazyflie hardware for the hardware practical can be found here: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/api/logs/#stateestimate


class Gates:
    def __init__(self,):
        self.b2cT = np.array([0.03,0,0.01]) # translation from body to camera frame
        self.focal_length = 161.013922282 # focal length in pixels
        self.principal_point = np.array([150, 150]) # principal point in normalized coordinates
        self.pixel_size = 0.000005 # pixel size in meters (example value ajust as needed)
        self.intrinsic_matrix = np.array([[self.focal_length, 0, self.principal_point[0]],
                                    [0, self.focal_length, self.principal_point[1]],
                                    [0, 0, 1]]) # intrinsic matrix
        self.b2i = np.eye(3) # place holder for camera extrinsic matrix (3x3 matrix)
        self.c2b = np.array([[0, 0, 1],
                        [-1, 0, 0],
                        [0, -1, 0]]) # place holder for camera extrinsic matrix (3x3 matrix)

    def centroids(self, camera_data):
        hsv = cv2.cvtColor(camera_data, cv2.COLOR_BGR2HSV)

        # Define range of purple in HSV
        lower_purple = np.array([125, 50, 50])
        upper_purple = np.array([155, 255, 255])

        # Threshold the HSV image to get only purple colors
        mask = cv2.inRange(hsv, lower_purple, upper_purple)

        # Optional: Clean the mask
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Loop through contours
        for cnt in contours:
            # Filter by area (to ignore small noise)
            area = cv2.contourArea(cnt)
            if area > 500:  # adjust threshold as needed
                x, y, w, h = cv2.boundingRect(cnt)
                print(f"Gate at position: x={x}, y={y}, width={w}, height={h}")


class Control:
    def __init__(self):
        self.state = 'takeoff'
        self.gates = Gates()
        self.scan_position = np.array([4.0, 4.0, 1.0]) # position to scan for gates
        self.threshold = 0.05 # threshold for positioning
        self.previous_control_command = np.array([0.0, 0.0, 0.0, 0.0]) # previous control command


    def takeoff(self, sensor_data):
        control_command = [sensor_data['x_global'], sensor_data['y_global'], 1.0, sensor_data['yaw']]
        if np.linalg.norm(sensor_data['z_global'] - 1.0) < self.threshold:
            self.state = 'move_to_initial_scan'
        return control_command # Ordered as array with: [pos_x_cmd, pos_y_cmd, pos_z_cmd, yaw_cmd] in meters and radians

    def move_to_initial_scan(self, sensor_data):
        control_command = [self.scan_position[0], self.scan_position[1], self.scan_position[2], sensor_data['yaw']]
        if np.linalg.norm(sensor_data['x_global'] - self.scan_position[0]) < self.threshold and np.linalg.norm(sensor_data['y_global'] - self.scan_position[1]) < self.threshold and np.linalg.norm(sensor_data['z_global'] - self.scan_position[2]) < self.threshold:
            self.state = 'initial_scan'
        return control_command
        
    def initial_scan(self, sensor_data):
        control_command = [self.scan_position[0], self.scan_position[1], self.scan_position[2], sensor_data['yaw']+0.5]
  


    def motion(self, sensor_data):
        if (self.state == 'takeoff'):
            control_command = self.takeoff(sensor_data)
        elif(self.state == 'move_to_initial_scan'):
            control_command = self.move_to_initial_scan(sensor_data)
        elif(self.state == 'initial_scan'):
            control_command = self.initial_scan(sensor_data)
        return control_command



control = Control()
def get_command(sensor_data, camera_data, dt):

    # NOTE: Displaying the camera image with cv2.imshow() will throw an error because GUI operations should be performed in the main thread.
    # If you want to display the camera image you can call it main.py.

    # ---- YOUR CODE HERE ----
    control_command = control.motion(sensor_data)
    return control_command # Ordered as array with: [pos_x_cmd, pos_y_cmd, pos_z_cmd, yaw_cmd] in meters and radians

