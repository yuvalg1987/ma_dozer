import cv2
import sys
import numpy as np

sys.path.append('.')
sys.path.append('../')

ARUCO_DICT = {
    "4X4_50": cv2.aruco.DICT_4X4_50,
    "4X4_100": cv2.aruco.DICT_4X4_100,
    "4X4_250": cv2.aruco.DICT_4X4_250,
    "4X4_1000": cv2.aruco.DICT_4X4_1000,
    "5X5_50": cv2.aruco.DICT_5X5_50,
    "5X5_100": cv2.aruco.DICT_5X5_100,
    "5X5_250": cv2.aruco.DICT_5X5_250,
    "5X5_1000": cv2.aruco.DICT_5X5_1000,
    "6X6_50": cv2.aruco.DICT_6X6_50,
    "6X6_100": cv2.aruco.DICT_6X6_100,
    "6X6_250": cv2.aruco.DICT_6X6_250,
    "6X6_1000": cv2.aruco.DICT_6X6_1000,
    "7X7_50": cv2.aruco.DICT_7X7_50,
    "7X7_100": cv2.aruco.DICT_7X7_100,
    "7X7_250": cv2.aruco.DICT_7X7_250,
    "7X7_1000": cv2.aruco.DICT_7X7_1000,
    "ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}


class ArucoDetector():
    """
    A class that aims to wrap the functionality of detecting an Aruco marker within an image
    :param grid_size : the Aruco marker's grid_size
    :param marker_num : the number of possible Aruco markers within the Dict
    :param marker_edge_length : the length of the marker in cm
    Both params define a unique Aruco dictionary that will be used for detection
    """
    def __init__(self, grid_size : int = 5, marker_num : int = 100, marker_length : float = 19.1):

        self.grid_size = grid_size
        self.marker_num = marker_num
        self.aruco_key = f'{grid_size}X{grid_size}_{marker_num}'
        self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[self.aruco_key])
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        self.marker_length = marker_length # The size in cm of the marker, the rotation/translation vectors will be estimates in the same unit

        self.parameters = cv2.aruco.DetectorParameters_create()


    def draw_markers(self, color_image, corners, ids, intrinsics_params, dist_coeffs, rotation_vecs_i2c, translation_vecs_c2i_c):
        cv2.aruco.drawDetectedMarkers(color_image, corners, ids)

        for (marker_idx, curr_marker_corners) in enumerate(corners):
            cv2.aruco.drawAxis(color_image, intrinsics_params, dist_coeffs,
                               rotation_vecs_i2c[marker_idx, ...],
                               translation_vecs_c2i_c[marker_idx, ...], 5)


    def find_markers(self, color_frame : np.array, camera_matrix : np.array, dist_coeffs : np.array):

        gray_image = cv2.cvtColor(color_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.parameters)


        rotation_vecs_i2c, translation_vecs_c2i_c, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners,
                                                                                                    self.marker_length,
                                                                                                    camera_matrix,
                                                                                                    dist_coeffs)
        if rotation_vecs_i2c is None or translation_vecs_c2i_c is None:
            return None, None, None, None

        rotation_vecs_i2c = rotation_vecs_i2c.astype(np.float32)
        translation_vecs_c2i_c = translation_vecs_c2i_c.astype(np.float32)
        return ids, corners, rotation_vecs_i2c, translation_vecs_c2i_c

    def estimate_inertial_system(self, ids, translation_vecs_c2i_c) :

        marker0_idx = np.where(ids == 0)[0].item()
        marker1_idx = np.where(ids == 1)[0].item()
        marker2_idx = np.where(ids == 2)[0].item()

        x_world_in_c = (translation_vecs_c2i_c[marker2_idx, :] - translation_vecs_c2i_c[marker0_idx, :])
        y_world_in_c = (translation_vecs_c2i_c[marker1_idx, :] - translation_vecs_c2i_c[marker0_idx, :])
        z_world_in_c = np.cross(x_world_in_c, y_world_in_c)
        y_world_in_c = np.cross(z_world_in_c, x_world_in_c)

        o_world_in_c = translation_vecs_c2i_c[marker0_idx, :].T
        x_world_in_c = (x_world_in_c / np.linalg.norm(x_world_in_c)).T
        y_world_in_c = (y_world_in_c / np.linalg.norm(y_world_in_c)).T
        z_world_in_c = (z_world_in_c / np.linalg.norm(z_world_in_c)).T

        t_c2w_c = o_world_in_c
        rot_c2w = np.vstack((x_world_in_c.T, y_world_in_c.T, z_world_in_c.T))
        t_w2c_w = -rot_c2w @ t_c2w_c

        # t_c2w_c = o_world_in_c
        # rot_w2c = np.vstack((x_world_in_c.T, y_world_in_c.T, z_world_in_c.T))
        #
        # rot_c2w = rot_w2c.T
        # t_w2c_w = -rot_c2w @ t_c2w_c

        return rot_c2w, t_w2c_w


class ArucoGenerator():
    """"
    A class that aims to wrap the functinality of generating an Aruco marker. This is required for printing
    as these markers should appear on the Dozer it self
    :param grid_size : the Aruco marker's grid_size
    :param marker_num : the number of possible Aruco markers within the Dict
    Both params define a unique Aruco dictionary that will be used for detection
    """
    def __init__(self, grid_size : int = 5, marker_num : int = 100):

        self.grid_size = grid_size
        self.marker_num = marker_num
        self.aruco_key = f'{grid_size}X{grid_size}_{marker_num}'
        self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[self.aruco_key])

    """
    A function that generates an Aruco marker according to its index, where the markers dictionary is already initialized 
    """
    def generate_aruco_marker(self, marker_idx):

        curr_marker = np.zeros((300, 300, 1), dtype="uint8")
        cv2.aruco.drawMarker(self.aruco_dict, marker_idx, 300, curr_marker, 1)
        return curr_marker