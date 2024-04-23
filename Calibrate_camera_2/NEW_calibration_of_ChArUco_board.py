import numpy as np
import cv2
import os

ARUCO_DICT = cv2.aruco.DICT_6X6_250  # Dictionary ID
SQUARES_VERTICALLY = 7               # Number of squares vertically
SQUARES_HORIZONTALLY = 5             # Number of squares horizontally
SQUARE_LENGTH = 150                   # Square side length (in pixels)
MARKER_LENGTH = 120                   # ArUco marker side length (in pixels)
MARGIN_PX = 20                       # Margins size (in pixels)

IMG_SIZE = tuple(i * SQUARE_LENGTH + 2 * MARGIN_PX for i in (SQUARES_VERTICALLY, SQUARES_HORIZONTALLY))
OUTPUT_NAME = 'ChArUco_Marker.png'

def create_and_save_new_board(show=False,save=True):
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard((SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), SQUARE_LENGTH, MARKER_LENGTH, dictionary)
    size_ratio = SQUARES_HORIZONTALLY / SQUARES_VERTICALLY
    img = cv2.aruco.CharucoBoard.generateImage(board, IMG_SIZE, marginSize=MARGIN_PX)
    if show:
        cv2.imshow('ChArUco Board', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    if save:
        cv2.imwrite(OUTPUT_NAME, img)

create_and_save_new_board(1,0)


datadir = "Calibrate_camera_2\Img_for_calibration_2\\"
images = np.array([datadir + f for f in os.listdir(datadir) if f.endswith(".jpg") ])
order = np.argsort([int(p.split("\\")[-1].split(".")[0]) for p in images])
images = images[order]
print(images)
#images

def get_calibration_parameters(image_files):
    # Define the aruco dictionary, charuco board and detector
    dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    board = cv2.aruco.CharucoBoard((SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), SQUARE_LENGTH, MARKER_LENGTH, dictionary)
    params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, params)
    
    # Load images from directory
    #image_files = [os.path.join(img_dir, f) for f in os.listdir(img_dir) if f.endswith(".bmp")]
    all_charuco_ids = []
    all_charuco_corners = []

    # Loop over images and extraction of corners
    for image_file in image_files:
        print("=> Processing image {0}".format(image_file))
        image = cv2.imread(image_file)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        imgSize = image.shape
        image_copy = image.copy()
        marker_corners, marker_ids, rejectedCandidates = detector.detectMarkers(image)
        
        if len(marker_ids) > 0: # If at least one marker is detected
            # cv2.aruco.drawDetectedMarkers(image_copy, marker_corners, marker_ids)
            ret, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, image, board)

            if charucoIds is not None and len(charucoCorners) > 3:
                all_charuco_corners.append(charucoCorners)
                all_charuco_ids.append(charucoIds)
    
    print("### CAMERA CALIBRATION ### - please wait...")
    # Calibrate camera with extracted information
    result, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(all_charuco_corners, all_charuco_ids, board, imgSize, None, None)
    return mtx, dist, ret

datadir = "Calibrate_camera_2\Img_for_calibration_2\\"

mtx, dist, ret = get_calibration_parameters(images)

print("Camera matrix:\n", mtx)
print("Distortion coefficients:\n", dist)

np.savez("Calibrate_camera_2\calib_new_1.npz", ret=ret, mtx=mtx, dist=dist)

"""
Camera matrix:
 [[289.17101938   0.         426.23687843]
 [  0.         289.14205298 401.22256516]
 [  0.           0.           1.        ]]
Distortion coefficients:
 [[-0.22814816  0.04330513 -0.00027584  0.00057192 -0.00322855]]
"""