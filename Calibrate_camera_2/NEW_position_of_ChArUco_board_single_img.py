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

#location of images for calibration
DATADIR = "Calibrate_camera_2\Img_for_calibration_2\\"

#calibration file
CALIBRATION_FILE = "Calibrate_camera_2\calib_new_1.npz"

SQUARE_LENGTH_M = 0.0582                   # chessboard square side length (normally in meters)
MARKER_LENGTH_M = 0.0466                   # 	marker side length (same unit than squareLength)

img_id=7



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
    return board

board=create_and_save_new_board(1,0)


images = np.array([DATADIR + f for f in os.listdir(DATADIR) if f.endswith(".jpg") ])
order = np.argsort([int(p.split("\\")[-1].split(".")[0]) for p in images])
images = images[order]
print(images)



img=cv2.imread(images[img_id])

with np.load(CALIBRATION_FILE) as X:
    ret,mtx, dist = [X[i] for i in ("ret","mtx", "dist")]

h,  w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
img_undist = cv2.undistort(img, mtx, dist, None, newcameramtx)

cv2.imshow("img_undist",img_undist)
cv2.waitKey(0)
cv2.imshow("img_undist",img_undist[100:-100,100:-100])
cv2.waitKey(0)
cv2.destroyAllWindows()

image=img_undist



all_charuco_ids = []
all_charuco_corners = []

dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
board = cv2.aruco.CharucoBoard((SQUARES_VERTICALLY, SQUARES_HORIZONTALLY), SQUARE_LENGTH_M, MARKER_LENGTH_M, dictionary)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, params)

marker_corners, marker_ids, rejectedCandidates = detector.detectMarkers(image)
if marker_ids is not None and len(marker_ids) > 0: # If at least one marker is detected
    # cv2.aruco.drawDetectedMarkers(image_copy, marker_corners, marker_ids)
    ret, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, image, board)
    if charucoCorners is not None and charucoIds is not None and len(charucoCorners) > 3:
        all_charuco_corners.append(charucoCorners)
        all_charuco_ids.append(charucoIds)

    
    retval, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(np.array(all_charuco_corners)[0], np.array(all_charuco_ids)[0], board, np.array(mtx), np.array(dist), np.empty(1), np.empty(1))

    if retval:
        cv2.aruco.drawDetectedMarkers(image, marker_corners, marker_ids)
        cv2.drawFrameAxes(image, mtx, dist, rvec, tvec, length=0.1, thickness=5)

    cv2.imshow("img",image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    Zx, Zy, Zz = tvec[0][0], tvec[1][0], tvec[2][0]
    fx, fy = mtx[0][0], mtx[1][1]

    print(f'Zz = {Zz}\nfx = {fx}')

def perspective_function(x, Z, f): 
    return x*Z/f

nb_pixels = 192
print(perspective_function(nb_pixels, Zz, fx))

