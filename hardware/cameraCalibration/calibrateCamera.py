import cv2
import numpy as np
import os
import re
from picamera2 import Picamera2
from glob import glob
from time import sleep

# ======================================== CONFIGURATION ========================================
FRAME_SIZE = (450, 240)
CHESSBOARD_SIZE = (5, 6)
CHESSBOARD_SQUARE_SIZE = 50 # mm


# ======================================== FUNCTIONS ========================================
def captureCalibrationPhotos(takePhotos=False, keepOldPhotos=True):
    if not takePhotos:
        return
    
    imagesPath = glob("hardware/cameraCalibration/images/image*.jpg")
    pathLength = len(imagesPath)

    if pathLength != 0 and not keepOldPhotos:
        for imagePath in imagesPath:
            os.remove(imagePath)

    if pathLength == 0 or not keepOldPhotos:
        index = 0
    else:
        biggestIndex = max(extractNumber(imagePath) for imagePath in imagesPath)
        index = biggestIndex + 1

    camera = Picamera2()
    previewConfig = camera.create_preview_configuration(
        buffer_count=1,
        queue=False,
        main={"format": "RGB888", "size": (450, 240)}, # 2048, 1080
        lores={"size": (320, 240)},
        encode="lores"
    )

    camera.configure(previewConfig)
    camera.start()

    sleep(2)
    print("Press 's' to save the current frame")
    print("Press 'q' to quit")

    while True:
        request = camera.capture_array("main")
        cv2.imshow("Preview", request)

        key = cv2.waitKey(1)

        if key == ord("q"):
            break
        elif key == ord("s"):
            cv2.imwrite(f"hardware/cameraCalibration/images/image{index}.jpg", request)
            index += 1

    cv2.destroyAllWindows()


def findChessboardCorners(imagesPath):
    objectPoints3d = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
    objectPoints3d[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
    objectPoints3d *= CHESSBOARD_SQUARE_SIZE

    points3d = []
    points2d = []

    for index, imagePath in enumerate(imagesPath):
        image = cv2.imread(imagePath)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            points3d.append(objectPoints3d)
            points2d.append(corners2)

            cv2.drawChessboardCorners(image, CHESSBOARD_SIZE, corners2, ret)
            cv2.imwrite(f"output/corners{index}.jpg", image)

    return points3d, points2d


def extractNumber(string):
    return int(re.search(r"\d+", string).group())


# ======================================== MAIN ========================================
def main(takePhotos, keepOldPhotos):
    captureCalibrationPhotos(takePhotos, keepOldPhotos)
    imagesPath = glob("hardware/cameraCalibration/images/image*.jpg")

    if len(imagesPath) == 0:
        print("No images found for camera calibration!")
        return 
    
    points3d, points2d = findChessboardCorners(imagesPath)

    # calibrate the camera
    if points2d and points3d:
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(points3d, points2d, FRAME_SIZE, None, None)

        # save the calibration
        if not ret:
            print("Calibration failed")
            return
        
        calibration = {"camera_matrix": mtx, "dist_coeff": dist, "rvecs": rvecs, "tvecs": tvecs}
        np.save("hardware/cameraCalibration/calibration.npy", calibration)
    else:
        print("Points not found for calibration")


if __name__ == "__main__":
    main()
