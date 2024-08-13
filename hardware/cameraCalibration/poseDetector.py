import cv2
import numpy as np


class PoseDetector:
    CHESSBOARD_SIZE = (6, 5)

    # X, Y, Z 
    ROAD = np.float32([[-1, 0 ,0], [6, 0 ,0], [-2, 5, 0], [7, 5, 0]])
    INTERSECTION = np.float32([[-2, 1, 0], [7, 1, 0], [-2, 5, 0], [7, 5, 0]])
    SIGNS = np.float32([[6, 0, -4], [6, 6, -4], [6, 0, -1], [6, 6, -2]])
    # OBSTACLE = np.float32([[-2, 7, 0], [-2, 7, -6], [7, 7, 0], [7, 7, -6]])

    def __init__(self, cameraMatrix, distCoeffs, width=640, height=480):
        self.chessboardFound = False
        self.cameraMatrix = cameraMatrix
        self.distCoeffs = distCoeffs
        self.image = None
        self.width = width
        self.height = height

        self.objectPoints3d = np.zeros((self.CHESSBOARD_SIZE[0] * self.CHESSBOARD_SIZE[1], 3), np.float32)
        self.objectPoints3d[:, :2] = np.mgrid[0:self.CHESSBOARD_SIZE[0], 0:self.CHESSBOARD_SIZE[1]].T.reshape(-1, 2)

        self.ROIS = {}
        self.points2d = {}
        self.defaultPoints2d = {
            "Road": [(118, 127), (332, 127), (0, 210), (450, 210)],
            "Intersection": [(72, 127), (378, 127), (72, 220), (378, 220)],
            "Sign": [(320, 30), (440, 30), (320, 110), (440, 150)]
        }

    def findAreas(self, image):
        self.image = image
        width = self.image.shape[1]
        height = self.image.shape[0]

        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, self.CHESSBOARD_SIZE, None)

        if found:
            _, rvecs, tvecs = cv2.solvePnP(self.objectPoints3d, corners, self.cameraMatrix, self.distCoeffs)

            self.points2d["Road"], _ = cv2.projectPoints(self.ROAD, rvecs, tvecs, self.cameraMatrix, self.distCoeffs)
            self.points2d["Intersection"], _ = cv2.projectPoints(self.INTERSECTION, rvecs, tvecs, self.cameraMatrix, self.distCoeffs)
            self.points2d["Sign"], _ = cv2.projectPoints(self.SIGNS, rvecs, tvecs, self.cameraMatrix, self.distCoeffs)
            # points2d["Obstacle"], _ = cv2.projectPoints(self.OBSTACLE, rvecs, tvecs, self.cameraMatrix, self.distCoeffs)

            for setOfPoints in self.points2d:
                for point in self.points2d[setOfPoints]:
                    if point[0][0] >= width:
                        point[0][0] = width
                    elif point[0][0] <= 0:
                        point[0][0] = 0

                    if point[0][1] >= height:
                        point[0][1] = height
                    elif point[0][1] <= 0:
                        point[0][1] = 0
                
                points = np.array(self.points2d[setOfPoints], dtype=np.int32)
                rect = cv2.boundingRect(points)
                x, y, w, h = rect
                self.ROIS[setOfPoints] = [y, y + h, x, x + w]            
            return True
        return False

    def getUndistortedImage(self, image):
        newCameraMatrix, _ = cv2.getOptimalNewCameraMatrix(self.cameraMatrix, self.distCoeffs, (self.width, self.height), 1, (self.width, self.height))
        return cv2.undistort(image, self.cameraMatrix, None, newCameraMatrix)
    
    def drawPoints(self, image, points2d, color = (0, 0, 255)):
        points2d = np.int32(points2d).reshape(-1, 2)
        for point in points2d:
            image = cv2.circle(image, (point[0], point[1]), radius = 2, color = color, thickness = -1)
        return image

    def getRoi(self, roi):
        area = self.ROIS[roi]
        return self.image[area[0] : area[1], area[2] : area[3]]
    
    def get2dCoords(self):
        for roi in self.points2d:
            self.points2d[roi] = np.int32(self.points2d[roi]).reshape(-1, 2)
        return self.points2d
    
    def getDefault2dCoords(self):
        return self.defaultPoints2d
    

if __name__ == "__main__":
    from utils.helpers import readCameraParamsFromFile

    cameraMatrix, distCoeffs = readCameraParamsFromFile("hardware/cameraCalibration/calibration.npy")

    poseDetector = PoseDetector(cameraMatrix, distCoeffs)

    imageOriginal = cv2.imread("hardware/cameraCalibration/images/image0.jpg")
    image = poseDetector.getUndistortedImage(imageOriginal)
    
    if poseDetector.findAreas(imageOriginal):
        points2d = poseDetector.get2dCoords()
        image = poseDetector.drawPoints(image, points2d["Road"])
        image = poseDetector.drawPoints(image, points2d["Intersection"], color=(255, 0, 255))
        image = poseDetector.drawPoints(image, points2d["Sign"], color=(255, 255, 0))   

    cv2.imshow("Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()