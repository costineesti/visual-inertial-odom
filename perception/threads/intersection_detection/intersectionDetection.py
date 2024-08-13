import cv2
import numpy as np

from perception.threads.intersection_detection.preprocess import Preprocess
from perception.threads.intersection_detection.lineTracker import LineTracker


class IntersectionDetection:
    def __init__(self, offset):
        self.preprocess = Preprocess()
        self.lineTracker = LineTracker()

        self.width = 242
        self.height = 93

        self.offset = offset
        self.intersectionDistance = 75
        self.validSigns = ["Stop", "Priority", "Roundabout", "Forbidden"] # Highway exit
 
    def detect(self, image, sign):
        preprocessedImage = self.preprocess.preprocess(image)
        lines = cv2.HoughLinesP(preprocessedImage, 2, np.pi / 180, 5, minLineLength=125, maxLineGap=5)

        if lines is not None:
            horizontalLines = []

            for line in lines:
                x1, y1, x2, y2 = line[0]
                angle = np.arctan2(y2 - y1, x2 - x1)
                horizontalThreshold = np.pi / 18  # approx 10 degrees

                if abs(angle) > horizontalThreshold:
                    continue

                horizontalLines.append(line)

            if len(horizontalLines) != 0:
                meanLine = [int(np.mean([line[0][i] for line in horizontalLines])) for i in range(4)]
                self.lineTracker.update(meanLine)

                trackerLine = self.lineTracker.line
                trackerID = self.lineTracker.id

                if trackerLine is not None:
                    x1, y1, x2, y2 = trackerLine

                    x1 += self.offset[0] # we need to add offset because we are using a cropped image
                    x2 += self.offset[0] # so we can convert the coordinates to the original image
                    y1 += self.offset[1]
                    y2 += self.offset[1]

                    distance = self.distance(trackerLine)
                    if distance < self.intersectionDistance:
                        if sign in self.validSigns or sign is None:
                            # cv2.putText(image, "Intersection detected!", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
                            return {"distance": distance, "sign": sign, "line": [x1, y1, x2, y2], "id": trackerID}
            else:
                self.lineTracker.update(None)
        else:
            self.lineTracker.update(None)

    def distance(self, line):
        x1, y1, x2, y2 = line
        midPoint = (x1 + x2) / 2, (y1 + y2) / 2
        return ((midPoint[0] - self.width / 2) ** 2 + (midPoint[1] - self.height) ** 2) ** 0.5
