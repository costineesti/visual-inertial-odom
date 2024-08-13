import numpy as np


class LaneDetect:
    # ============================== INIT ==============================
    def __init__(self, width, height):
        self.width = width
        self.height = height

        self.laneWidth = 230 # in pixels

        self.windowsNumber = 7
        self.widowMinPixels = np.int32(width / 24)

        self.windowWidth = np.int32(width / 12)
        self.windowHeight = np.int32(height / self.windowsNumber)

        self.sanity = False
        self.lastLeftFit = None
        self.lastRightFit = None
        
    # ============================== DETECT LANE ==============================
    def detect(self, image):
        leftWindows = None
        rightWindows = None

        if self.sanity:
            leftLine, rightLine = self.fastSearch(image, self.lastLeftFit, self.lastRightFit)
        else:
            leftLine, rightLine, leftWindows, rightWindows = self.searchLane(image)

        self.sanity = self.sanityCheck(leftLine, rightLine, debug=False)
            
        self.lastLeftFit = leftLine["fit"]
        self.lastRightFit = rightLine["fit"]
        leftLineCoordinates, rightLineCoordinates = self.getLaneLines(leftLine, rightLine)
        middleLineCoordinates = self.getMiddleLine(leftLineCoordinates, rightLineCoordinates)

        return {"lines": {"left": leftLineCoordinates, "right": rightLineCoordinates, "middle": middleLineCoordinates}, "windows": {"left": leftWindows, "right": rightWindows}}
    
    # ============================== SEARCH LANE ==============================
    def searchLane(self, image):
        leftPeak, rightPeak = self.getHistogramPeaks(image)

        nonzero = image.nonzero()
        nonzeroX = np.array(nonzero[1])
        nonzeroY = np.array(nonzero[0])

        leftIndices = []
        rightIndices = []

        leftCount = 0
        rightCount = 0

        leftWindowsCoords = []
        rightWindowsCoords = []

        for window in range(self.windowsNumber):
            windowYLow = self.height - (window + 1) * self.windowHeight
            windowYHigh = self.height - window * self.windowHeight

            if leftPeak and leftCount < 1:
                windowXLeftLow = leftPeak - self.windowWidth
                windowXLeftHigh = leftPeak + self.windowWidth

                leftWindowsCoords.append([[windowXLeftLow, windowYLow], [windowXLeftHigh, windowYHigh]])

                leftIndicesInWindow = ((nonzeroY >= windowYLow) & (nonzeroY < windowYHigh) & (nonzeroX >= windowXLeftLow) & (nonzeroX < windowXLeftHigh)).nonzero()[0]
                leftIndices.append(leftIndicesInWindow)

                if len(leftIndicesInWindow) > self.widowMinPixels:
                    leftPeak = np.int32(np.mean(nonzeroX[leftIndicesInWindow]))
                else:
                    leftCount += 1

            if rightPeak and rightCount < 1:
                windowXRightLow = rightPeak - self.windowWidth
                windowXRightHigh = rightPeak + self.windowWidth

                rightWindowsCoords.append([[windowXRightLow, windowYLow], [windowXRightHigh, windowYHigh]])

                rightIndicesInWindow = ((nonzeroY >= windowYLow) & (nonzeroY < windowYHigh) & (nonzeroX >= windowXRightLow) & (nonzeroX < windowXRightHigh)).nonzero()[0]
                rightIndices.append(rightIndicesInWindow)

                if len(rightIndicesInWindow) > self.widowMinPixels:
                    rightPeak = np.int32(np.mean(nonzeroX[rightIndicesInWindow]))
                else:
                    rightCount += 1

        if leftIndices:
            leftIndices = np.concatenate(leftIndices)

        if rightIndices:
            rightIndices = np.concatenate(rightIndices)

        leftPoints = [nonzeroX[leftIndices], nonzeroY[leftIndices]] if len(leftIndices) >= 50 else None
        rightPoints = [nonzeroX[rightIndices], nonzeroY[rightIndices]] if len(rightIndices) >= 50 else None

        leftLine = {"fit": None}
        rightLine = {"fit": None}

        if leftPoints:
            leftLine["fit"] = np.polyfit(leftPoints[1], leftPoints[0], 2)
            leftLine["min"] = min(leftPoints[1])
            leftLine["max"] = max(leftPoints[1])
            
        if rightPoints:
            rightLine["fit"] = np.polyfit(rightPoints[1], rightPoints[0], 2)
            rightLine["min"] = min(rightPoints[1])
            rightLine["max"] = max(rightPoints[1])

        return leftLine, rightLine, leftWindowsCoords, rightWindowsCoords
    
    def fastSearch(self, image, leftFit, rightFit):
        nonzero = image.nonzero()
        nonzeroX = np.array(nonzero[1])
        nonzeroY = np.array(nonzero[0])

        leftIndices = ((nonzeroX > (leftFit[0] * np.power(nonzeroY, 2) + leftFit[1] * nonzeroY + leftFit[2] - self.windowWidth))
                    & (nonzeroX < (leftFit[0] * np.power(nonzeroY, 2) + leftFit[1] * nonzeroY + leftFit[2] + self.windowWidth)))
        
        rightIndices = ((nonzeroX > (rightFit[0] * np.power(nonzeroY, 2) + rightFit[1] * nonzeroY + rightFit[2] - self.windowWidth))
                    & (nonzeroX < (rightFit[0] * np.power(nonzeroY, 2) + rightFit[1] * nonzeroY + rightFit[2] + self.windowWidth)))
        
        leftPoints = [nonzeroX[leftIndices], nonzeroY[leftIndices]] if len(leftIndices) >= 50 else None
        rightPoints = [nonzeroX[rightIndices], nonzeroY[rightIndices]] if len(rightIndices) >= 50 else None

        leftLine = {"fit": None}
        rightLine = {"fit": None}

        if leftPoints[0].any() and leftPoints[1].any():
            leftLine["fit"] = np.polyfit(leftPoints[1], leftPoints[0], 2)
            leftLine["min"] = min(leftPoints[1])
            leftLine["max"] = max(leftPoints[1])
            
        if rightPoints[0].any() and rightPoints[1].any():
            rightLine["fit"] = np.polyfit(rightPoints[1], rightPoints[0], 2)
            rightLine["min"] = min(rightPoints[1])
            rightLine["max"] = max(rightPoints[1])

        return leftLine, rightLine

    # ============================== SANITY CHECK ==============================
    def sanityCheck(self, leftLine, rightLine, debug=False):
        if leftLine["fit"] is None or rightLine["fit"] is None:
            return False
    
        leftY = np.linspace(leftLine["min"], leftLine["max"] - 1, leftLine["max"] - leftLine["min"])
        leftFitX = leftLine["fit"][0] * leftY ** 2 + leftLine["fit"][1] * leftY + leftLine["fit"][2]

        rightY = np.linspace(rightLine["min"], rightLine["max"] - 1, rightLine["max"] - rightLine["min"])
        rightFitX = rightLine["fit"][0] * rightY ** 2 + rightLine["fit"][1] * rightY + rightLine["fit"][2]

        if leftFitX.shape[0] < rightFitX.shape[0]:
            rightFitX = rightFitX[-leftFitX.shape[0]:]

        if rightFitX.shape[0] < leftFitX.shape[0]:
            leftFitX = leftFitX[-rightFitX.shape[0]:]

        deltaLines = np.mean(rightFitX - leftFitX)

        if not 200 <= deltaLines <= 270:
            print(f"deltaLines: {deltaLines}") if debug else None
            return False
        
        left_slope = 2 * leftLine["fit"][0] * self.height + leftLine["fit"][1]
        right_slope = 2 * rightLine["fit"][0] * self.height + rightLine["fit"][1]

        leftAngle = self.calculateAngle(left_slope)
        rightAngle = self.calculateAngle(right_slope)

        if abs(leftAngle - rightAngle) > 10:
            print(f"diff: {abs(leftAngle - rightAngle)}") if debug else None
            return False
        
        if abs(leftAngle) > 7 or abs(rightAngle) > 7:
            print(f"leftAngle: {leftAngle}, rightAngle: {rightAngle}") if debug else None
            return False

        return True

    # ============================== HELPER ==============================
    def getHistogram(self, image, percent=0.75):
        return np.sum(image[int(image.shape[0] * percent):, :], axis=0)
    
    def getHistogramPeaks(self, image):
        percent = 0.75
        histogram = self.getHistogram(image, percent)

        while True:
            midpoint = np.int32(histogram.shape[0] / 2)

            leftPeak = np.argmax(histogram[:midpoint])
            if histogram[leftPeak] == 0:
                leftPeak = None

            # we need to invert the right histogram because the argmax function returns the first occurence of the max value and we need the last
            rightHistogram = histogram[midpoint:]
            rightPeak = np.where(rightHistogram == np.max(rightHistogram))[0][-1] + midpoint
            if histogram[rightPeak] == 0:
                rightPeak = None

            if leftPeak is None or rightPeak is None:
                return leftPeak, rightPeak

            if abs(leftPeak - rightPeak) > 150:
                return leftPeak, rightPeak

            percent += 0.05
            if percent > 1:
                return None, None

            histogram = self.getHistogram(image, percent)

    def getLaneLines(self, leftLine, rightLine):
        leftLineCoordinates = None
        rightLineCoordinates = None

        if leftLine["fit"] is not None:
            leftLineCoordinates = self.getLine(leftLine)

        if rightLine["fit"] is not None:
            rightLineCoordinates = self.getLine(rightLine)

        return leftLineCoordinates, rightLineCoordinates
    
    def getLine(self, line):
        y = np.linspace(line["min"], line["max"] - 1, line["max"] - line["min"])
        x = line["fit"][0] * y ** 2 + line["fit"][1] * y + line["fit"][2]
        return np.column_stack((x.astype(np.int32), y.astype(np.int32)))
    
    def calculateAngle(self, slope):
        angleRad = np.arctan(slope)
        angleDeg = np.degrees(angleRad)
        return angleDeg

    def getMiddleLine(self, leftLine, rightLine):
        if leftLine is None and rightLine is None:
            return None

        if leftLine is None or (leftLine.shape[0] <= 150 and rightLine is not None and rightLine.shape[0] > leftLine.shape[0]):
            return np.column_stack((rightLine[:, 0] - self.laneWidth // 2, rightLine[:, 1]))
        
        if rightLine is None or (rightLine.shape[0] <= 150 and leftLine is not None and leftLine.shape[0] > rightLine.shape[0]):
            return np.column_stack((leftLine[:, 0] + self.laneWidth // 2, leftLine[:, 1]))
        
        # if leftLine has fewer points than rightLine
        if leftLine.shape[0] < rightLine.shape[0]:
            rightLine = rightLine[-leftLine.shape[0]:]
        
        # if rightLine has fewer points than leftLine
        if rightLine.shape[0] < leftLine.shape[0]:
            leftLine = leftLine[-rightLine.shape[0]:]
        
        return np.mean([leftLine, rightLine], axis=0).astype(np.int32)
        #return np.column_stack((rightLine[:, 0] - self.laneWidth // 2, rightLine[:, 1]))
