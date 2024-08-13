from utils.constants import IMAGE_WIDTH
from utils.helpers import findClosestValueForKey, findSublist


class DistanceError:
    def __init__(self):
        self.roadWidthInCM = 35 # cm
        self.linesDifferenceDict = {} # {y_level: difference}

    def getError(self, laneData):
        if laneData["lines"]["middle"] is not None:
            errorTop = IMAGE_WIDTH / 2 - laneData["lines"]["middle"][0][0]
            errorBottom = IMAGE_WIDTH / 2 - laneData["lines"]["middle"][-1][0]

            errorTop = self.pixelsToCentimeters(errorTop, laneData, laneData["lines"]["middle"][0][1])
            errorBottom = self.pixelsToCentimeters(errorBottom, laneData, laneData["lines"]["middle"][-1][1])

            return errorTop, errorBottom
        
        return None, None
    
    def pixelsToCentimeters(self, px, laneData, yLevel):
        invalidLeft = laneData["lines"]["left"] is None or (
            yLevel < laneData["lines"]["left"][0][1] or
            yLevel > laneData["lines"]["left"][-1][1]
        )

        invalidRight = laneData["lines"]["right"] is None or (
            yLevel < laneData["lines"]["right"][0][1] or
            yLevel > laneData["lines"]["right"][-1][1]
        )

        if invalidLeft or invalidRight:
            if len(self.linesDifferenceDict) > 0:
                if yLevel not in self.linesDifferenceDict:
                    difference = findClosestValueForKey(self.linesDifferenceDict, yLevel)
                else:
                    difference = self.linesDifferenceDict[yLevel] 
            else:
                return None
        else:
            leftCoords = findSublist(laneData["lines"]["left"], yLevel)
            rightCoords = findSublist(laneData["lines"]["right"], yLevel)
            difference = abs(leftCoords[0] - rightCoords[0])
            self.linesDifferenceDict[yLevel] = difference
        
        if difference == 0:
            difference = 35 # Avoiding division by zero.
        return "{:.2f}".format(px * self.roadWidthInCM / difference) # (distance in px) * 35cm / 35 cm in px
    
