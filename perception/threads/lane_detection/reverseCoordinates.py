import numpy as np


class ReverseCoordinates:

    @staticmethod
    def reverseLineCoordinates(lanes, inverseTransformMatrix):
        transformMatrix = inverseTransformMatrix

        for lane in lanes:
            if lane is None:
                continue

            coords = np.hstack((lane, np.ones((lane.shape[0], 1), dtype=np.float32)))

            transformedCoords = np.dot(coords, transformMatrix.T)
            transformedCoords /= transformedCoords[:, 2][:, np.newaxis]

            lane[:, 0] = transformedCoords[:, 0].astype(int)
            lane[:, 1] = transformedCoords[:, 1].astype(int)

    @staticmethod
    def reverseWindowsCoordinates(windows, inverseTransformMatrix):
        if windows is None:
            return
        
        transformMatrix = inverseTransformMatrix

        for window in windows:
            for pointPair in window:
                coords = np.hstack((pointPair, [1.0]))

                transformedCoords = np.dot(coords, transformMatrix.T)
                transformedCoords /= transformedCoords[2]

                pointPair[0] = int(transformedCoords[0])
                pointPair[1] = int(transformedCoords[1])
