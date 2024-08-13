import numpy as np


class ReverseCoordinates:
    
    @staticmethod
    def reverseCoordinates(coords, inverseTransformMatrix) :
        transformMatrix = inverseTransformMatrix

        for i in range(len(coords)):
            coord = coords[i]

            homogeneousCoord = np.append(coord, 1)
            transformedCoord = np.dot(transformMatrix, homogeneousCoord)
            transformedCoord = transformedCoord[:2] / transformedCoord[2]

            coords[i] = transformedCoord.astype(int).tolist()

        return coords
    