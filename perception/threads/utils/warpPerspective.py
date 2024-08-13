import cv2


class WarpPerspective:
    def __init__(self, regionOfInterest, width, height):
        self.regionOfInterest = regionOfInterest
        self.imageShape = (width, height)
        self.transformMatrix, self.inverseTransformMatrix = self.CalculateTransformMatrices()

    def CalculateTransformMatrices(self):
        src, dst = self.regionOfInterest["src"], self.regionOfInterest["dst"]
        transformMatrix = cv2.getPerspectiveTransform(src, dst)
        inverseTransformMatrix = cv2.getPerspectiveTransform(dst, src)

        return transformMatrix, inverseTransformMatrix

    def transform(self, image):
        return cv2.warpPerspective(image, self.transformMatrix, self.imageShape)

    def inverseTransform(self, image):
        return cv2.warpPerspective(image, self.inverseTransformMatrix, self.imageShape)
