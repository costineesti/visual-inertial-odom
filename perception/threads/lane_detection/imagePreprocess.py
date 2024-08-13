import cv2
# import numpy as np


class ImagePreprocess:

    @staticmethod    
    def preprocess(image):
        grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurredImage = cv2.GaussianBlur(grayImage, (5, 5), 0)
        sobel = cv2.Sobel(blurredImage, cv2.CV_8U, 1, 1, ksize=3)
        _, sobelThresh = cv2.threshold(sobel, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        # kernel = np.ones((12,12),np.uint8)
        # morphClose= cv2.morphologyEx(sobelThresh, cv2.MORPH_CLOSE, kernel)

        return sobelThresh
