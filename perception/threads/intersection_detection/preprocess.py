import cv2


class Preprocess:
    def __init__(self):
        pass

    def preprocess(self, image):
        grayImage = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurredImage = self.multiScaleGaussianBlur(grayImage, [3, 5, 7], [1.0, 1.5, 2.0])
        
        sobelY = cv2.Sobel(blurredImage, cv2.CV_8U, 0, 1, ksize=3)
        _, sobelThresh = cv2.threshold(sobelY, 100, 255, cv2.THRESH_BINARY)
        return sobelThresh
    
    def multiScaleGaussianBlur(self, image, kernel_sizes, standard_deviations):
        blurred_image = image
        for kernel_size, standard_deviation in zip(kernel_sizes, standard_deviations):
            blurred_image = cv2.GaussianBlur(blurred_image, (kernel_size, kernel_size), standard_deviation)
        return blurred_image