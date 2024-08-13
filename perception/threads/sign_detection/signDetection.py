import cv2
import numpy as np
import random

from keras.models import load_model


class SignDetection:
    def __init__(self):
        self.colorSegmentation = ColorSegmentation()
        self.imageClassifier = ImageClassifier()
        self.colorLength = 4

    def detect(self, image, saveData=False):
        HSVImage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        HSVImage[:,:,2] = cv2.medianBlur(HSVImage[:,:,2], 3)

        enhancedImage = ImageEnhancement(HSVImage[:,:,2]).unsharpImage(2).getImage()

        for color in range(self.colorLength):
            colorMask = self.colorSegmentation.segment(HSVImage, color).reduceNoise().getColorMask()
            colorLimitedImage = cv2.bitwise_and(enhancedImage, colorMask)

            segmentation = ImageSegmentation(colorLimitedImage).createBinaryMap().getContours().analyzeShape()
            rectData = segmentation.rectData

            if not rectData:
                continue
        
            bestRect = rectData[self.filterRect(rectData)]
            croppedImage = self.cropImage(image, bestRect)
            sign = self.imageClassifier.classify(croppedImage)

            if saveData:
                croppedImage = cv2.resize(croppedImage, (32, 32))

                imageID = random.randint(10000, 99999)

                if sign == "Unknown":
                    cv2.imwrite(f"signs/Unknown/{imageID}.png", croppedImage)
                
                if sign == "Crosswalk":
                    cv2.imwrite(f"signs/Crosswalk/{imageID}.png", croppedImage)

                if sign == "Oneway":
                    cv2.imwrite(f"signs/Oneway/{imageID}.png", croppedImage)
                
                if sign == "Parking":
                    cv2.imwrite(f"signs/Parking/{imageID}.png", croppedImage)

                if sign == "Priority":
                    cv2.imwrite(f"signs/Priority/{imageID}.png", croppedImage)

                if sign == "Roundabout":
                    cv2.imwrite(f"signs/Roundabout/{imageID}.png", croppedImage)

                if sign == "Stop":
                    cv2.imwrite(f"signs/Stop/{imageID}.png", croppedImage)

                if sign == "Forbidden":
                    cv2.imwrite(f"signs/Forbidden/{imageID}.png", croppedImage)

                if sign == "Highway entrance":
                    cv2.imwrite(f"signs/HighwayEntrance/{imageID}.png", croppedImage)

                if sign == "Highway exit":
                    cv2.imwrite(f"signs/HighwayExit/{imageID}.png", croppedImage)

            if sign != "Unknown":
                return {"type": sign, "coords": bestRect, "image": croppedImage}
            
        return None

    def filterRect(self, rectData):
        best = 0
        for i in range(len(rectData)):
            x1, y1, x2, y2 = rectData[i]
            current = (x2 - x1) * (y2 - y1)
            if current > best:
                best = current
                bestIndex = i
        return bestIndex
    
    def cropImage(self, image, rectData):
        x1, y1, x2, y2 = rectData
        return image[y1:y2, x1:x2]


class ImageEnhancement:
    def __init__(self, image):
        self.image = image
        self.unsharpMask = np.zeros_like(self.image)
        self.sharpenedImage = np.zeros_like(self.image)
    
    def unsharpImage(self, sharpening, kSize=5, sigma=10):
        blurredImage = cv2.GaussianBlur(self.image, (kSize, kSize), sigma, sigma)
        cv2.addWeighted(self.image, 1, blurredImage, -1, 0, self.unsharpMask)
        cv2.addWeighted(self.image, 1, self.unsharpMask, sharpening, 0, self.sharpenedImage)
        return self

    def getImage(self):
        return self.sharpenedImage


class ColorSegmentation:
    def __init__(self):
        # 0: BLUE, 1: RED, 2: GREEN, 3: YELLOW
        self.colorMinRanges = {
            0: [(100, 60, 30)],
            1: [(0, 20, 40), (160, 10, 30)],
            2: [(75, 25, 80)],
            3: [(20, 35, 130)]
        }

        self.colorMaxRanges = {
            0: [(125, 255, 255)],
            1: [(10, 255, 255), (180, 255, 255)],
            2: [(90, 255, 255)],
            3: [(30, 255, 255)]
        }

        self.morphKernel = np.ones((5,5), np.uint8)

    def segment(self, image, colorKey):
        if len(self.colorMinRanges[colorKey]) == 1:
            self.colorMask = cv2.inRange(image, self.colorMinRanges[colorKey][0], self.colorMaxRanges[colorKey][0])
        else:
            for i in range(len(self.colorMinRanges[colorKey])):
                currentMask = cv2.inRange(image, self.colorMinRanges[colorKey][i], self.colorMaxRanges[colorKey][i])

                if i == 0:
                    self.colorMask = currentMask
                else:
                    self.colorMask = cv2.bitwise_or(self.colorMask, currentMask)
        return self

    def reduceNoise(self):
        self.colorMask = cv2.morphologyEx(self.colorMask, cv2.MORPH_OPEN, self.morphKernel)
        self.colorMask = cv2.dilate(self.colorMask, self.morphKernel)
        return self

    def getColorMask(self):
        return self.colorMask


class ImageSegmentation:
    def __init__(self, image, kSize=7):
        self.image = image
        self.morphKernel = np.ones((kSize, kSize), np.uint8)

    def createBinaryMap(self, minTruncLevel=70, maxTruncLevel=190):
        self.binaryMap = cv2.threshold(self.image,minTruncLevel, maxTruncLevel, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        self.binaryMap = cv2.erode(self.binaryMap, self.morphKernel)
        return self
    
    def getContours(self):
        self.contours = cv2.findContours(self.binaryMap, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        return self
    
    def analyzeShape(self):
        self.rectData = []

        for shape in self.contours:
            area = cv2.contourArea(shape)

            if area < 2500: # 3500
                continue

            hull = cv2.convexHull(shape)
            epsilon = 0.04 * cv2.arcLength(hull,True)
            approx = cv2.approxPolyDP(hull, epsilon, True)
            sidesApprox = len(approx)

            if sidesApprox == 3:
                polygon = "triangle"
            elif sidesApprox == 4:
                (x, y, w, h) = cv2.boundingRect(approx)
                ar = w / float(h)
                polygon = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
            else:
                continue

            M = cv2.moments(hull)

            if not 1200 < M["m00"] < 40000: # 2500
                continue

            cX = int((M["m10"] / M["m00"]))
            cY = int((M["m01"] / M["m00"]))
            
            x, y, w, h = cv2.boundingRect(hull)
            aspectRatio = float(w) / h

            if not 0.7 < aspectRatio < 1.3:
                continue

            self.rectData.append([x, y, x + w, y + h])

            cv2.rectangle(self.image, (x, y), (x + w, y + h), (255, 255, 255), 2)
            cv2.drawContours(self.image, [shape], -1, (255, 255, 255), 2)
            cv2.putText(self.image, polygon, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        return self


class ImageClassifier:
    CLASSES = { 
        0: "Crosswalk",
        1: "Oneway",
        2: "Parking",
        3: "Priority",
        4: "Roundabout",
        5: "Stop",
        6: "Forbidden",
        7: "Highway entrance",
        8: "Highway exit",
        9: "Unknown"
    }

    def __init__(self):
        self.model = load_model("perception/threads/sign_detection/CNN_model.h5")

    def classify(self, sign):
        if sign.shape != (32, 32, 3):
            sign = cv2.resize(sign, (32, 32))

        if sign.shape != (1, 32, 32, 3):
            sign = sign.reshape(1, *sign.shape)

        signGray = np.sum(sign / 3, axis=3, keepdims=True)
        signGrayNorm = (signGray - 32) / 32

        predictions = self.model.predict(signGrayNorm, verbose=0)
        predictedLabels = np.argmax(predictions)

        return self.CLASSES[predictedLabels]
