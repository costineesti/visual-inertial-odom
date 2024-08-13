# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
# All rights reserved.

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import cv2
import numpy as np
import time

from multiprocessing import shared_memory, Pipe
from utils.allMessages import (IntersectionDetected, SignDetected, SignType, SignImage, LaneDetected, 
                               LeftLine, RightLine, MiddleLine, DisplayWindow, DistanceErrorTop, DistanceErrorBottom)
from utils.helpers import subscribeQueueMessage, readLatestFromPipeAndClear, removePipeElements
from utils.constants import IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_CHANNELS
from Brain.src.templates.threadwithstop import ThreadWithStop


class threadDisplay(ThreadWithStop):

    # ===================================== INIT =========================================
    def __init__(self, rois, event, sharedMemoryImageName, queues, logger, debugger):
        super(threadDisplay, self).__init__()
        
        # init variables
        self.queuesList = queues
        self.logger = logger
        self.debugger = debugger

        # wait for shared memory thread to start
        time.sleep(1)

        # init event
        self.event = event

        # init shared memory
        self.sharedMemoryImage = shared_memory.SharedMemory(name=sharedMemoryImageName) 

        # init rois
        self.rois = rois

        # init messages
        self.initSubscribeMessages()

        self.font = cv2.FONT_HERSHEY_DUPLEX

    # ===================================== RUN ==========================================
    def run(self):
        self.laneReadData = None
        self.laneLeft = None
        self.laneRight = None
        self.laneMiddle = None
        self.laneDisplayWindows = None

        self.signType = None
        self.signImage = None
        self.signDetected = None

        self.intersectionData = None

        self.displayTimerSign = 0
        self.displayTimerIntersection = 0

        self.errorBottom = None
        self.errorTop = None

        while self._running:
            try:
                #event
                self.event.wait()

                # get image from shared memory
                sharedImage = np.ndarray(shape=(IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS), dtype=np.uint8, buffer=self.sharedMemoryImage.buf)
                image = sharedImage.copy()

                self.readInformation()
                self.drawInformation(image)

                # cv2.imshow("image", image)
                # cv2.waitKey(1)

            except Exception as e:
                print(f"An error occurred in {__class__.__name__}: {e}")

    # ===================================== READ PIPES =====================================
    def readInformation(self):
        # recv intersection detection
        self.intersectionData = None
        if self.pipeRecvIntersectionData.poll():
            self.intersectionData = readLatestFromPipeAndClear(self.pipeRecvIntersectionData)
            self.displayTimerIntersection = time.time()

        # recv lane detection
        self.laneReadData = False
        if self.pipeRecvLaneDetected.poll():
            if time.time() - self.displayTimerIntersection > 3:
                self.laneReadData = True
                
            removePipeElements(self.pipeRecvLaneDetected)

            self.laneLeft = readLatestFromPipeAndClear(self.pipeRecvLeftLine, self.laneReadData)
            self.laneRight = readLatestFromPipeAndClear(self.pipeRecvRightLine, self.laneReadData)
            self.laneMiddle = readLatestFromPipeAndClear(self.pipeRecvMiddleLine, self.laneReadData)
            self.laneDisplayWindows = readLatestFromPipeAndClear(self.pipeRecvDisplayWindow, self.laneReadData)

        # recv sign detection
        self.signCoords = None
        if self.pipeRecvSignDetected.poll():
            self.signDetected = True
            removePipeElements(self.pipeRecvSignDetected)

            self.signType = readLatestFromPipeAndClear(self.pipeRecvSignType)
            self.signImage = readLatestFromPipeAndClear(self.pipeRecvSignImage)
            self.displayTimerSign = time.time()

        # recv distance error
        if self.pipeRecvLaneErrorBottom.poll():
            self.errorBottom = readLatestFromPipeAndClear(self.pipeRecvLaneErrorBottom)

        if self.pipeRecvLaneErrorTop.poll():
            self.errorTop = readLatestFromPipeAndClear(self.pipeRecvLaneErrorTop)

    # ===================================== DRAW =====================================
    def drawInformation(self, image):
        # draw lane detection
        if self.laneLeft is not None:
            cv2.polylines(image, [self.laneLeft], False, (255, 0, 0), 2)

        if self.laneRight is not None:
            cv2.polylines(image, [self.laneRight], False, (0, 0, 255), 2)

        if self.laneMiddle is not None:
            cv2.polylines(image, [self.laneMiddle], False, (0, 255, 0), 2)

        if self.laneDisplayWindows is not None:
            if self.laneDisplayWindows["left"] is not None:
                for window in self.laneDisplayWindows["left"]:
                    cv2.rectangle(image, window[0], window[1], (255, 0, 0), 1)

            if self.laneDisplayWindows["right"] is not None:
                for window in self.laneDisplayWindows["right"]:
                    cv2.rectangle(image, window[0], window[1], (0, 0, 255), 1)

        # draw intersection detection
        if self.intersectionData:
            x1, y1, x2, y2 = self.intersectionData["line"]
            midPoint = (x1 + x2) / 2, (y1 + y2) / 2
            
            cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            cv2.putText(image, str(self.intersectionData["id"]), (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
            cv2.line(image, (IMAGE_WIDTH // 2, IMAGE_HEIGHT), (int(midPoint[0]), int(midPoint[1])), (0, 0, 255), 2)
            cv2.circle(image, (int(midPoint[0]), int(midPoint[1])), 5, (0, 255, 0), -1)
            cv2.circle(image, (IMAGE_WIDTH // 2, IMAGE_HEIGHT), 5, (0, 255, 0), -1)

        # draw sign detection
        if self.signDetected:
            offset = 5
            size = (64, 64)
            self.signImage = cv2.resize(self.signImage, size)

            image[offset:offset + size[0], offset:offset + size[1]] = self.signImage
            cv2.rectangle(image, (offset, offset), (offset + size[0], offset + size[1]), (0, 200, 200), 1)

            cv2.putText(image, self.signType, (offset, offset + size[0] + 15), self.font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            if time.time() - self.displayTimerSign > 4:
                self.signDetected = False

        # draw roi
        cv2.line(image, self.rois["Intersection"][0], self.rois["Intersection"][1], (255, 0, 255), 1)
        cv2.line(image, self.rois["Intersection"][1], self.rois["Intersection"][3], (255, 0, 255), 1)
        cv2.line(image, self.rois["Intersection"][3], self.rois["Intersection"][2], (255, 0, 255), 1)
        cv2.line(image, self.rois["Intersection"][2], self.rois["Intersection"][0], (255, 0, 255), 1)

        cv2.line(image, self.rois["Sign"][0], self.rois["Sign"][1], (255, 255, 0), 1)
        cv2.line(image, self.rois["Sign"][1], self.rois["Sign"][3], (255, 255, 0), 1)
        cv2.line(image, self.rois["Sign"][3], self.rois["Sign"][2], (255, 255, 0), 1)
        cv2.line(image, self.rois["Sign"][2], self.rois["Sign"][0], (255, 255, 0), 1)

        offset = 50
        cv2.line(image, (self.rois["Road"][0][0] - offset, self.rois["Road"][0][1]), (self.rois["Road"][1][0] + offset, self.rois["Road"][1][1]), (0, 0, 255), 1)
        cv2.line(image, (self.rois["Road"][1][0] + offset, self.rois["Road"][1][1]), (self.rois["Road"][3][0] + offset, self.rois["Road"][3][1]), (0, 0, 255), 1)
        cv2.line(image, (self.rois["Road"][3][0] + offset, self.rois["Road"][3][1]), (self.rois["Road"][2][0] - offset, self.rois["Road"][2][1]), (0, 0, 255), 1)
        cv2.line(image, (self.rois["Road"][2][0] - offset, self.rois["Road"][2][1]), (self.rois["Road"][0][0] - offset, self.rois["Road"][0][1]), (0, 0, 255), 1)

        cv2.line(image, (IMAGE_WIDTH // 2, 0), (IMAGE_WIDTH // 2, IMAGE_HEIGHT), (255, 255, 255), 1)

        # draw distance error
        if self.errorTop is not None:
            cv2.putText(image, "DET: " + self.errorTop, (0, 105), self.font, 0.6, (0, 0, 255), 1, cv2.LINE_AA)

        if self.errorBottom is not None:
            cv2.putText(image, "DEB: " + self.errorBottom, (0, 125), self.font, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
    
    # ===================================== INIT MESSAGES ==========================================
    def initSubscribeMessages(self):
        # init pipes
        self.pipeRecvLaneDetected, pipeSendLaneDetected = Pipe(duplex=False)
        self.pipeRecvLeftLine, pipeSendLeftLine = Pipe(duplex=False)
        self.pipeRecvRightLine, pipeSendRightLine = Pipe(duplex=False)
        self.pipeRecvMiddleLine, pipeSendMiddleLine = Pipe(duplex=False)
        self.pipeRecvDisplayWindow, pipeSendDisplayWindow = Pipe(duplex=False)

        self.pipeRecvIntersectionData, pipeSendIntersection = Pipe(duplex=False)
        self.pipeRecvSignDetected, pipeSendSignDetected = Pipe(duplex=False)
        self.pipeRecvSignType, pipeSendSignType = Pipe(duplex=False)
        self.pipeRecvSignImage, pipeSendSignImage = Pipe(duplex=False)

        self.pipeRecvLaneErrorBottom, pipeSendLaneErrorBottom = Pipe(duplex=False) 
        self.pipeRecvLaneErrorTop, pipeSendLaneErrorTop = Pipe(duplex=False) 

        # subscribe to messages
        subscribeQueueMessage(self.queuesList, LaneDetected, pipeSendLaneDetected, __class__.__name__)
        subscribeQueueMessage(self.queuesList, LeftLine, pipeSendLeftLine, __class__.__name__)
        subscribeQueueMessage(self.queuesList, RightLine, pipeSendRightLine, __class__.__name__)
        subscribeQueueMessage(self.queuesList, MiddleLine, pipeSendMiddleLine, __class__.__name__)
        subscribeQueueMessage(self.queuesList, DisplayWindow, pipeSendDisplayWindow, __class__.__name__)

        subscribeQueueMessage(self.queuesList, IntersectionDetected, pipeSendIntersection, __class__.__name__)
        subscribeQueueMessage(self.queuesList, SignDetected, pipeSendSignDetected, __class__.__name__)
        subscribeQueueMessage(self.queuesList, SignType, pipeSendSignType, __class__.__name__)
        subscribeQueueMessage(self.queuesList, SignImage, pipeSendSignImage, __class__.__name__)
        subscribeQueueMessage(self.queuesList, IntersectionDetected, subscribeQueueMessage, __class__.__name__)

        subscribeQueueMessage(self.queuesList, DistanceErrorTop, pipeSendLaneErrorTop, __class__.__name__)
        subscribeQueueMessage(self.queuesList, DistanceErrorBottom, pipeSendLaneErrorBottom, __class__.__name__)

    # ==================================== START =========================================
    def start(self):
        super(threadDisplay, self).start()

    # ==================================== STOP ==========================================
    def stop(self):
        super(threadDisplay, self).stop()
