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

import numpy as np
import time
import cv2
from multiprocessing import shared_memory, Pipe
from perception.threads.lane_detection.imagePreprocess import ImagePreprocess
from perception.threads.lane_detection.laneDetect import LaneDetect
from perception.threads.lane_detection.distanceError import DistanceError
from perception.threads.utils.warpPerspective import WarpPerspective
from utils.allMessages import LaneDetected, NoLaneDetected, LeftLine, RightLine, MiddleLine, DisplayWindow, DistanceErrorBottom, DistanceErrorTop, IntersectionDetected
from utils.helpers import sendQueueMessage, removePipeElements, subscribeQueueMessage
from utils.constants import IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_CHANNELS
from Brain.src.templates.threadwithstop import ThreadWithStop

class threadLaneDetection(ThreadWithStop):
    
    # ===================================== INIT =========================================
    def __init__(self, roi, event, sharedMemoryImageName, queues, logger, debugger):
        super(threadLaneDetection, self).__init__()

        # init variables
        self.queuesList = queues
        self.logger = logger
        self.debugger = debugger

        # wait for shared memory thread to start
        time.sleep(1)

        # init event
        self.event = event

        # region of interest
        offset = 50
        regionOfInterest = {
            "src": np.float32([(roi[0][0] - offset, roi[0][1]), (roi[1][0] + offset, roi[1][1]), (roi[2][0] - offset, roi[2][1]), (roi[3][0] + offset, roi[3][1])]),
            "dst": np.float32([(1, 1), (450, 1), (1, 240), (450, 240)])
        }

        # init shared memory
        self.sharedMemoryImage = shared_memory.SharedMemory(name=sharedMemoryImageName)

        # init utils
        self.warpPerspective = WarpPerspective(regionOfInterest, IMAGE_WIDTH, IMAGE_HEIGHT)
        self.laneDetect = LaneDetect(IMAGE_WIDTH, IMAGE_HEIGHT)
        self.distanceError = DistanceError()
        self.rightTurn = False
        self.leftTurn = False
        self.intersection = False
        pipeRecvIntersection, pipeSendIntersection = Pipe(duplex=False)
        self.pipeRecvIntersection = pipeRecvIntersection
        self.pipeSendIntersection = pipeSendIntersection
        subscribeQueueMessage(self.queuesList, IntersectionDetected, self.pipeSendIntersection, __class__.__name__)
        self.counter = 0

    # ===================================== RUN ==========================================
    def run(self):

        while self._running:
            try:
                # event
                self.event.wait()
                # self.readInformation()
                # get image from shared memory
                image = np.ndarray(shape=(IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS), dtype=np.uint8, buffer=self.sharedMemoryImage.buf)

                # lane detection
                preprocessedImage = ImagePreprocess().preprocess(image)
                warpPerspectiveImage = self.warpPerspective.transform(preprocessedImage)
                cv2.imshow("image", warpPerspectiveImage)
                cv2.waitKey(1)
                laneData = self.laneDetect.detect(warpPerspectiveImage)              

                # send message and continue if no lane data found
                if laneData["lines"]["left"] is None and laneData["lines"]["right"] is None:
                    sendQueueMessage(self.queuesList, NoLaneDetected, True)
                    continue

                # for side in ["left", "right"]:
                #     if laneData["lines"][side] is not None:
                #         if laneData["lines"][side][0][0] > laneData["lines"][side][-1][0]:
                #             self.rightTurn = True
                #         else:
                #             self.leftTurn = True
                #         break

                # # get lane distance error 
                # if not self.rightTurn and not self.leftTurn:
                #     errorTop, errorBottom = self.distanceError.getError(laneData)
                # elif self.rightTurn:
                #     errorTop = errorBottom = '-20.0'
                #     self.rightTurn = False
                # else:
                #     errorTop = errorBottom = '20.0'
                #     self.leftTurn = False
                # if self.intersection:
                #     while self.counter < 50:
                #         errorTop = errorBottom = '-20.0'
                #         self.counter += 1
                #     if self.counter >= 50:
                #         self.counter = 0
                #         self.intersection = False
                # else:
                errorTop, errorBottom = self.distanceError.getError(laneData)
                #if abs(float(errorTop)) - abs(float(errorBottom)) > 10:
                #    errorBottom = errorTop
                #print(f'Bottom: {errorBottom}, Top: {errorTop}')
                 
                # send data
                if errorTop is not None and errorBottom is not None:
                    sendQueueMessage(self.queuesList, DistanceErrorTop, errorTop)
                    sendQueueMessage(self.queuesList, DistanceErrorBottom, errorBottom)
                    
                sendQueueMessage(self.queuesList, LeftLine, laneData["lines"]["left"])
                sendQueueMessage(self.queuesList, RightLine, laneData["lines"]["right"])
                sendQueueMessage(self.queuesList, MiddleLine, laneData["lines"]["middle"])
                sendQueueMessage(self.queuesList, DisplayWindow, laneData["windows"])
                sendQueueMessage(self.queuesList, LaneDetected, True)

            except Exception as e:
                print(f"An error occurred in {__class__.__name__}: {e}")

    # ===================================== READ PIPES =====================================
    def readInformation(self):
        if self.pipeRecvIntersection.poll():
            print('sign')
            self.intersection = True
            removePipeElements(self.pipeRecvIntersection)
    
    # ==================================== START =========================================
    def start(self):
        super(threadLaneDetection, self).start()

    # ==================================== STOP ==========================================
    def stop(self):
        super(threadLaneDetection, self).stop()
