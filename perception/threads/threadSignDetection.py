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

from multiprocessing import shared_memory
from perception.threads.sign_detection.signDetection import SignDetection
from perception.threads.sign_detection.reverseCoordinates import ReverseCoordinates
from perception.threads.utils.warpPerspective import WarpPerspective
from utils.allMessages import SignDetected, SignType, SignCoordinations, SignImage
from utils.helpers import sendQueueMessage
from utils.constants import IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_CHANNELS
from Brain.src.templates.threadwithstop import ThreadWithStop


class threadSignDetection(ThreadWithStop):

    # ===================================== INIT =========================================
    def __init__(self, roi, event, sharedMemoryImageName, queues, logger, debugger):
        super(threadSignDetection, self).__init__()
        
        # init variables
        self.queuesList = queues
        self.logger = logger
        self.debugger = debugger

        # wait for shared memory thread to start
        time.sleep(1)

        # init event
        self.event = event

        # region of interest
        offset = 0
        regionOfInterest = {
            "src": np.float32([(roi[0][0] - offset, roi[0][1]), (roi[1][0] + offset, roi[1][1]), (roi[2][0] - offset, roi[2][1]), (roi[3][0] + offset, roi[3][1])]),
            "dst": np.float32([(1, 1), (320, 1), (1, 240), (320, 240)])
        }

        # image resize
        self.resize = (32, 32)

        # init shared memory
        self.sharedMemoryImage = shared_memory.SharedMemory(name=sharedMemoryImageName)

        # init utils
        self.warpPerspective = WarpPerspective(regionOfInterest, IMAGE_WIDTH, IMAGE_HEIGHT)
        self.signDetection = SignDetection()

    # ===================================== RUN ==========================================
    def run(self):

        while self._running:
            try:
                # event
                self.event.wait()

                # get image from shared memory
                image = np.ndarray(shape=(IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS), dtype=np.uint8, buffer=self.sharedMemoryImage.buf)

                # sign detection
                warpPerspectiveImage = self.warpPerspective.transform(image)
                signData = self.signDetection.detect(warpPerspectiveImage, saveData=False)
                
                if signData is None:
                    continue

                x1 = signData["coords"][0]
                y1 = signData["coords"][1]
                x2 = signData["coords"][2]
                y2 = signData["coords"][3]

                # reverse coords
                signData["coords"] = ReverseCoordinates().reverseCoordinates([[x1, y1], [x2, y2]], self.warpPerspective.inverseTransformMatrix)

                # resize image to 32x32
                signData["image"] = cv2.resize(signData["image"], self.resize)

                # send data to draw thread and intersection thread
                sendQueueMessage(self.queuesList, SignType, signData["type"])
                sendQueueMessage(self.queuesList, SignCoordinations, signData["coords"])
                sendQueueMessage(self.queuesList, SignImage, signData["image"])
                sendQueueMessage(self.queuesList, SignDetected, True)
                
            except Exception as e:
                print(f"An error occurred in {__class__.__name__}: {e}")

    # ==================================== START =========================================
    def start(self):
        super(threadSignDetection, self).start()

    # ==================================== STOP ==========================================
    def stop(self):
        super(threadSignDetection, self).stop()
