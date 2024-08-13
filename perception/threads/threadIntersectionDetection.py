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

from multiprocessing import Pipe, shared_memory
from perception.threads.intersection_detection.intersectionDetection import IntersectionDetection
from utils.allMessages import IntersectionDetected, SignType
from utils.helpers import subscribeQueueMessage, sendQueueMessage, readLatestFromPipeAndClear
from utils.constants import IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_CHANNELS
from Brain.src.templates.threadwithstop import ThreadWithStop


class threadIntersectionDetection(ThreadWithStop):

    # ===================================== INIT =========================================
    def __init__(self, roi, event, sharedMemoryImageName, queues, logger, debugger):
        super(threadIntersectionDetection, self).__init__()

        # init variables
        self.queuesList = queues
        self.logger = logger
        self.debugger = debugger

        # wait for shared memory thread to start
        time.sleep(1)

        # init event
        self.event = event

        # region of interest
        self.regionOfInterest = np.int32([roi[0][0], roi[0][1], roi[3][0], roi[3][1]])

        # init utils
        self.intersectionDetection = IntersectionDetection((roi[0][0], roi[0][1]))

        # init shared memory
        self.sharedMemoryImage = shared_memory.SharedMemory(name=sharedMemoryImageName)

        # init pipes
        self.pipeRecvSign, pipeSendSign = Pipe(duplex=False)

        # subscribe to messages
        subscribeQueueMessage(self.queuesList, SignType, pipeSendSign, __class__.__name__)

    # ===================================== RUN ==========================================
    def run(self):
        sign = None

        while self._running:
            try:
                # event
                self.event.wait()

                # get image from shared memory
                image = np.ndarray(shape=(IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS), dtype=np.uint8, buffer=self.sharedMemoryImage.buf)

                # intersection detection
                roiImage = self.getRoiImage(image)

                if self.pipeRecvSign.poll():
                    sign = readLatestFromPipeAndClear(self.pipeRecvSign)
                    timer = time.time()

                if sign is not None:
                    if time.time() - timer > 3:
                        sign = None

                detection = self.intersectionDetection.detect(roiImage, sign)

                if detection:
                    sendQueueMessage(self.queuesList, IntersectionDetected, detection)

            except Exception as e:
                print(f"An error occurred in {__class__.__name__}: {e}")

    # ===================================== REGION OF INTEREST ==========================================
    def getRoiImage(self, image):
        x1 = self.regionOfInterest[0]
        y1 = self.regionOfInterest[1]
        x2 = self.regionOfInterest[2]
        y2 = self.regionOfInterest[3]

        return image[y1:y2, x1:x2]

    # ==================================== START =========================================
    def start(self):
        super(threadIntersectionDetection, self).start()

    # ==================================== STOP ==========================================
    def stop(self):
        super(threadIntersectionDetection, self).stop()
