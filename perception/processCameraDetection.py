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
#    contributors may be used to endorse or promote products derived Owner
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
import random
import string
import time

from multiprocessing import Event, shared_memory, Pipe

from Brain.src.templates.workerprocess import WorkerProcess
from perception.threads.threadIntersectionDetection import threadIntersectionDetection
from perception.threads.threadLaneDetection import threadLaneDetection
from perception.threads.threadVisualOdometry import threadVisualOdometry
from perception.threads.threadDisplay import threadDisplay
from perception.threads.threadSharedMemory import threadSharedMemory
from perception.threads.threadSignDetection import threadSignDetection
from utils.allMessages import Rois
from utils.helpers import subscribeQueueMessage, unsubscribeQueueMessage, readLatestFromPipeAndClear
from utils.constants import IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_CHANNELS


class processCameraDetection(WorkerProcess):

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logger, debugging=False):
        self.queuesList = queueList
        self.logger = logger
        self.debugging = debugging

        # wait for other processes to start
        time.sleep(1)

        # init event 
        self.events = {"intersection": Event(), "lane": Event(), "sign": Event(), "display": Event(), "visual": Event()}

        # init shared memory 
        bufferSize = IMAGE_WIDTH * IMAGE_HEIGHT * IMAGE_CHANNELS * np.uint8().itemsize

        self.sharedMemoryImageName = self.generateRandomWord(10)
        self.sharedMemoryImage = shared_memory.SharedMemory(name=self.sharedMemoryImageName, create=True, size=bufferSize)
        print("CameraDetection shared memory name: ", self.sharedMemoryImageName)

        super(processCameraDetection, self).__init__(queueList)
        
    # ===================================== RUN ===========================================
    def run(self):
        super(processCameraDetection, self).run()

    # ===================================== STOP ==========================================
    def stop(self):
        for thread in self.threads:
            thread.stop()
            thread.join()
        
        self.sharedMemoryImage.close()
        self.sharedMemoryImage.unlink()
        print("CameraDetection shared memory closed")

        super(processCameraDetection, self).stop()

    # ===================================== INIT TH ==========================================
    def _init_threads(self):
        rois = self.getRois(message=Rois)

        sharedMemoryThread = threadSharedMemory(self.events, self.sharedMemoryImageName, self.queuesList, self.logger, self.debugging)
        displayThread = threadDisplay(rois, self.events["display"], self.sharedMemoryImageName, self.queuesList, self.logger, self.debugging)
        intersectionDetectionThread = threadIntersectionDetection(rois["Intersection"], self.events["intersection"], self.sharedMemoryImageName, self.queuesList, self.logger, self.debugging)
        laneDetectionThread = threadLaneDetection(rois["Road"], self.events["lane"], self.sharedMemoryImageName, self.queuesList, self.logger, self.debugging)
        signDetectionThread = threadSignDetection(rois["Sign"], self.events["sign"], self.sharedMemoryImageName, self.queuesList, self.logger, self.debugging)
        visualOdometryThread = threadVisualOdometry(self.events["visual"], self.queuesList, self.logger, self.debugging)

        self.threads.extend([sharedMemoryThread, intersectionDetectionThread, laneDetectionThread, visualOdometryThread, signDetectionThread, displayThread])        

    # ===================================== GET ROIS ==========================================
    def getRois(self, message):
        pipeRecv, pipeSend = Pipe(duplex=False)
        subscribeQueueMessage(self.queuesList, message, pipeSend, __class__.__name__)

        while not pipeRecv.poll():
            time.sleep(0.01)

        roi = readLatestFromPipeAndClear(pipeRecv)
        unsubscribeQueueMessage(self.queuesList, message, pipeRecv, pipeSend, __class__.__name__)

        return roi
    
    # ===================================== GENERATE RANDOM WORD ==========================================
    def generateRandomWord(self, length):
        characters = string.ascii_letters + string.digits
        return "".join(random.choice(characters) for _ in range(length))
