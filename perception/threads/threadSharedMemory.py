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
from utils.allMessages import mainCamera
from utils.helpers import subscribeQueueMessage
from utils.constants import IMAGE_WIDTH, IMAGE_HEIGHT, IMAGE_CHANNELS
from Brain.src.templates.threadwithstop import ThreadWithStop


class threadSharedMemory(ThreadWithStop):

    # ===================================== INIT =========================================
    def __init__(self, events, sharedMemoryImageName, queues, logger, debugger):
        super(threadSharedMemory, self).__init__()

        # init variables
        self.queuesList = queues
        self.logger = logger
        self.debugger = debugger

        # init event
        self.events = events

        # init shared memory 
        self.sharedMemoryImage = shared_memory.SharedMemory(name=sharedMemoryImageName)

        # init pipes
        self.pipeRecvData, pipeSendData = Pipe(duplex=False)

        # subscribe to messages
        subscribeQueueMessage(self.queuesList, mainCamera, pipeSendData, __class__.__name__)

    # ===================================== RUN ==========================================
    def run(self):
        while self._running:
            try:
                if self.pipeRecvData.poll():
                    message = self.pipeRecvData.recv()
                    image = message["value"]

                    if image.shape != (IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS):
                        image = self.resizeWithBlackBars(image)

                    sharedImage = np.ndarray(shape=(IMAGE_HEIGHT, IMAGE_WIDTH, IMAGE_CHANNELS), dtype=np.uint8, buffer=self.sharedMemoryImage.buf)
                    sharedImage[:,:,:] = image

                    for event in self.events.values():
                        event.set()
                        event.clear()
                else:
                    time.sleep(0.01)

            except Exception as e:
                print(f"An error occurred in {__class__.__name__}: {e}")

    # ==================================== RESIZE WITH BLACK BARS ===========================
    def resizeWithBlackBars(self, image):
        originalWidth = image.shape[1]

        widthDifference = IMAGE_WIDTH - originalWidth
        newImage = np.zeros((image.shape[0], IMAGE_WIDTH, image.shape[2]), dtype=np.uint8)

        pasteStart = int(widthDifference / 2)

        newImage[:, pasteStart:pasteStart + originalWidth, :] = image

        return newImage
    
    # ==================================== START =========================================
    def start(self):
        super(threadSharedMemory, self).start()

    # ==================================== STOP ==========================================
    def stop(self):
        super(threadSharedMemory, self).stop()
