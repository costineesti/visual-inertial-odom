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
import time

from hardware.cameraCalibration.poseDetector import PoseDetector
from utils.allMessages import mainCamera, Rois
from utils.helpers import readCameraParamsFromFile
from Brain.src.templates.threadwithstop import ThreadWithStop


class threadSimulatedCamera(ThreadWithStop):

    # ================================ INIT ===============================================
    def __init__(self, pipeRecv, pipeSend, queuesList, logger, debugger):
        super(threadSimulatedCamera, self).__init__()

        videoPath = "hardware/simulatedCamera/video.mp4"
        self.capture = cv2.VideoCapture(videoPath)

        self.pipeRecvConfig = pipeRecv
        self.pipeSendConfig = pipeSend

        self.frames = 30

        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger

        cameraMatrix, distCoeffs = readCameraParamsFromFile("hardware/cameraCalibration/calibration.npy")
        self.poseDetector = PoseDetector(cameraMatrix, distCoeffs)
        self.sendRois()

    # ================================ RUN ================================================
    def run(self):
        try:
            while self._running:
                if self.debugger == True:
                    self.logger.warning("getting image")

                ret, image = self.capture.read()

                if not ret:
                    self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, image = self.capture.read()

                image = self.poseDetector.getUndistortedImage(image)

                message = {
                    "Owner": mainCamera.Owner.value,
                    "msgID": mainCamera.msgID.value,
                    "msgType": mainCamera.msgType.value,
                    "msgValue": image
                }

                self.queuesList[mainCamera.Queue.value].put(message)

                time.sleep(1 / self.frames)
                
        except Exception as e:
            print(f"An error occurred in {__class__.__name__}: {e}")

    # =============================== OTHERS ================================================    
    def getRois(self):
        ret, image = self.capture.read()

        if ret and self.poseDetector.findAreas(image):
            return self.poseDetector.get2dCoords()
        else:
            return self.poseDetector.getDefault2dCoords()
        
    def sendRois(self):
        rois = self.getRois()

        message = { 
            "Owner": Rois.Owner.value,
            "msgID": Rois.msgID.value,
            "msgType": Rois.msgType.value,
            "msgValue": rois
        }

        self.queuesList[Rois.Queue.value].put(message)

    # =============================== START ===============================================
    def start(self):
        super(threadSimulatedCamera, self).start()
