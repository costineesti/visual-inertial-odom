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
import threading
import time

from multiprocessing import Pipe
from picamera2 import Picamera2

from hardware.cameraCalibration.poseDetector import PoseDetector
from utils.allMessages import mainCamera, Recording, Record, Config, Rois
from utils.helpers import sendQueueMessage, subscribeQueueMessage, readCameraParamsFromFile
from Brain.src.templates.threadwithstop import ThreadWithStop


class threadCamera(ThreadWithStop):
    # ================================ INIT ===============================================
    def __init__(self, pipeRecv, pipeSend, queuesList, logger, debugger):
        super(threadCamera, self).__init__()

        self.pipeRecvConfig = pipeRecv
        self.pipeSendConfig = pipeSend

        pipeRecvRecord, pipeSendRecord = Pipe(duplex=False)
        self.pipeRecvRecord = pipeRecvRecord

        self.queuesList = queuesList
        self.logger = logger
        self.debugger = debugger

        self.frameRate = 5
        self.recording = False

        subscribeQueueMessage(self.queuesList, Record, pipeSendRecord, __class__.__name__)
        subscribeQueueMessage(self.queuesList, Config, self.pipeSendConfig, __class__.__name__)

        self.videoWriter = ""
        self.initCamera()
        self.queueSending()
        self.configs()

        cameraMatrix, distCoeffs = readCameraParamsFromFile("hardware/cameraCalibration/calibration.npy")
        self.poseDetector = PoseDetector(cameraMatrix, distCoeffs)
        self.sendRois()

    def queueSending(self):
        self.queuesList[Recording.Queue.value].put(
            {
                "Owner": Recording.Owner.value,
                "msgID": Recording.msgID.value,
                "msgType": Recording.msgType.value,
                "msgValue": self.recording,
            }
        )
        threading.Timer(1, self.queueSending).start()

    # ================================ INIT CAMERA ========================================
    def initCamera(self):
        self.camera = Picamera2()

        config = self.camera.create_preview_configuration(
            buffer_count=1,
            queue=False,
            main={"format": "RGB888", "size": (450, 240)}, # 2048, 1080
            lores={"size": (320, 240)},
            encode="lores"
        )

        self.camera.configure(config)
        self.camera.start()

    # ================================ RUN ================================================
    def run(self):
        send = True

        while self._running:
            try:
                if self.pipeRecvRecord.poll():
                    msg = self.pipeRecvRecord.recv()
                    self.recording = msg["value"]

                    if msg["value"] == False:
                        self.videoWriter.release()
                    else:
                        fourcc = cv2.VideoWriter_fourcc(*"MJPG")  # You can choose different codecs, e.g., "MJPG", "XVID", "H264", etc.

                        self.videoWriter = cv2.VideoWriter(
                            "output_video" + str(time.time()) + ".avi",
                            fourcc,
                            self.frameRate,
                            (2048, 1080)
                        )
            except Exception as e:
                print(f"An error occurred in {__class__.__name__}: {e}")

            if self.debugger == True:
                self.logger.warning("getting image")

            if send:
                request = self.camera.capture_array("main")
                request = self.poseDetector.getUndistortedImage(request)

                sendQueueMessage(self.queuesList, mainCamera, request)

            send = not send

    # =============================== START ===============================================
    def start(self):
        super(threadCamera, self).start()

    # =============================== STOP ================================================
    def stop(self):
        if self.recording:
            self.videoWriter.release()
        super(threadCamera, self).stop()

    # =============================== CONFIG ==============================================
    def configs(self):
        while self.pipeRecvConfig.poll():
            message = self.pipeRecvConfig.recv()
            message = message["value"]

            print(message)
            
            self.camera.set_controls(
                {
                    "AeEnable": False,
                    "AwbEnable": False,
                    message["action"]: float(message["value"]),
                }
            )
            
        threading.Timer(1, self.configs).start()

    # ================================ OTHERS ==========================================
    def getRois(self):
        image = self.camera.capture_array("main")

        if self.poseDetector.findAreas(image):
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
        