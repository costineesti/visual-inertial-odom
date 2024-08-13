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
from Brain.src.templates.threadwithstop import ThreadWithStop
from utils.allMessages import (
    ImuData,
)
from Brain.src.hardware.serialhandler.threads.filehandler import FileHandler

class ExtendedThreadRead(ThreadWithStop):
    """This thread read the data that NUCLEO send to Raspberry PI.\n

    Args:
        f_serialCon (serial.Serial): Serial connection between the two boards.
        f_logFile (FileHandler): The path to the history file where you can find the logs from the connection.
        queueList (dictionar of multiprocessing.queues.Queue): Dictionar of queues where the ID is the type of messages.
    """

    # ===================================== INIT =========================================
    def __init__(self, f_serialCon, f_logFile, queueList):
        super(ExtendedThreadRead, self).__init__()
        self.serialCon = f_serialCon
        self.logFile = f_logFile
        self.buff = ""
        self.isResponse = False
        self.queuesList = queueList
        self.acumulator = 0
        camera = "data_camera.txt"
        imu = "data_imu.txt"
        predict = "data_predict.txt"
        update = "data_update.txt"
        self.data_camera = FileHandler(camera)
        self.data_imu = FileHandler(imu)
        self.data_predict = FileHandler(predict)
        self.data_update = FileHandler(update)

    # ====================================== RUN ==========================================
    def run(self):
        while self._running:
            read_chr = self.serialCon.read()
            try:
                read_chr = read_chr.decode("ascii")
                if read_chr == "@":
                    self.isResponse = True
                    if len(self.buff) != 0:
                        self.sendqueue(self.buff)
                    self.buff = ""
                elif read_chr == "\r":
                    self.isResponse = False
                    if len(self.buff) != 0:
                        self.sendqueue(self.buff)
                    self.buff = ""
                if self.isResponse:
                    self.buff += read_chr
            except UnicodeDecodeError:
                pass

    # ==================================== SENDING =======================================

    def sendqueue(self, buff):
        """This function select which type of message we receive from NUCLEO and send the data further."""
        print(buff)
        if buff[1] == '5': # CAMERA
            buff = buff[3:-2]
            #print(buff)
            splitedBuffer = buff.split(";")
            if len(splitedBuffer) == 3:
                line = splitedBuffer[0]+','+splitedBuffer[1]+','+splitedBuffer[2]
                self.data_camera.write(line)

        elif buff[1] == '6': # IMU
            buff = buff[3:-2]
            #print(buff)
            splitedBuffer = buff.split(";")
            if len(splitedBuffer) == 3:
                line = splitedBuffer[0]+','+splitedBuffer[1]+','+splitedBuffer[2]
                self.data_imu.write(line)
        
        elif buff[1] == '7': # PREDICT
            buff = buff[3:-2]
            splitedBuffer = buff.split(";")
            if len(splitedBuffer) == 3:
                line = splitedBuffer[0]+','+splitedBuffer[1]+','+splitedBuffer[2]
                self.data_predict.write(line)

        elif buff[1] == '8': # update
            buff = buff[3:-2]
            #print(buff)
            splitedBuffer = buff.split(";")
            if len(splitedBuffer) == 3:
                line = splitedBuffer[0]+','+splitedBuffer[1]+','+splitedBuffer[2]
                self.data_update.write(line)
