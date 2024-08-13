# Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC orginazers
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
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# ===================================== GENERAL IMPORTS ==================================
import sys
import logging

from multiprocessing import Queue, Event
from gui.gui import GUI

sys.path.append(".")
sys.path.append("Brain")

# ===================================== PROCESS IMPORTS ==================================
from Brain.src.gateway.processGateway import processGateway
#from Brain.src.hardware.serialhandler.processSerialHandler import processSerialHandler
from hardware.serialhandler.ExtendedProcessSerialHandler import ExtendedProcessSerialHandler
from perception.processCameraDetection import processCameraDetection

# ================================== DELETE CSV FILES ==================================
import os
files_to_delete = ['data_camera.csv','data_imu.csv','data_predict.csv','data_update.csv']
for file in files_to_delete:
    if os.path.exists(file):
        os.remove(file)
# ======================================== SETTING UP ====================================
allProcesses = list()

queueList = {
    "Critical": Queue(),
    "Warning": Queue(),
    "General": Queue(),
    "Config": Queue() 
}

logging = logging.getLogger()

SerialHandler = True

# ===================================== GUI ====================================
gui = GUI(fullscreen=False)

# ===================================== CAMERA CALIBRATION ====================================
if gui.startCalibrate:
    from hardware.cameraCalibration import calibrateCamera

    calibrateCamera.main(gui.takePhotos, gui.keepOldPhotos)
    exit()

# ===================================== SETUP PROCESSES ====================================
# Initializing gateway
processGateway = processGateway(queueList, logging)
allProcesses.append(processGateway)

# Initializing serial connection NUCLEO - > PI
if SerialHandler:
   # processSerialHandler = processSerialHandler(queueList, logging)
    processSerialHandler = ExtendedProcessSerialHandler(queueList, logging) # Added my extendedProcessSerialHandler.
    allProcesses.append(processSerialHandler)

    # CHECK if parameters kp ki and kd were passed by the user. If so, then send them to NUCLEO
    if len(sys.argv) == 4:
        kP = sys.argv[1]
        kI = sys.argv[2]
        kD = sys.argv[3]
        print(f'kP: {kP}, kI: {kI}, kD: {kD}')
        command = {
            "action": "4",
            "proportional": float(kP),
            "integral": float(kI),
            "derivative": float(kD),
        }
        # Create an instance of ExtendedMessageConverter
        from hardware.serialhandler.threads.ExtendedMessageConverter import ExtendedMessageConverter
        messageConverter = ExtendedMessageConverter()
        # create an instance of logFile
        command_msg = messageConverter.get_command(**command)
        print(f'command to change PID params: {command_msg}')
        processSerialHandler.serialCom.write(command_msg.encode("ascii"))
        processSerialHandler.historyFile.write(command_msg)
        
        from Brain.src.utils.messages.allMessages import SpeedMotor
        command = {"action":"1", "speed":0.0}
        command_msg = messageConverter.get_command(**command)
        print('COMMAND SENT TO MOTOR 0')
        processSerialHandler.serialCom.write(command_msg.encode("ascii"))
        processSerialHandler.historyFile.write(command_msg)
    
# Initializing camera
if gui.startVideo:
    from hardware.simulatedCamera.processSimulatedCamera import processSimulatedCamera
    
    processSimulatedCamera = processSimulatedCamera(queueList, logging)
    allProcesses.append(processSimulatedCamera)

if gui.startLive:
    from hardware.camera.processCamera import processCamera
    
    processCamera = processCamera(queueList, logging)
    allProcesses.append(processCamera)

if gui.startVideo or gui.startLive:
    processCameraDetection = processCameraDetection(queueList, logging)
    allProcesses.append(processCameraDetection)


# ===================================== START PROCESSES ====================================
for process in allProcesses:
    process.daemon = True
    process.start()

# ===================================== STAYING ALIVE ====================================
blocker = Event()

try:
    blocker.wait()

except KeyboardInterrupt:
    print("\nCatching a KeyboardInterruption exception! Shutdown all processes.\n")

    for proc in allProcesses:
        print("Process stopped", proc)
        proc.stop()
        proc.join()
