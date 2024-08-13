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

import time

from multiprocessing import Pipe
from planning.threads.states.states import States
from Brain.src.templates.threadwithstop import ThreadWithStop
from utils.helpers import subscribeQueueMessage, readLatestFromPipeAndClear
from utils.allMessages import NoLaneDetected


class threadStateMachine(ThreadWithStop):

    # ===================================== INIT =========================================
    def __init__(self, queues, logger, debugger):
        super(threadStateMachine, self).__init__()
        
        # init variables
        self.queuesList = queues
        self.logger = logger
        self.debugger = debugger

        self.lastState = States.AUTONOMOUS_DRIVE
        self.currentState = States.AUTONOMOUS_DRIVE
        
        self.pipeRecvNoLaneDetected, pipeSendNoLaneDetected = Pipe(duplex=False)
        subscribeQueueMessage(self.queuesList, NoLaneDetected, pipeSendNoLaneDetected, __class__.__name__)

    # ===================================== RUN ==========================================
    def run(self):

        while self._running:
            try:
                noLaneDetected = False
                self.lastState = self.currentState

                # no lane detected
                if self.pipeRecvNoLaneDetected.poll():
                    noLaneDetected = readLatestFromPipeAndClear(self.pipeRecvNoLaneDetected, read=True)

                if noLaneDetected and self.currentState == States.AUTONOMOUS_DRIVE:
                    self.currentState = States.MANUAL_DRIVE
                
                if not noLaneDetected and self.currentState == States.MANUAL_DRIVE:
                    self.currentState = States.AUTONOMOUS_DRIVE

                # print state switch
                if self.currentState != self.lastState:
                    print("Switching from " + self.lastState.name + " to " + self.currentState.name + "...")
                
                time.sleep(1)

            except Exception as e:
                print(f"An error occurred in {__class__.__name__}: {e}")

    # ==================================== START =========================================
    def start(self):
        super(threadStateMachine, self).start()

    # ==================================== STOP ==========================================
    def stop(self):
        super(threadStateMachine, self).stop()
