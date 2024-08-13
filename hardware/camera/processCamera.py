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

from multiprocessing import Pipe

from Brain.src.templates.workerprocess import WorkerProcess
from hardware.camera.threads.threadCamera import threadCamera


class processCamera(WorkerProcess):

    # ====================================== INIT ==========================================
    def __init__(self, queueList, logging, debugging=False):
        pipeRecv, pipeSend = Pipe(duplex=False)
        
        self.pipeRecv = pipeRecv
        self.pipeSend = pipeSend
        
        self.queuesList = queueList
        self.logging = logging
        self.debugging = debugging
        
        super(processCamera, self).__init__(self.queuesList)

    # ===================================== INIT TH ======================================
    def _init_threads(self):
        cameraThread = threadCamera(self.pipeRecv, self.pipeSend, self.queuesList, self.logging, self.debugging)
        self.threads.append(cameraThread)

    # ===================================== RUN ==========================================
    def run(self):
        super(processCamera, self).run()

    # ===================================== STOP ==========================================
    def stop(self):
        for thread in self.threads:
            thread.stop()
            thread.join()

        super(processCamera, self).stop()
