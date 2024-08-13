from Brain.src.hardware.serialhandler.processSerialHandler import processSerialHandler as OriginalProcessSerialHandler
from hardware.serialhandler.threads.ExtendedThreadWrite import ExtendedThreadWrite
from hardware.serialhandler.threads.ExtendedThreadRead import ExtendedThreadRead

class ExtendedProcessSerialHandler(OriginalProcessSerialHandler):
    def __init__(self, queueList, logging):
        super().__init__(queueList, logging)

    def _init_threads(self):
        """Initializes the read and the write thread."""
        readTh = ExtendedThreadRead(self.serialCom, self.historyFile, self.queuesList)
        self.threads.append(readTh)
        writeTh = ExtendedThreadWrite(
            self.queuesList, self.serialCom, self.historyFile, self.example
        )
        self.threads.append(writeTh)
    # ===================================== STOP ==========================================
    def stop(self):
        super().stop()
    
    # ===================================== RUN ==========================================
    def run(self):
        super().run()

    # =================================== EXAMPLE =========================================
#             ++    THIS WILL RUN ONLY IF YOU RUN THE CODE FROM HERE  ++
#                  in terminal:    python3 processSerialHandler.py
if __name__ == "__main__":
    from multiprocessing import Queue, Pipe
    import logging
    import time

    allProcesses = list()
    debugg = False
    # We have a list of multiprocessing.Queue() which individualy represent a priority for processes.
    queueList = {
        "Critical": Queue(),
        "Warning": Queue(),
        "General": Queue(),
        "Config": Queue(),
    }
    logger = logging.getLogger()
    pipeRecv, pipeSend = Pipe(duplex=False)
    process = ExtendedProcessSerialHandler(queueList, logger, debugg, True)
    process.daemon = True
    process.start()
    time.sleep(4)  # modify the value to increase/decrease the time of the example
    process.stop()
