from Brain.src.hardware.serialhandler.threads.threadWrite import threadWrite as OriginalThreadWrite
from hardware.serialhandler.threads.ExtendedMessageConverter import ExtendedMessageConverter
from multiprocessing import Pipe
from utils.allMessages import DistanceErrorBottom, VisualOdometry

class ExtendedThreadWrite(OriginalThreadWrite):
    def __init__(self, queues, serialCom, logFile, example=False):
        # Create a new Pipe for the additional functionality
        pipeRecvError, pipeSendError = Pipe(duplex=False)
        self.pipeRecvError = pipeRecvError
        self.pipeSendError = pipeSendError
        pipeRecvCamera, pipeSendCamera = Pipe(duplex=False)
        self.pipeRecvCamera = pipeRecvCamera
        self.pipeSendCamera = pipeSendCamera
        pipeRecvVisualOdometry, pipeSendVisualOdometry = Pipe(duplex=False)
        self.pipeRecvVisualOdometry = pipeRecvVisualOdometry
        self.pipeSendVisualOdometry = pipeSendVisualOdometry
        super().__init__(queues, serialCom, logFile, example)
        # Create an instance of ExtendedMessageConverter
        self.messageConverter = ExtendedMessageConverter()
        # Set self.running on true for testing
        self.running = True

    def subscribe(self):
        super().subscribe()
        # Subscribe to the new message type
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": DistanceErrorBottom.Owner.value,
                "msgID": DistanceErrorBottom.msgID.value,
                "To": {"receiver": __class__.__name__, "pipe": self.pipeSendError},
            }
        )
        self.queuesList["Config"].put(
            {
                "Subscribe/Unsubscribe": "subscribe",
                "Owner": VisualOdometry.Owner.value,
                "msgID": VisualOdometry.msgID.value,
                "To": {"receiver": __class__.__name__, "pipe": self.pipeSendVisualOdometry},
            }
        )
    
    def Queue_Sending(self):
        """Callback function for engine running flag."""
        super().Queue_Sending()

    def run(self):
        """In this function we check if we got the enable engine signal. After we got it we will start getting messages from raspberry PI. It will transform them into NUCLEO commands and send them."""
        while self._running:
            try:
                if self.pipeRecvRunningSignal.poll():
                    msg = self.pipeRecvRunningSignal.recv()
                    if msg["value"] == True:
                        self.running = True
                    else:
                        self.running = False
                        command = {"action": "1", "speed": 0.0}
                        command_msg = self.messageConverter.get_command(**command)
                        self.serialCom.write(command_msg.encode("ascii"))
                        self.logFile.write(command_msg)
                        command = {"action": "2", "steerAngle": 0.0}
                        command_msg = self.messageConverter.get_command(**command)
                        self.serialCom.write(command_msg.encode("ascii"))
                        self.logFile.write(command_msg)
                if self.running:
                    if self.pipeRecvBreak.poll():
                        message = self.pipeRecvBreak.recv()
                        command = {"action": "1", "speed": float(message["value"])}
                        command_msg = self.messageConverter.get_command(**command)
                        self.serialCom.write(command_msg.encode("ascii"))
                        self.logFile.write(command_msg)
                    elif self.pipeRecvSpeed.poll():
                        message = self.pipeRecvSpeed.recv()
                        command = {"action": "1", "speed": float(message["value"])}
                        command_msg = self.messageConverter.get_command(**command)
                        self.serialCom.write(command_msg.encode("ascii"))
                        self.logFile.write(command_msg)
                    elif self.pipeRecvSteer.poll():
                        message = self.pipeRecvSteer.recv()
                        command = {"action": "2", "steerAngle": float(message["value"])}
                        command_msg = self.messageConverter.get_command(**command)
                        self.serialCom.write(command_msg.encode("ascii"))
                        self.logFile.write(command_msg)
                    elif self.pipeRecvControl.poll():
                        message = self.pipeRecvControl.recv()
                        command = {
                            "action": "9",
                            "time": float(message["value"]["Time"]),
                            "speed": float(message["value"]["Speed"]),
                            "steer": float(message["value"]["Steer"]),
                        }
                        command_msg = self.messageConverter.get_command(**command)
                        self.serialCom.write(command_msg.encode("ascii"))
                        self.logFile.write(command_msg)
                    elif self.pipeRecvError.poll():
                        message = self.pipeRecvError.recv()
                        command = {"action": "0", "error": float(message["value"])}
                        command_msg = self.messageConverter.get_command(**command)
                        self.serialCom.write(command_msg.encode("ascii"))
                        self.logFile.write(command_msg)
                        #print(f"Comanda catre STM: {command_msg}")
                    elif self.pipeRecvVisualOdometry.poll():
                        message = self.pipeRecvVisualOdometry.recv()
                        command = {"action": "5", "psi": float(message["value"])}
                        command_msg = self.messageConverter.get_command(**command)
                        self.serialCom.write(command_msg.encode("ascii"))
                        self.logFile.write(command_msg)
                        #print(f'Comanda catre stm: {command_msg}')
            except Exception as e:
                print(e)
    # ==================================== START =========================================
    def start(self):
        super().start()

    # ==================================== STOP ==========================================
    def stop(self):
        """This function will close the thread and will stop the car."""
        super().stop()

    # ================================== EXAMPLE =========================================
    def example(self):
        """This function simulte the movement of the car."""
        super().example()

