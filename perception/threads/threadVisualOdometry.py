import numpy as np
import matplotlib.pyplot as plt
import cv2
from utils.allMessages import (LaneDetected, MiddleLine, VisualOdometry)
from utils.helpers import subscribeQueueMessage, removePipeElements, sendQueueMessage
from Brain.src.templates.threadwithstop import ThreadWithStop
from multiprocessing import Pipe
from Brain.src.utils.messages.allMessages import SpeedMotor


def to_degrees(radians):
    return radians * 180.0 / np.pi

def get_orientation(ref_line, middleLane):
    transf_matrix = cv2.estimateAffinePartial2D(ref_line, middleLane, confidence=0.99)[0]
    scale = np.sqrt(transf_matrix[0,0]*transf_matrix[0,0] + transf_matrix[0,1]*transf_matrix[0,1])
    # print(f'scale: {scale}')
    transf_matrix /= scale
    angle_rad = np.arctan2(transf_matrix[0,1], transf_matrix[0,0])
    return angle_rad


def UpdatePlot(self, line, ax, orientation_text, fig):
    x_data = np.append(line.get_xdata(), self.x)
    y_data = np.append(line.get_ydata(), self.y)
    line.set_data(x_data, y_data)
    ax.set_xlim(min(x_data) - 50, max(x_data) + 50)  # Adjust x-axis limits dynamically
    ax.set_ylim(min(y_data) - 50, max(y_data) + 50)  # Adjust y-axis limits dynamically
    # Update the value of self.orientation on the plot
    orientation_text.set_text("Orientation: {:.2f}".format(self.orientation_deg))
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(0.01)

def adjust_orientation(self, middleLane, angle_rad):
    if middleLane[-1,0] < middleLane[0,0] and abs(middleLane[-1,0] - middleLane[0,0]) > LANE_POSITION_THRESHOLD:
        self.orientation += abs(angle_rad) # right turn
    elif middleLane[-1,0] > middleLane[0,0] and abs(middleLane[-1,0] - middleLane[0,0]) > LANE_POSITION_THRESHOLD:
        self.orientation -= abs(angle_rad) # left turn
    elif abs(middleLane[-1,0] - middleLane[0,0]) < LANE_POSITION_THRESHOLD:
        self.orientation += angle_rad # straight line
    return self.orientation


LANE_POSITION_THRESHOLD = 12.0

class threadVisualOdometry(ThreadWithStop):

    # ===================================== INIT =========================================
    def __init__(self, event, queues, logger, debugger):
        super(threadVisualOdometry, self).__init__()

        # init variables
        self.queuesList = queues
        self.logger = logger
        self.debugger = debugger

        # init event
        self.event = event

        # init messages
        self.initSubscribeMessages()
        
        # Class variables
        self.orientation = 0.0
        self.orientation_deg = 0.0
        self.crosswalkStepCounter = 0
        self.x = 0.0
        self.y = 0.0
        self.plot = True
        self.setSpeed = True
        self.signCounter = 0

    # ===================================== RUN ==========================================
    def run(self):
        self.laneMiddle = None
        self.laneDetected = None
        self.signType = None

        while self._running:
            try:
                self.event.wait()
                self.readInformation()
                if self.laneDetected == True:
                    
                    if self.setSpeed == True:
                        sendQueueMessage(self.queuesList, SpeedMotor, 15.0)
                        print("Set motor speed to 10")
                        self.setSpeed = False
                    
                    if self.plot == True and self.laneMiddle is not None:
                        ref_line = self.laneMiddle
                        # plt.ion()  # Turn on interactive mode
                        # fig, ax = plt.subplots()  # Create a figure and an axes
                        # line, = ax.plot([], [])  # Create a line object for the plot (initially empty)  
                        # orientation_text = ax.text(0.05, 0.95, "", transform=ax.transAxes, fontsize=12, verticalalignment='top')  # Create text object for orientation
                        self.plot = False

                    if self.laneMiddle is not None and self.setSpeed == False:
                        middleLane = self.laneMiddle

                        if len(ref_line) > len(middleLane):
                            ref_line = ref_line[:len(middleLane)]
                        else:
                            middleLane = middleLane[:len(ref_line)]

                        angle_rad = get_orientation(ref_line, middleLane)
                        # filter noisy data
                        if abs(to_degrees(angle_rad)) > 6:
                            angle_rad = 0
                        self.orientation = adjust_orientation(self, middleLane, angle_rad)
                        print(f'diferenta de unghi: {to_degrees(angle_rad)} la curbura: {middleLane[-1,0] - middleLane[0,0]} si cu orientarea: {self.orientation_deg}')
                        # Aici verific daca am dat comanda la motor si atunci ii dau sa ploteze pe x si y. Momentan merge cum merge.
                        self.orientation %= 2*np.pi
                        self.orientation_deg = to_degrees(self.orientation)
                        self.orientation_deg %= 360
                        # Send update state to NUCLEO
                        if self.setSpeed == False: 
                            sendQueueMessage(self.queuesList, VisualOdometry, self.orientation_deg)
                        # UpdatePlot(self, line, ax, orientation_text, fig)
                        ref_line = self.laneMiddle

            except Exception as e:
                print(e)

    # ===================================== READ PIPES =====================================
    def readInformation(self):
        # recv lane detection -> middle lane.
        if self.pipeRecvLaneDetected.poll():
            self.laneDetected = True
            removePipeElements(self.pipeRecvLaneDetected)
        
        if self.pipeRecvMiddleLine.poll():
            self.laneMiddle = self.pipeRecvMiddleLine.recv()['value']
            removePipeElements(self.pipeRecvMiddleLine)

    # ===================================== INIT MESSAGES ==========================================
    def initSubscribeMessages(self):
        # init pipes
        self.pipeRecvLaneDetected, pipeSendLaneDetected = Pipe(duplex=False)
        self.pipeRecvMiddleLine, pipeSendMiddleLine = Pipe(duplex=False)
        # subscribe to messages
        subscribeQueueMessage(self.queuesList, LaneDetected, pipeSendLaneDetected, __class__.__name__)
        subscribeQueueMessage(self.queuesList, MiddleLine, pipeSendMiddleLine, __class__.__name__)
    # ==================================== START =========================================
    def start(self):
        super(threadVisualOdometry, self).start()

    # ==================================== STOP ==========================================
    def stop(self):
        super(threadVisualOdometry, self).stop()
