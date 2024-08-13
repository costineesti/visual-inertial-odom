import numpy as np


def sendQueueMessage(queuesList, message, value):
    """
    Puts a value into a specific message queue.
    """
    queuesList[message.Queue.value].put(
    {
        "Owner": message.Owner.value,
        "msgID": message.msgID.value,
        "msgType": message.msgType.value,
        "msgValue": value
    }
)
    

def subscribeQueueMessage(queuesList, message, pipe, receiver):
    """
    Subscribes to a message
    """
    queuesList["Config"].put(
        {
            "Subscribe/Unsubscribe": "subscribe",
            "Owner": message.Owner.value,
            "msgID": message.msgID.value,
            "To": {"receiver": receiver, "pipe": pipe}
        }
    )


def unsubscribeQueueMessage(queuesList, message, pipeRecv, pipeSend, receiver):
    """
    Unsubscribes from a message
    """
    queuesList["Config"].put(
        {
            "Subscribe/Unsubscribe": "unsubscribe",
            "Owner": message.Owner.value,
            "msgID": message.msgID.value,
            "To": {"receiver": receiver}
        }
    )

    pipeRecv.close()
    pipeSend.close()


def removePipeElements(pipe):
    """
    Removes elements from a pipe
    """
    while pipe.poll():
        pipe.recv()
    

def readLatestFromPipeAndClear(pipe, read=True):
    """
    Reads the latest element from a pipe and removes the remaining elements

    Example: 
        readPipe(pipe)  # Output: pipe value
    """
    recv = None
    if read:
        recv = pipe.recv()

    removePipeElements(pipe)
    return recv['value'] if recv is not None else None


def findClosestValueForKey(dictionary, inputKey):
    """
    Finds the closest key in the dict to the input key and returns the corresponding value.

    Example:
        dictionary = {1: 'a', 3: 'b', 5: 'c', 7: 'd'}
        inputKey = 4
        print(findClosestValueForKey(dictionary, inputKey))  # Output: 'b'
    """
    closest = min(dictionary.keys(), key=lambda x: abs(x - inputKey))
    return dictionary[closest]


def findSublist(searchList, valueToFind):
    """
    Finds a sublist containing a specific value within a list of sublists.

    Example: 
        list = [[10, 20], [30, 40], [50, 60]]
        valueToFind = 10  # Output: [10, 20]
    """
    for sublist in searchList:
        if valueToFind in sublist:
            return sublist
        
        
def cropImage(image, coord1, coord2):
    """
    Crop a region from the given image.

    Example:
        image = cv2.imread("input.jpg")
        croppedImage = cropImage(image, [10, 10], [50, 50])
    """
    return image[coord1[1]:coord2[1], coord1[0]:coord2[0]]


def readCameraParamsFromFile(filename):
    cameraParams = np.load(filename, allow_pickle="TRUE").item()
    cameraMatrix = cameraParams["camera_matrix"]
    dist = cameraParams["dist_coeff"]
    return cameraMatrix, dist