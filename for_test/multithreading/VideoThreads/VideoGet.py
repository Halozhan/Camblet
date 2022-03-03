from threading import Thread
import cv2

class VideoGet:
    """
    Class that continuously gets frames from a VideoCapture object
    with a dedicated thread.
    """

    def __init__(self, src=0):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, 852)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, 1480)
        self.stream.set(cv2.CAP_PROP_FPS, 1000)
        (self.grabbed, self.frame) = self.stream.read()
        # self.frame = cv2.flip(self.frame, -1)
        self.stopped = False

    def start(self):    
        Thread(target=self.get, args=()).start()
        return self

    def get(self):
        while not self.stopped:
            if not self.grabbed:
                self.stop()
            else:
                (self.grabbed, self.frame) = self.stream.read()

    def stop(self):
        self.stopped = True