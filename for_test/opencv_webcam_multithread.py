#!/usr/bin/env python

from threading import Thread, Lock
import cv2

class WebcamVideoStream:
    def __init__(self, src = 0, width = 320, height = 240):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        (self.grabbed, self.frame) = self.stream.read()
        self.started = False
        self.read_lock = Lock()

    def start(self):
        if self.started:
            print("already started!!")
            return None
        self.started = True
        self.thread = Thread(target=self.update, args=())
        self.thread.start()
        return self

    def update(self):
        while self.started:
            (grabbed, frame) = self.stream.read()
            self.read_lock.acquire()
            self.grabbed, self.frame = grabbed, frame
            self.read_lock.release()

    def read(self):
        self.read_lock.acquire()
        frame = self.frame.copy()
        self.read_lock.release()
        return frame

    def stop(self):
        self.started = False
        self.thread.join()

    def __exit__(self, exc_type, exc_value, traceback):
        self.stream.release()

from mouse_keyboard import Mouse
m = Mouse()

if __name__ == "__main__":
    
    import time
    prevTime = 0
    highestFPS = 0
    
    vs = WebcamVideoStream(src= 1, width=1280, height=720).start()
    while True:
        frame = vs.read()
        # frame = cv2.flip(frame, -1)
        
            
        curTime = time.time()
        sec = curTime - prevTime
        prevTime = curTime
        fps = 1 / (sec)
        if highestFPS < fps:
            highestFPS = fps
            print(highestFPS)
        str = "FPS:%.1f" % fps
        cv2.putText(frame, str, (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
        # m.move_mouse((100, 200))
        
        cv2.imshow('webcam', frame)
        if cv2.waitKey(1) == 27:
            break

    vs.stop()
    cv2.destroyAllWindows()