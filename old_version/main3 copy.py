from threading import Thread, Lock
import cv2
import numpy as np

class WebcamVideoStream:
    def __init__(self, src = 0, width = 1280, height = 720):
        self.stream = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # self.stream.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        # self.stream.set(cv2.CAP_PROP_EXPOSURE, -11)
        (self.grabbed, self.frame) = self.stream.read()
        self.started = False
        self.read_lock = Lock()
        
    def start(self):
        if self.started:
            print("already started")
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
        

if __name__ == "__main__":
    
    MONITOR_WIDTH = 2560
    MONITOR_HEIGHT = 1440
    CAMERA_WIDTH = 1920
    CAMERA_HEIGHT = 1080
    
    camera = WebcamVideoStream(src=1, width=CAMERA_WIDTH, height=CAMERA_HEIGHT).start()
    
    # frame = camera.read()
    # frame = cv2.flip(frame, -1)
    # frame = frame[350:-300, 1200:-300]
    
    cutX = 1200
    cutY = 350
    saturation = 0
    value = 0
    
    cv2.namedWindow("camera")
    
    cv2.createTrackbar("cutX", "camera", 0, CAMERA_WIDTH - 1, lambda x : x)
    cv2.createTrackbar("cutY", "camera", 0, CAMERA_HEIGHT - 1, lambda x : x)
    cv2.createTrackbar("saturation", "camera", 0, 255, lambda x : x)
    cv2.createTrackbar("value", "camera", 0, 255, lambda x : x)

    cv2.setTrackbarPos("cutX", "camera", 0) # 950
    cv2.setTrackbarPos("cutY", "camera", 0) # 170
    cv2.setTrackbarPos("saturation", "camera", 110)
    cv2.setTrackbarPos("value", "camera", 100) # 80 for mouse
    

    import time
    from mouse_keyboard import Mouse
    m = Mouse()
    prevTime = 0
    mouse_move = True
    
    cv2.namedWindow("camera")
    
    while True:
        frame = camera.read()
        
            
        # frame = cv2.flip(frame, -1)
        
        frame = frame[cutY:, cutX:-30]
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # masked_image = cv2.inRange(hsv, (100, 0, 0), (130, sv, sv)) tablet
        # masked_image = cv2.inRange(hsv, (0, 0, 0), (255, saturation, value)) mouse
        masked_image = cv2.inRange(hsv, (0, 200, 255), (255, 255, 255))
        
        # result = cv2.bitwise_and(frame, frame, mask=masked_image)
        
        # kernel = np.ones((22, 22), np.uint8)
        # masked_image = cv2.morphologyEx(masked_image, cv2.MORPH_OPEN, kernel)
        # masked_image = cv2.morphologyEx(masked_image, cv2.MORPH_CLOSE, kernel)
        
        numOfLabels, img_label, stats, centroids = cv2.connectedComponentsWithStats(masked_image)

        for idx, centroid in enumerate(centroids):
            if stats[idx][0] == 0 and stats[idx][1] == 0:
                continue

            if np.any(np.isnan(centroid)):
                continue
            
            x, y, width, height, area = stats[idx]
            centerX, centerY = int(centroid[0]), int(centroid[1])
            
            # if area > 2000:
            if area > 200:
                # print(area)
                nowX, nowY = m.get_position()
                nowX += 1
                nowY += 1 # offset.
                
                cv2.circle(frame, (centerX, centerY), 10, (0, 0, 255), 10)
                cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 0, 255))
                # centerX = int(centerX / (CAMERA_WIDTH - cutX - 300) * MONITOR_WIDTH)
                centerX = int(centerX / (CAMERA_WIDTH) * MONITOR_WIDTH)
                # centerY = int(centerY / (CAMERA_HEIGHT - cutY - 400) * MONITOR_HEIGHT)
                centerY = int(centerY / (CAMERA_HEIGHT) * MONITOR_HEIGHT)
                
                # print("\n", nowX, centerX, "\n", nowY, centerY)
                m.move_mouse((centerX, centerY))
                """
                draw_steps = 5  # total times to update cursor

                dx = (centerX-nowX)/draw_steps
                dy = (centerY-nowY)/draw_steps

                for n in range(draw_steps):
                    x = int(nowX+dx*n)
                    y = int(nowY+dy*n)
                    m.move_mouse((x,y))
                    # time.sleep(dt)
                """
        
        
        cutX = cv2.getTrackbarPos("cutX", "camera")
        cutY = cv2.getTrackbarPos("cutY", "camera")
        saturation = cv2.getTrackbarPos("saturation", "camera")
        value = cv2.getTrackbarPos("value", "camera")

        _, binary = cv2.threshold(frame, cutX, MONITOR_WIDTH - 1, cv2.THRESH_BINARY)
        _, binary = cv2.threshold(frame, cutY, MONITOR_WIDTH - 1, cv2.THRESH_BINARY)
        _, binary = cv2.threshold(frame, saturation, 255, cv2.THRESH_BINARY)
        _, binary = cv2.threshold(frame, value, 255, cv2.THRESH_BINARY)
        
        try:
            curTime = time.time()
            sec = curTime - prevTime
            prevTime = curTime
            fps = 1 / (sec)
            str = "FPS:%.1f" % fps
            cv2.putText(frame, str, (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
            # print(str)
        except ZeroDivisionError:
            pass
        
        cv2.imshow("camera", frame)
        # cv2.imshow("masked image", masked_image)
        
        if cv2.waitKey(1) == 27:
            break
        
    camera.stop()
    cv2.destroyAllWindows()