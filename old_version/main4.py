from threading import Thread, Lock
import cv2
import numpy as np
import time
from mouse_keyboard import Mouse
import keyboard
import pickle

class WebcamVideoStream:
    def __init__(self, src = 0, width = 1280, height = 720):
        self.stream = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.stream.set(cv2.CAP_PROP_FPS, 60)
        self.stream.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        self.stream.set(cv2.CAP_PROP_EXPOSURE, -7)
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
        
mouse_area = np.empty((0, 2), int)
mouse_area = np.append(mouse_area, np.array([[222, 351], [431, 365], [416, 483], [216, 482]]), axis=0)
# 222 351
# 431 365
# 416 483
# 216 482

area_left = np.empty((0, 2), int)
area_top = np.empty((0, 2), int)
area_right = np.empty((0, 2), int)
area_bottom = np.empty((0, 2), int)

print("left", area_left)
print("top", area_top)
print("right", area_right)
print("bottom", area_bottom)

def cal_dist(x1, y1, x2, y2, a, b):
    area = abs((x1-a) * (y2-b) - (y1-b) * (x2 - a))
    AB = ((x1-x2)**2 + (y1-y2)**2) **0.5
    distance = area/AB
    return distance

def MouseCallback(events, x, y, flags, params):
    global mouse_area
    # global area_left
    # global area_top
    # global area_right
    # global area_bottom
    
    if events == cv2.EVENT_LBUTTONDOWN:
        print(x, y)
        # print(mouse_area)
        mouse_area = np.append(mouse_area, np.array([[x, y]]), axis=0)
        # print(mouse_area)
        
    if events == cv2.EVENT_RBUTTONDOWN:
        try:
            mouse_area = np.delete(mouse_area, -1, axis=0)
            print("deleted")
            
        except IndexError:
            print("no more can be deleted")
            pass


if __name__ == "__main__":
    
    MONITOR_WIDTH = 2560
    MONITOR_HEIGHT = 1440
    CAMERA_WIDTH = 1920
    CAMERA_HEIGHT = 1080
    
    camera = WebcamVideoStream(src=0, width=CAMERA_WIDTH, height=CAMERA_HEIGHT).start()
    
    # frame = camera.read()
    # frame = cv2.flip(frame, -1)
    # frame = frame[350:-300, 1200:-300]
    
    cutX = 1200
    cutY = 350
    hue = 0
    saturation_min = 0
    saturation_max = 0
    value_min = 0
    value_max = 0
    
    cv2.namedWindow("camera")

    cv2.createTrackbar("hue", "camera", 0, 255, lambda x : x)
    cv2.createTrackbar("saturation min", "camera", 0, 255, lambda x : x)
    cv2.createTrackbar("saturation max", "camera", 0, 255, lambda x : x)
    cv2.createTrackbar("value min", "camera", 0, 255, lambda x : x)
    cv2.createTrackbar("value max", "camera", 0, 255, lambda x : x)

    # cv2.setTrackbarPos("hue", "camera", 110)
    cv2.setTrackbarPos("hue", "camera", 170)
    # cv2.setTrackbarPos("saturation min", "camera", 35)
    cv2.setTrackbarPos("saturation min", "camera", 110)
    cv2.setTrackbarPos("saturation max", "camera", 255)
    # cv2.setTrackbarPos("value min", "camera", 90)
    cv2.setTrackbarPos("value min", "camera", 40)
    cv2.setTrackbarPos("value max", "camera", 255)
        
    # if exist saved_roi then load roi
    # or not exist, make new roi
    try:
        with open("saved_roi", "rb") as file:
            roi = pickle.load(file)
    except:
        roi_frame = camera.read()
        roi = cv2.selectROI("select mouse pad then press 'space' or 'enter'", roi_frame, showCrosshair=True)
        cv2.destroyWindow("select mouse pad then press 'space' or 'enter'")
        with open("saved_roi", "wb") as saved_roi:
            pickle.dump(roi, saved_roi)
    
    m = Mouse()
    prevTime = 0
        
    # fgbg = cv2.bgsegm.createBackgroundSubtractorMOG(history = 200, nmixtures = 5, \
                    # backgroundRatio = 0.7, noiseSigma = 0)
    # fgbg = cv2.bgsegm.createBackgroundSubtractorMOG()
    # fgbg = cv2.createBackgroundSubtractorMOG2(history = 500, varThreshold = 16, \
        # detectShadows = False)
    
    while True:
        frame = camera.read()
        
        # frame = cv2.flip(frame, -1) # flip the frame
        
        frame = frame[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]] # cutting the not used section
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # convert bgr to hsv
        cv2.polylines(frame, [mouse_area], isClosed=True, color=(0, 255, 0), thickness=1)
        
        masked_image = cv2.inRange(hsv, (hue-30, saturation_min, value_min), (hue+30, saturation_max, value_max))
        # imgBlur = cv2.GaussianBlur(masked_image, (3, 3), 1)
        # imgThreshold = cv2.adaptiveThreshold(imgBlur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, \
        #                             cv2.THRESH_BINARY_INV, 25, 16)
        
        # result = cv2.bitwise_and(frame, frame, mask=masked_image)
        
        # morphology
        # kernel = np.ones((22, 22), np.uint8)
        # masked_image = cv2.morphologyEx(masked_image, cv2.MORPH_OPEN, kernel)
        # masked_image = cv2.morphologyEx(masked_image, cv2.MORPH_CLOSE, kernel)
        # fgmask = fgbg.apply(frame)
        
        numOfLabels, img_label, stats, centroids = cv2.connectedComponentsWithStats(masked_image)

        for idx, centroid in enumerate(centroids):
            if stats[idx][0] == 0 and stats[idx][1] == 0:
                continue

            if np.any(np.isnan(centroid)):
                continue
            
            x, y, width, height, area = stats[idx]
            centerX, centerY = int(centroid[0]), int(centroid[1])
            
            if area > 800:
                
                cv2.circle(frame, (centerX, centerY), 10, (0, 0, 255), 10)
                cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 0, 255))
                
                # if cv2.waitKey(1) == 32:
                #     print(centerX, centerY)
                #     mouse_area = np.append(mouse_area, np.array([[x, y]]), axis=0)
                try:
                    top_distance = cal_dist(mouse_area[0, 0], mouse_area[0, 1], mouse_area[1, 0], mouse_area[1, 1], centerX, centerY)
                    left_distance = cal_dist(mouse_area[0, 0], mouse_area[0, 1], mouse_area[3, 0], mouse_area[3, 1], centerX, centerY)
                
                    # if cv2.waitKey(1) == ord("a"):
                    #     print(centerX, centerY)
                    #     print(top_distance)
                    #     print(left_distance)
                    # print(area)
                    nowX, nowY = m.get_position()
                    # nowX += 1
                    # nowY += 1 # offset.
                    
                    
                    centerX = (left_distance / area_top * MONITOR_WIDTH)
                    centerY = (top_distance / area_left * MONITOR_HEIGHT)
                    # print(centerX, centerY)
                    # print("\n", nowX, centerX, "\n", nowY, centerY)
                    
                    # cursor smoothing
                    draw_steps = 3  # total times to update cursor

                    dx = (centerX-nowX)/draw_steps
                    dy = (centerY-nowY)/draw_steps

                    for n in range(draw_steps):
                        x = int(nowX+dx*n)
                        y = int(nowY+dy*n)
                        m.move_mouse((x,y))
                        # time.sleep(dt)
                    
                    
                    # m.move_mouse((centerX, centerY)) # no smoothing

                except:
                    pass
        
        try:
            hue = cv2.getTrackbarPos("hue", "camera")
            
            saturation_min = cv2.getTrackbarPos("saturation min", "camera")
            saturation_max = cv2.getTrackbarPos("saturation max", "camera")
            value_min = cv2.getTrackbarPos("value min", "camera")
            value_max = cv2.getTrackbarPos("value max", "camera")
            
            area_left = np.sqrt((mouse_area[0, 0] - mouse_area[3, 0])**2 + (mouse_area[0, 1] - mouse_area[3, 1])**2)
            area_top = np.sqrt((mouse_area[0, 0] - mouse_area[1, 0])**2 + (mouse_area[0, 1] - mouse_area[1, 1])**2)
            area_right = np.sqrt((mouse_area[1, 0] - mouse_area[2, 1])**2 + (mouse_area[1, 1] - mouse_area[2, 1])**2)
            area_bottom = np.sqrt((mouse_area[2, 0] - mouse_area[3, 0])**2 + (mouse_area[2, 1] - mouse_area[3, 1])**2)
            
        except:
            pass

        
        
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
        cv2.imshow("masked image", masked_image)
        # cv2.imshow("blured image", imgBlur)
        # cv2.imshow("Thresholded image", imgThreshold)
        cv2.setMouseCallback("camera", MouseCallback)
        # cv2.imshow("fgmask", fgmask)
        
        if cv2.waitKey(1) == 32:
            if keyboard.is_pressed("esc+space"):
                break
            if keyboard.is_pressed("a"):
                print(centerX, centerY)
                mouse_area = np.append(mouse_area, np.array([[x, y]]), axis=0)
                
                
    camera.stop()
    cv2.destroyAllWindows()