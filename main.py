from threading import Thread, Lock
import cv2
import numpy as np
import time
import keyboard
from mouse_keyboard import Mouse
import pickle

class WebcamVideoStream:
    def __init__(self, src = 0, width = 1280, height = 720):
        self.stream = cv2.VideoCapture(src, cv2.CAP_DSHOW)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.stream.set(cv2.CAP_PROP_FPS, 330)
        self.stream.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        self.stream.set(cv2.CAP_PROP_BRIGHTNESS, 255)
        self.stream.set(cv2.CAP_PROP_EXPOSURE, -1)
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
            global brightness
            if brightness != self.stream.get(cv2.CAP_PROP_BRIGHTNESS):
                # print(self.stream.get(cv2.CAP_PROP_BRIGHTNESS))
                self.stream.set(cv2.CAP_PROP_BRIGHTNESS, brightness)
            (grabbed, frame) = self.stream.read()
            self.read_lock.acquire()
            self.grabbed, self.frame = grabbed, frame
            self.read_lock.release()
            
    def read(self):
        self.read_lock.acquire()
        frame = self.frame.copy()
        self.read_lock.release()
        return frame
    
    def show(self):
        self.read_lock.acquire()
        frame = self.frame.copy()
        cv2.imshow("camera", frame)
        self.read_lock.release()
        
    def stop(self):
        self.started = False
        self.thread.join()
        
    def __exit__(self, exc_type, exc_value, traceback):
        self.stream.release()

class mouseMove:
    def __init__(self, draw_steps = 2):
        self.m = Mouse()
        self.curX, self.curY = self.m.get_position()
        self.centerX, self.centerY = self.m.get_position()
        self.mouse_area = np.empty((0, 2), int)
        self.mouse_area = np.append(self.mouse_area, np.array([[53, 42], [106, 45], [103, 73], [49, 70]]), axis=0)
        self.mouseEnable = True
        self.refreshed = False
        self.draw_steps = draw_steps  # total times to update cursor
        self.started = False
        
    def start(self):
        if self.started:
            pass
            # print("already started")
        else:
            self.started = True
            self.thread = Thread(target=self.update, args=())
            self.thread.start()
            return self
    
    def rePos(self, x, y):
        self.centerX = x
        self.centerY = y
        self.refreshed = True
    
    def addMouse_area(self, add_x, add_y):
        # if (self.mouse_area.size == 0) or (self.mouse_area[-1:] != (np.array([[add_x, add_y]]))):
        self.mouse_area = np.append(self.mouse_area, np.array([[add_x, add_y]]), axis=0)
        print(self.mouse_area)
        print(self.mouse_area.size)
        # print(add_x, add_y)
        
    def deleteMouse_area(self):
        try:
            self.mouse_area = np.delete(self.mouse_area, -1, axis=0)
            print("deleted")
            
        except IndexError:
            print("no more can be deleted")
            pass
    
    def update(self):
        while self.started:
            time.sleep(0.0005)
            
            
            
            
        
            self.centerX = (left_distance / area_top * MONITOR_WIDTH)
            self.centerY = (top_distance / area_left * MONITOR_HEIGHT)
            
            self.curX, self.curY = self.m.get_position()
        
            # cursor smoothing
            self.dx = (self.centerX - self.curX) / self.draw_steps
            self.dy = (self.centerY - self.curY) / self.draw_steps
        
            
            self.refreshed = False
            for n in range(self.draw_steps):
                # time.sleep(0.00025)
                if self.refreshed or not self.started or not self.mouseEnable:
                    break
                try:
                    self.x = int(self.curX + self.dx * n)
                    self.y = int(self.curY + self.dy * n)
                    if self.mouseEnable:
                        self.m.move_mouse((self.x, self.y))
                except:
                    pass
                # self.m.move_mouse((self.centerX, self.centerY))
            
    def stop(self):
        if self.started:
            self.started = False
            self.thread.join()
        else:
            pass
            # print("already stopped")

def cal_dist(x1, y1, x2, y2, a, b):
    area = abs((x1-a) * (y2-b) - (y1-b) * (x2 - a))
    AB = ((x1-x2)**2 + (y1-y2)**2) **0.5
    distance = area/AB
    return distance


def MouseCallback(events, x, y, flags, params):
    if events == cv2.EVENT_LBUTTONDOWN:
        mouse.addMouse_area(x, y)
        
    if events == cv2.EVENT_RBUTTONDOWN:
        mouse.deleteMouse_area()        


area_left = np.empty((0, 2), int)
area_top = np.empty((0, 2), int)
area_right = np.empty((0, 2), int)
area_bottom = np.empty((0, 2), int)
centerX, centerY = (0, 0)
left_distance = 0
top_distance = 0
brightness = 0

print("left", area_left)
print("top", area_top)
print("right", area_right)
print("bottom", area_bottom)


if __name__ == "__main__":
    
    MONITOR_WIDTH = 2560
    MONITOR_HEIGHT = 1440
    CAMERA_WIDTH = 640
    CAMERA_HEIGHT = 360
    
    camera = WebcamVideoStream(src=0, width=CAMERA_WIDTH, height=CAMERA_HEIGHT).start()
    mouse = mouseMove(draw_steps=2)
    
    hue = 0
    saturation_min = 0
    saturation_max = 0
    value_min = 0
    value_max = 0
    
    cv2.namedWindow("camera")

    cv2.createTrackbar("brightness", "camera", 0, 255, lambda x : x)
    cv2.createTrackbar("hue", "camera", 0, 255, lambda x : x)
    cv2.createTrackbar("saturation min", "camera", 0, 255, lambda x : x)
    cv2.createTrackbar("saturation max", "camera", 0, 255, lambda x : x)
    cv2.createTrackbar("value min", "camera", 0, 255, lambda x : x)
    cv2.createTrackbar("value max", "camera", 0, 255, lambda x : x)

    cv2.setTrackbarPos("brightness", "camera",  255)
    cv2.setTrackbarPos("hue", "camera", 164)
    cv2.setTrackbarPos("saturation min", "camera", 68)
    cv2.setTrackbarPos("saturation max", "camera", 255)
    cv2.setTrackbarPos("value min", "camera", 127)
    cv2.setTrackbarPos("value max", "camera", 255)
        
    # if exist saved_roi then load roi
    # or not exist, make new roi
    try:
        with open("saved_roi", "rb") as file:
            roi = pickle.load(file)
        mouse = mouse.start()
    except:
        roi_frame = camera.read()
        roi = cv2.selectROI("select mouse pad then press 'space' or 'enter'", roi_frame, showCrosshair=True)
        cv2.destroyWindow("select mouse pad then press 'space' or 'enter'")
        with open("saved_roi", "wb") as saved_roi:
            pickle.dump(roi, saved_roi)
    
    prevTime = 0
    
    while True:
        frame = camera.read()        
        frame = frame[roi[1]:roi[1]+roi[3], roi[0]:roi[0]+roi[2]] # cutting the not used section
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # convert bgr to hsv
        
        cv2.polylines(frame, [mouse.mouse_area], isClosed=True, color=(0, 255, 0), thickness=1)
        
        masked_image = cv2.inRange(hsv, (hue-30, saturation_min, value_min), (hue+30, saturation_max, value_max))
        
        numOfLabels, img_label, stats, centroids = cv2.connectedComponentsWithStats(masked_image)

        for idx, centroid in enumerate(centroids):
            if stats[idx][0] == 0 and stats[idx][1] == 0:
                continue

            if np.any(np.isnan(centroid)):
                continue
            
            x, y, width, height, area = stats[idx]
            centerX, centerY = centroid[0], centroid[1]
            
            if area > 25:
                
                cv2.circle(frame, (int(centerX), int(centerY)), 10, (0, 0, 255), 10)
                cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 0, 255))
                
                try:
                    top_distance = cal_dist(mouse.mouse_area[0, 0], mouse.mouse_area[0, 1], mouse.mouse_area[1, 0], mouse.mouse_area[1, 1], centerX, centerY)
                    left_distance = cal_dist(mouse.mouse_area[0, 0], mouse.mouse_area[0, 1], mouse.mouse_area[3, 0], mouse.mouse_area[3, 1], centerX, centerY)
                    
                    mouse.rePos(centerX, centerY)
                except:
                    pass
                
                
        
        try:
            brightness = cv2.getTrackbarPos("brightness", "camera")
            hue = cv2.getTrackbarPos("hue", "camera")
            
            saturation_min = cv2.getTrackbarPos("saturation min", "camera")
            saturation_max = cv2.getTrackbarPos("saturation max", "camera")
            value_min = cv2.getTrackbarPos("value min", "camera")
            value_max = cv2.getTrackbarPos("value max", "camera")
            
            area_left = np.sqrt((mouse.mouse_area[0, 0] - mouse.mouse_area[3, 0])**2 + (mouse.mouse_area[0, 1] - mouse.mouse_area[3, 1])**2)
            area_top = np.sqrt((mouse.mouse_area[0, 0] - mouse.mouse_area[1, 0])**2 + (mouse.mouse_area[0, 1] - mouse.mouse_area[1, 1])**2)
            area_right = np.sqrt((mouse.mouse_area[1, 0] - mouse.mouse_area[2, 1])**2 + (mouse.mouse_area[1, 1] - mouse.mouse_area[2, 1])**2)
            area_bottom = np.sqrt((mouse.mouse_area[2, 0] - mouse.mouse_area[3, 0])**2 + (mouse.mouse_area[2, 1] - mouse.mouse_area[3, 1])**2)
        except:
            pass

        try:
            curTime = time.time()
            sec = curTime - prevTime
            prevTime = curTime
            fps = 1 / (sec)
            str = "FPS:%.1f" % fps
            cv2.putText(frame, str, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
        except ZeroDivisionError:
            pass
        
        if keyboard.is_pressed("ctrl+f"):
            mouse.stop()
        if keyboard.is_pressed("ctrl+g"):
            mouse.start()
        
        
        cv2.imshow("camera", frame)
        cv2.imshow("masked image", masked_image)
        cv2.setMouseCallback("camera", MouseCallback)
        
        if cv2.waitKey(1) == 27:
            break
        # if keyboard.is_pressed("ctrl+space"):
        # if cv2.waitKey(1) == ord(" "):
        #     mouse.addMouse_area(centerX, centerY)
                
    mouse.stop()
    camera.stop()
    camera.__exit__(None, None, None)
    cv2.destroyAllWindows()