import cv2
import numpy as np
import time
from mouse_keyboard import Mouse


MONITOR_WIDTH = 2560
MONITOR_HEIGHT = 1440
CAMERA_WIDTH = 800
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 600
CAMERA_HEIGHT = 1080

m = Mouse()

if __name__ == "__main__":
    print("start")

camera = cv2.VideoCapture(0)

camera.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)


if camera.isOpened():
    
    
    ret, frame = camera.read()
    frame = cv2.flip(frame, -1)
    frame = frame[350:-100, 1200:-100]
    
    cv2.namedWindow("Select Mouse")
    cv2.imshow("Select Mouse", frame)
    
    mouseRect = cv2.selectROI("Select Mouse", frame, showCrosshair=True)
    
    cv2.destroyWindow("Select Mouse")
    
    tracker = cv2.TrackerCSRT_create()
    # tracker = cv2.TrackerKCF_create()
    
    tracker.init(frame, mouseRect)
    
    cv2.namedWindow("camera")
    
    cv2.createTrackbar("cutX", "camera", 0, CAMERA_WIDTH, lambda x : x)
    cv2.createTrackbar("cutY", "camera", 0, CAMERA_HEIGHT, lambda x : x)

    cv2.setTrackbarPos("cutX", "camera", 1200) # 950
    cv2.setTrackbarPos("cutY", "camera", 350) # 170
    
    
    cutX = 1200
    cutY = 350
    
    prevTime = 0
    
    mouse_move = True
    
    while True:
        ret, frame = camera.read()
        if ret:
            
            cv2.namedWindow("camera")
            
            frame = cv2.flip(frame, -1)
            frame = frame[cutY:-100, cutX:-100]
            
            curTime = time.time()
            sec = curTime - prevTime
            prevTime = curTime
            fps = 1 / (sec)
            str = "FPS:%.1f" % fps
            cv2.putText(frame, str, (0, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0))
            
            
            success, box = tracker.update(frame)
            
            left, top, w, h = [int(v) for v in box]
            try:
                centerX, centerY = left + w / 2, top + h / 2
                # print(centerX, centerY)
                centerX = int(centerX / (CAMERA_WIDTH - cutX - 100) * MONITOR_WIDTH) # 930
                centerY = int(centerY / (CAMERA_HEIGHT - cutY - 100) * MONITOR_HEIGHT) # 250
                # print(left, top, w, h)
                # pydirectinput.moveTo(centerX, centerY)
                if mouse_move:
                    m.move_mouse((centerX, centerY))
                    
            except ZeroDivisionError:
                pass
            
            cv2.rectangle(frame, pt1=(left, top), pt2=(left + w, top + h), color=(0, 255, 0), thickness=3)
            
            cutX = cv2.getTrackbarPos("cutX", "camera")
            cutY = cv2.getTrackbarPos("cutY", "camera")

            _, binary = cv2.threshold(frame, cutY, cutX, cv2.THRESH_BINARY)
            
            
            cv2.imshow("camera", frame)
            if cv2.waitKey(1) == ord("q"):
                break
            # elif cv2.waitKey(0) == ord("1"):
                # mouse_move = True
            # elif cv2.waitKey(0) == ord("2"):
                # mouse_move = False
        else:
            print("no frame")
            break
else:
    print("can't open camera.")

camera.release()
cv2.destroyAllWindows()