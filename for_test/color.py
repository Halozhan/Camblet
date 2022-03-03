import numpy as np
import cv2

cap = cv2.VideoCapture(0)

if cap.isOpened():
    while True:
        yes_or_no, frame = cap.read()
        
        if yes_or_no:
            
            frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)

            src_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            #  0 < B < 100 ,   128 < G < 255 , 0 < R < 100
            dst1 = cv2.inRange(frame, (0, 128, 0), (100, 255, 100))
            img_result = cv2.bitwise_and(src_hsv, src_hsv, mask = dst1)

            dst2 = cv2.inRange(src_hsv, (50, 150, 0), (80, 255, 255))
            img_result2 = cv2.bitwise_and(src_hsv, src_hsv, mask = dst2)

            cv2.imshow('src', frame)
            cv2.moveWindow('src',400,100)

            cv2.imshow('dst1', dst1)
            cv2.moveWindow('dst1',400,450)

            cv2.imshow('img_result', img_result)
            cv2.moveWindow('img_result',800,450)


            cv2.imshow('dst2', dst2)
            cv2.moveWindow('dst2',400,800)


            cv2.imshow('img_result2', img_result2)
            cv2.moveWindow('img_result2',1100,450)
            
            if cv2.waitKey(1) == ord("q"):
                break
        else:
            print("no frame")
            break
else:
    print("can't open camera.")
    
cap.release()
cv2.destroyAllWindows()