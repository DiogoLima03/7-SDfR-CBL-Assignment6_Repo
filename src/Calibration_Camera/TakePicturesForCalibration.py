import cv2
import os

if not os.path.exists('ImagesForCalibration'):
    os.makedirs('ImagesForCalibration')

cap = cv2.VideoCapture(0)

num = 0

while cap.isOpened():
    
    success, img = cap.read()
    
    cv2.imshow('Img', img)
    k = cv2.waitKey(5)
    
    if k == 27:
        break
    elif k == ord('s'): # wait for 's' key to save and exit
        cv2.imwrite('ImagesForCalibration/img' + str(num) + '.png', img)
        print("Image " + str(num) + " saved!")
        num += 1
        
    
# Release the camera and close all OpenCV windows
cap.release()
cv2.destroyAllWindows()

