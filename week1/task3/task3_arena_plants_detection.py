import cv2
import numpy as np

#aruco marker detection 
img = cv2.imread("C:/python/opencv/aeroXprojects/week1/task3/Task3_Aero.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)



aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
corners, ids, rejected = detector.detectMarkers(gray)

cv2.aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))

# yellow flowers detection
# crop_img = img[150:223,525:636]
crop_img = img[223:296,585:696]

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
# y,x = 186,585
# print(hsv[y,x]) #[ 22 227  99]#for flowers

a=0
lower_yellow = np.array([20,220,90])
upper_yellow = np.array([25,235,110])
mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
mask = cv2.GaussianBlur(mask, (9,9), 0) # bluring to get the edge detected
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
for cnt in contours:
    area = cv2.contourArea(cnt)
    if area > 1000:
        cv2.drawContours(img, [cnt], -1, (0,0,255), 2)
        a+=1
print("no.of yellow flowers detected:", a)

#border detection
# y,x = 259,585
# print(hsv[y,x]) # [  5 255  22] #for border

border_L = np.array([00,245,10])
border_U = np.array([15,255,35])
mask2 = cv2.inRange(hsv, border_L, border_U)
mask2 = cv2.GaussianBlur(mask2, (9,9), 0) # bluring to get the edge detected
contours, _ = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
for cnt in contours:
    area = cv2.contourArea(cnt)
    if area > 1000:
        cv2.drawContours(img, [cnt], -1, (255,0,0), 2)



cv2.namedWindow("image", cv2.WINDOW_NORMAL)
# cv2.imshow("image", mask2)
cv2.imshow("image",img)
# cv2.imshow("image", crop_img)
# cv2.imwrite("task3_output_arena_plants.png", img)
cv2.waitKey(0) 