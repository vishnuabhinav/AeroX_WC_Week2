import cv2

img = cv2.imread("C:/python/opencv/aeroXprojects/week1/task1/Task-1_AeroX.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


# cv2.imshow("gray", gray)
# cv2.imshow("img", img)

# detecting aruco markers
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
#detecting parameters
parameters = cv2.aruco.DetectorParameters()

# create detector
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

corners, ids, rejected = detector.detectMarkers(gray)

# print("corners\n",corners)
# print("IDS\n",ids)
#drawing the outline of the aruco markers
cv2.aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))


for i in range(len(ids)):
    c = corners[i][0]
    cx = (c[0][0] + c[2][0])/2
    cy = (c[0][1]+ c[2][1])/2
    print(f"aruco marker with ID: {ids[i]}  has center: ({cx}, {cy})")


#for resizeable window
cv2.namedWindow("image", cv2.WINDOW_NORMAL)
cv2.imshow("image", img)
# cv2.imwrite("task1_output_aruco_annotated.png", img)
cv2.waitKey(0)