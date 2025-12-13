import cv2

img = cv2.imread("C:/Users/vishn/Downloads/task-2AeroX.png")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

_, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

output_lines = []

for cnt in contours:
    epsilon =  0.01 * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt,epsilon, True)
    
    cv2.drawContours(img, [cnt], 0, (0, 0, 255 ), 4)  #drawing border line 
    
    sides = len(approx)

    if sides == 3:
        shape = "Triangle"
    elif sides == 4:
        shape = "Quadrilateral"
    elif sides == 5:
        shape = "Pentagon"
    elif sides == 6:
        shape = "hexagon"
    elif sides == 7:
        shape = "heptagon"
    elif sides == 8:
        shape = "octagon"
    else:
        shape = "Circle"
    
    M = cv2.moments(cnt)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
    else:
        cx, cy = 0, 0

    #put a dot at centriod
    cv2.circle(img, (cx, cy), 4, (0, 0, 255), -1)


    # # Put text on the image
    # cv2.putText(img, shape, (cx - 20, cy - 20),
    #             cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    output_lines.append(f"{shape}: with Centroid = ({cx}, {cy})") #saving to list


with open("task2_output_centroids.txt", "w") as file:
    for line in output_lines:
        file.write(line + "\n")

cv2.imshow("Contours", img)
cv2.imwrite("task2_output_contours.png", img)
cv2.waitKey(0)
