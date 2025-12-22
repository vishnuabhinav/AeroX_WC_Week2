'''

PID values = (7, 0, 367)

Task 2.2 - 

A car is loaded. You have to develop a PID controller for that car such that it runs along the line y = 0.
The line is also visible on the plane.
Callibrate the PID gains such that car gets to the line as fast as possible and follows it without much disturbance.
Refer to the past two taks and their codes for hints.



INSTRUCTIONS -
    Select the simulation window and Press ENTER to execute


'''




import numpy as np 
import pybullet as p 
import time
import math
import cv2

p_id = p.connect(p.GUI)                #Loading the simulation
p.setGravity(0, 0, -10)                #Setting the gravity

plane = p.loadURDF("C:/python/aeroXprojects/week2/task2/src/plane.urdf")        #Loading the plane
carPos = [0,3,0.1]                      #This is where the car will spawn, this is constant. Don't change

m = 0                           #Declaring the slope of the required line y = mx + c
c = 0                           #Declaring the contsnat of the reuired line  y = mx + c
angle = math.atan(m)

car = p.loadURDF("C:/python/aeroXprojects/week2/task2/src/car/car1.urdf", carPos, p.getQuaternionFromEuler([0,0,angle]))  #Loading the car with head parallel to the given line


def printLine(m, c):                        #This functions draws a line that we need to follow
    angle = math.atan(m)
    z = 0.02
    origin = [0,c,z]
    line = p.loadURDF("C:/python/aeroXprojects/week2/task2/src/line.urdf", origin, p.getQuaternionFromEuler([0,0,angle]))

printLine(m, c)                    #Calling the function to print the line


num = p.getNumJoints(car)                  #Getting the total number of joints in the car
for i in range(num):
    print(p.getJointInfo(car, i))           #Printing the information of each joint to get the motor joints


#These are the 4 motor joints that we need to manipulate, we declare them here.

fl = 2
fr = 3
bl = 4
br = 5

p.setJointMotorControlArray(car, [fl, bl, fr, br], p.VELOCITY_CONTROL, forces = [0,0,0,0])   #This is done to enable torque control in wheels of the car
p.stepSimulation()


'''
Above this is the loading code, make no changes to it
Below this is the code that you need to work with.
'''

def callback():            #Dummy function for the track-bars
    pass

#P-D gains to be adjusted
cv2.namedWindow('controls')                   #Creating Track-Bars that can be used to adjust the PID values in real time.
cv2.createTrackbar('P', 'controls', 0, 500, callback)       #Setting the lower and upper limits on track-bars
cv2.createTrackbar('I', 'controls', 0, 500, callback)       #Creating three different track-bars for each P-I-D
cv2.createTrackbar('D', 'controls', 0, 500, callback)

P=cv2.getTrackbarPos('P', 'controls')/10                #Loading the PID constants from the trackbars
I=cv2.getTrackbarPos('I', 'controls')/10000
D=5*cv2.getTrackbarPos('D', 'controls')
#press escape key to execute
k=cv2.waitKey(1) & 0xFF                                 #This is needed to keep the track-bars active in real time
#P, D = 0.1, 0.5

#Declare the desired_state and base_torque globally
desired_state = 0.0  #Set Value Yourself
base_torque = 15   #Set Value Yourself


def moveCar( base_torque, action):  #Enter the motor control here to move the car, give base torque and action calculated as input
    t_fl = base_torque - action
    t_bl = base_torque - action
    t_fr = base_torque + action 
    t_br = base_torque + action 

    p.setJointMotorControlArray(car, [fl, bl, fr, br], p.TORQUE_CONTROL, forces=[t_fl, t_bl, t_fr, t_br])

    #Use p.JointMotorControlArray() function in torque mode
    #Use differential drive to nullify the error
    #The differential drive must increase or decrease the speed of the tyres about a constant base torque using gains



def calc_error(desired_state, derivative, prev_error, integral): #You can calculate the error and required action using this function
    state = p.getBasePositionAndOrientation(car)[0][1]    #y coordinate
    # orientation = p.getBasePositionAndOrientation(car)[1]
    error = state - desired_state
    derivative = error - prev_error
    prev_error = error
    if(p.getBaseVelocity(car)[0][1]<0.01):
        integral += error
    pid = P * error + D * derivative + I * integral
    action = -pid 
    return  action, derivative, integral, prev_error






#Select the simulation window and Press ENTER to execute




while(True):                         #This while loop will run until ESCAPE key is pressed, then it will start the simulation.
    keycode = p.getKeyboardEvents()       #Getting the keyboard events through PyBullet
    if keycode.get(p.B3G_RETURN) == 1:                     #As soon as any key is pressed and it's ENTER key, simulation starts
        integral =0
        derivative = 0
        prev_error = 0
        error = 0
        p.resetSimulation()       #Simulation is reseted
        p.setGravity(0, 0, -10)

        plane = p.loadURDF("C:/python/aeroXprojects/week2/task2/src/plane.urdf")
        car = p.loadURDF("C:/python/aeroXprojects/week2/task2/src/car/car1.urdf", carPos, p.getQuaternionFromEuler([0,0,angle]))   #Plane and car loaded again
        p.setJointMotorControlArray(car, [fl, bl, fr, br], p.VELOCITY_CONTROL, forces = [0,0,0,0])   #This is done to enable torque control in wheels of the car
        printLine(m, c)   #This draws a line along y = 0, which we have to follow

        while(True):
            P=cv2.getTrackbarPos('P', 'controls')/10     #Get P from trackbar, dividing P by 10 to get it into range of 0-50 from 0-500 as desired value is in range of 0-50 and track-bar return values between 0-500
            I=cv2.getTrackbarPos('I', 'controls')/1000    #Get I from trackbar, dividing I by 10000 to get it into range of 0-0.05 from 0-500 as desired value is in range of 0-0.05 and track-bar return values between 0-500
            D=5*cv2.getTrackbarPos('D', 'controls')       #Get D from trackbar, desired value is in range of 0-500 only

            p.resetDebugVisualizerCamera(7, -90, -45, p.getBasePositionAndOrientation(car)[0])  #This will keep the camera on the car always
            p.stepSimulation()    #This steps the simulation further by 0.01 seconds approx


            k = cv2.waitKey(1)    #Uncomment this while using trackbars, otherwise they won't work in real time.
            action, derivative, integral, prev_error = calc_error(desired_state, derivative, prev_error, integral)
            moveCar(base_torque, action)
            #Call all the other functions inside this while loop
            time.sleep(1./240.)
            print("running")

            keycode = p.getKeyboardEvents()    #This will keep tracking if ENTER key is pressed again.
            if keycode.get(p.B3G_RETURN) == 1:              #We end the current simulation and start a new one again if ENTER key is pressed
                print("Episode finished")                   #This is a way to re-run the simulation without re-executing the code
                p.resetSimulation()  #Reseting the simulation
                break                #Breaking out of the inner while loop

    


