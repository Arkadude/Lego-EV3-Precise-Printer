#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks import nxtdevices as nxt
from pybricks.media.ev3dev import SoundFile, ImageFile
import math




# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()
#motors
Y=Motor(Port.A, Direction.CLOCKWISE) #adjusted to convert from mm
X=Motor(Port.B, Direction.COUNTERCLOCKWISE)
Pen=Motor(Port.C, Direction.COUNTERCLOCKWISE)

#sensors
TouchX=TouchSensor(Port.S2)
TouchY=nxt.TouchSensor(Port.S1)

# Write your program here.
def ClosePrinter():
    with open('calibration.txt', 'w') as f:
        #save to file current motor positions
        f.write(str([X.angle(), Y.angle(), Pen.angle()]))
    wait(1000)

def OpenPrinter():
    with open('calibration.txt', 'r') as f:
        MotorPositions=f.read().strip('[]')
        #parse to list
        MotorPositions=MotorPositions.replace(' ','').split(',')
        print(MotorPositions)
        #update motor angles
        X.reset_angle(int(MotorPositions[0]))
        Y.reset_angle(int(MotorPositions[1]))
        Pen.reset_angle(int(MotorPositions[2]))
def TestTouch():
    for i in range(10):
        print("X sensor: "+str(TouchX.pressed()))
        print("Y sensor: "+str(TouchY.pressed()))
        wait(100)

def WaitForTouch(sensor):
    while not (sensor.pressed() or Button.CENTER in ev3.buttons.pressed()):
        wait(1)
def WaitForRelease(sensor):
    while sensor.pressed():
        wait(1)

def CalibrateMotor(m,sm):
    #motor runs till it hits the sensor, then backs up and gradually approaches again
    if sm.pressed(): #back off a little first
        m.run_angle(720, -720, then=Stop.HOLD, wait=True)
    m.run(1000)
    WaitForTouch(sm)
    m.hold()
    ev3.speaker.beep()
    m.run_angle(360, -360, then=Stop.HOLD, wait=True)
    m.run(100)
    WaitForTouch(sm)
    m.hold()
    ev3.speaker.beep()
    #reset tacho count
    m.reset_angle(0)
    print(str(m)+" Was Calibrated")

def CalibratePen():
    while not Button.CENTER in ev3.buttons.pressed():
        if Button.UP in ev3.buttons.pressed():
            Pen.run(80)
        elif Button.DOWN in ev3.buttons.pressed():
            Pen.run(-80)
        else:
            Pen.hold()
    Pen.reset_angle(0)
    RaisePen()
    
    ev3.speaker.beep()

def CalibrationSequence():
    CalibrateMotor(X, TouchX)
    CalibrateMotor(Y, TouchY)
    CenterMotors()
    CalibratePen()

def LoadPaper():
    StraightTo(0,-60)
    while not Button.CENTER in ev3.buttons.pressed(): #wait for user to load/unload paper
        wait(1)
    #ReturnToCenter()   

def MoveBy_mm(m, speed, dist, wait):
    dist=dist*360/3.18 #conversion from mm to motor degrees
    if int(speed)==0: #Apparently speed can't be 0
        speed=1

    m.run_angle(speed, dist, then=Stop.HOLD, wait=wait)
def MoveTo_mm(m, speed, dest, wait):
    dest=dest*360/3.18 #conversion from mm to motor degrees
    if int(speed)==0: #Apparently speed can't be 0
        speed=1
    m.run_target(speed, dest, then=Stop.HOLD, wait=wait)
def GetPos(m):
    return m.angle()*3.18/360
def PrintCoords():
    print("X at: "+str(GetPos(X)))
    print("Y at: "+str(GetPos(Y)))
    print("Pen at: "+str(GetPos(Pen)))
    
def CenterMotors():
    MoveTo_mm(X, 720, -63, False)
    MoveTo_mm(Y, 720, -69.5, True)
    X.reset_angle(0)
    Y.reset_angle(0)
    wait(1000) #kill time until arrival
    ev3.speaker.beep()


def ReturnToCenter():
    MoveTo_mm(X, 720, 0, False)
    MoveTo_mm(Y, 720, 0, True)
    while (GetPos(X)**2+GetPos(Y)**2)>1: #Kill time until both motors arrive
        wait(1)
    wait(1000)
    ev3.speaker.beep()
def RaisePen():
    Pen.run_target(360, 360, then=Stop.HOLD, wait=True)

def LowerPen():
    Pen.run_target(360, 0, then=Stop.HOLD, wait=True)


def StraightTo(x,y):
    speed=500
    dx=abs(x-GetPos(X))
    dy=abs(y-GetPos(Y))
    Vx=speed*dx/math.sqrt(dx**2+dy**2)
    Vy=speed*dy/math.sqrt(dx**2+dy**2)
    print(Vx)
    print(Vy)
    if int(Vx)>0:
        MoveTo_mm(X,Vx,x,False)
    if int(Vy)>0:       
        MoveTo_mm(Y,Vy,y,True)
    while ((x-GetPos(X))**2+(y-GetPos(Y))**2)>4: #Kill time until both motors arrive
        wait(1)
    
    

def lineTest():
    verticals=50 #resolution of the image
    horizontals=50
    widthX=50.0 #width in mm
    widthY=50.0
    stepX=widthX/verticals #step size for vertical lines
    stepY=widthY/horizontals #step size for horizontal lines

    StraightTo(-widthX/2, -widthY/2)
    d=1
    for i in range(10):
        LowerPen()
        MoveBy_mm(X,300,50*d,True)
        RaisePen()
        MoveBy_mm(Y,300,stepY,True)
        d=d*(-1)

def readImage(filename):
    
    with open(filename, 'r') as f:
        matrix=f.readlines()
        for line, value in enumerate(matrix):
            matrix[line]=value.strip('\r\n').split(',')
    imagewidth=len(matrix[0])
    imageheight=len(matrix)
            
    print("image width:"+ str(imagewidth))
    print("image height:"+ str(imageheight))
    ev3.speaker.beep()
    while not Button.CENTER in ev3.buttons.pressed(): #wait for user input
        wait(1)
    return matrix,imagewidth,imageheight

def drawLineByLine(matrix, imagewidth, imageheight):
    speed=500 #motor speed for this procedure
    lines=100 #how many lines to pass
    width_mm=70.0 #canvas width
    height_mm=70.0 #canvas height
    vertical_step_mm=height_mm/lines
    horizontal_step_mm=width_mm/imagewidth
    passdirection=0   

    #move pen to starting position
    RaisePen()
    StraightTo(-width_mm/2, height_mm/2)

    for i in reversed(range(lines-1)): #each loop is a horizontal pass
        i_matrix=int(i*imageheight/lines)
        
        #going left to right
        if passdirection==0:
            #check the first pixel in current line
            firstpixel=bool(int(matrix[i_matrix][0]))
            if firstpixel==False: #if first pixel in line is black lower pen
                MoveTo_mm(X,speed,-width_mm/2,True)
                LowerPen()
            lookingfor=not firstpixel #the initial pixel determines what to start from
            
            #FALSE is BLACK
            #TRUE is WHITE

            print ("currently on line"+ str(i_matrix))
            #print(matrix[i_matrix])

            for index, value in enumerate(matrix[i_matrix]): #loops the pixel values in each line

                #print(bool(value))
                if bool(int(value))==lookingfor: #if current pixel is what we're looking for
                    print("looking for"+str(lookingfor))
                    #move the pen over the pixel
                    MoveTo_mm(X,speed,(-width_mm/2+horizontal_step_mm*index),True)
                    #decide if to lower or raise the pen
                    if lookingfor==True: #if we found a white pixel
                        RaisePen()
                    else: #if we found a black pixel (need to draw)
                        LowerPen()

                    #flip what we're looking for
                    lookingfor= not lookingfor
            passdirection=1 #next pass will be right to left

        else:
            #check the first pixel in current line
            firstpixel=bool(int(matrix[i_matrix][-1]))
            if firstpixel==False: #if first pixel in line is black lower pen
                MoveTo_mm(X,speed,width_mm/2,True)
                LowerPen()
            lookingfor=not firstpixel #the initial pixel determines what to start from
            
            #FALSE is BLACK
            #TRUE is WHITE

            print ("currently on line"+ str(i_matrix))
            #print(matrix[i_matrix])

            for index, value in reversed(list(enumerate(matrix[i_matrix]))): #loops the pixel values in each line

                #print(bool(value))
                if bool(int(value))==lookingfor: #if current pixel is what we're looking for
                    print("looking for"+str(lookingfor))
                    #move the pen over the pixel
                    MoveTo_mm(X,speed,(-width_mm/2+horizontal_step_mm*index),True)
                    #decide if to lower or raise the pen
                    if lookingfor==True: #if we found a white pixel
                        RaisePen()
                    else: #if we found a black pixel (need to draw)
                        LowerPen()

                    #flip what we're looking for
                    lookingfor= not lookingfor
            passdirection=0 #next pass will be left to right

                
        #raise pen before moving a line
        RaisePen()
        #return X to the start of the line - NOT REQUIRED
        #MoveTo_mm(X,speed,-width_mm/2,True)

        #move down a line
        MoveBy_mm(Y,speed,-vertical_step_mm,True)


    
#CalibrateMotor(X,TouchX)
#CalibrateMotor(Y,TouchY)
#CenterMotors()
#StraightTo(35,-35)
OpenPrinter()
CalibrationSequence()

print(X.angle())
print(Y.angle())
print(Pen.angle())



#ReturnToCenter()
#load Paper
LoadPaper()
ReturnToCenter()
#draw stuff


matrix,imagewidth,imageheight=readImage('thanks.txt')
drawLineByLine(matrix, imagewidth, imageheight)

#Unload Paper
LoadPaper()

PrintCoords()
ClosePrinter()
