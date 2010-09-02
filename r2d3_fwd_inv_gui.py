######################################
# R2D3 Robotic Development 3nvironment
# SCORBOT ER-5 Simulator
# written by Anupam Jain
# Code released under The GNU GPL
# 1 Blender Unit = 200 mm
######################################

######################################
# Requires the following packages:
#   Python
#   Blender
#   Tkinter
#   pySerial
######################################

######################################
# This file is a part of R2D3.blend
#   and needs to be run only from
#   within that file in Blender
# It specifically requires the
#   geometry contained in that scene
######################################

######################################
# r2d3_fwd_inv_gui.py
# This file implements
#    GUI based FORWARD KINEMATICS
#    GUI based INVERSE KINEMATICS
#    NETWORK based ACL CONTROL
#    INBUILT ACL EDITOR
# PRESS ALT+P HERE!!
######################################

######################################
# DOCSTRING
######################################
"""
R2D3 Robotic Development 3nvironment
With integrated SCORBOT ER-5 Simulation
written by Anupam Jain
Code released under The GNU GPL
"""

######################################
# IMPORTS (AND THEIR PURPOSE :) )
######################################
# Client server programming
from socket import *
# Serial port (SCORBOT) programming
from serial import *
# ACL interpretation using regexps
import re
# Timestamping functions
from time import sleep, time, ctime
# Trigonometric functions etc.
from math import *
# Our server runs in a separate thread
import threading
# Blender imports
import Blender
from Blender import Draw, BGL, Window
from Blender.BGL import *
from Blender.Mathutils import *

######################################
#####   UTILITY FUNCTIONS    #########
######################################

# Applies rotation matrix to vector
def rotvec(mat, vec):
    a1 = mat[0][0]*vec[0]+mat[0][1]*vec[1]+mat[0][2]*vec[2]
    a2 = mat[1][0]*vec[0]+mat[1][1]*vec[1]+mat[1][2]*vec[2]
    a3 = mat[2][0]*vec[0]+mat[2][1]*vec[1]+mat[2][2]*vec[2]
    return Vector([a1,a2,a3])

# Converts any sequential type to a Vector
def toVector(seq):
    return Vector([seq[0],seq[1],seq[2]])

# Conversion between degrees and radians
def deg2rad(deg):
    return deg*pi/180
def rad2deg(rad):
    return rad*180/pi

# Gets the position of the 3D cursor
def getcursor():
    return Vector(Blender.Window.GetCursorPos())

# Gets the position of the scorbot tip
def getTip():
    rx,ry,rz = prongR.getLocation()
    lx,ly,lz = prongL.getLocation()
    return toVector(((rx+lx)/2.0, (ry+ly)/2.0, (rz+lz)/2.0))

# Gets the position of the target
def getTarget():
    return toVector(target.getLocation())

######################################
#####   Main Code Begins here   ######
######################################

# Inverse kine objects
EVENT_SLIDER_X = 10
EVENT_TEXT_2 = 11
EVENT_TEXT_3 = 12
EVENT_SLIDER_Y = 13
EVENT_SLIDER_Z = 14
EVENT_SLIDER_ORIENTATION = 15
EVENT_BUTTON_SEARCH = 16
EVENT_BUTTON_SOLVE = 17
EVENT_BUTTON_SOL1 = 18
EVENT_BUTTON_SOL2 = 19
EVENT_BUTTON_CURSOR = 20
Object_Slider_X = Draw.Create(130)
Object_Text_2 = Draw.Create("")
Object_Text_3 = Draw.Create("")
Object_Slider_Y = Draw.Create(0)
Object_Slider_Z = Draw.Create(514)
Object_Slider_Orientation = Draw.Create(70)
Object_Button_Search = Draw.Create(0)
Object_Button_Solve = Draw.Create(0)
Object_Button_Sol1 = Draw.Create(0)
Object_Button_Sol2 = Draw.Create(0)
Object_Button_Cursor = Draw.Create(0.0)

# Forward kine objects
EVENT_SLIDER_BASE = 21
EVENT_SLIDER_SHOULDER = 22
EVENT_SLIDER_ELBOW = 23
EVENT_SLIDER_WRIST = 24
EVENT_SLIDER_EE = 25
EVENT_BUTTON_GRIPPERCLOSE = 26
EVENT_BUTTON_GRIPPEROPEN = 27
EVENT_TEXT_7 = 28
EVENT_TEXT_8 = 29
Object_Slider_Base = Draw.Create(0)
Object_Slider_Shoulder = Draw.Create(-120)
Object_Slider_Elbow = Draw.Create(90)
Object_Slider_Wrist = Draw.Create(100)
Object_Slider_EE = Draw.Create(0)
Object_Button_GripperOpen = Draw.Create(0.0)
Object_Button_GripperClose = Draw.Create(0.0)
Object_Text_7 = Draw.Create("")
Object_Text_8 = Draw.Create("")

# (Server + Home + Select) Objects
EVENT_BUTTON_STARTSERVER = 30
EVENT_BUTTON_STOPSERVER = 31
EVENT_SLIDER_SELECT = 32
EVENT_BUTTON_HOME = 33
Object_Button_StartServer = Draw.Create(0.0)
Object_Button_StopServer = Draw.Create(0.0)
Object_Slider_Select = Draw.Create(0)
Object_Button_Home = Draw.Create(0.0)
Object_Text_9 = Draw.Create("")

# Teach Positions Objects
EVENT_STRING_POSITION = 40
EVENT_BUTTON_TEACH = 41
EVENT_BUTTON_MOVE = 42
Object_Text_10 = Draw.Create("")
Object_String_Position = Draw.Create("")
Object_Button_Teach = Draw.Create(0.0)
Object_Button_Move = Draw.Create(0.0)

# Get all the objects
table = Blender.Object.Get("Table")
base = Blender.Object.Get("Base")
shoulder = Blender.Object.Get("Shoulder")
elbow = Blender.Object.Get("Elbow")
wrist = Blender.Object.Get("Wrist")
ee = Blender.Object.Get("EndEffector")
prongR = Blender.Object.Get("ProngR")
prongL = Blender.Object.Get("ProngL")
target = Blender.Object.Get("Target")

# Get all the billets
billets = []
for i in range(1,10):
    billets.append(Blender.Object.Get("Billet.00"+str(i)))
print billets

# Currently picked up object
objpick = None

# Relative transform matrix of the
#   currently picked up object
#   w.r.t end effector
objpickmat = Matrix()

######################################
##   PROGRAM INTERPRETATION GLOBALS ##
######################################

# The full program as text
# We give a default program
#   to ease debugging
#"open\nhome\nsetpv bil2 0 -6 59 -52 0\nmove bil2\nclose\nsetpv cnc 123 -34 85 -51 95\nmove cnc\nopen\nhome\nmove cnc\nclose\nhome\nopen"
program = """setpv b1 0 -4 53 -49 0
setpv b2 0 -6 58 -52 0
setpv b3 11 -4 53 -49 0
setpv g1 -168 -4 54 -49 0
setpv g2 180 -6 58 -52 0
setpv g3 168 -4 53 -49 0
setpv cnc 123 -34 85 -51 95
home
open
move b1
close
home
move cnc
open
home
move cnc
close
home
move g1
open
home"""
# The current line of program being run
lineno = 0
# Whether the program is to be run now
run = 0
# The dictionary of active positions
# joint positions
positions = {}
# cartesian positions
cpositions = {}

root = None

######################################
#####   FORWARD KINEMATICS   #########
######################################

# The joint angles
# These values are the 'current' values
# They are shown in the fwd kine sliders
# and also are the ones used to draw the robot
# They are updated during forward kinematics
# They start with the HOME position values
t1, t2, t3, t4, t5 = 0,-120*pi/180,90*pi/180,100*pi/180,0
# The gripper value
# Valid Range is [0, 0.15] BU (0 - 30 mm)
grip = 0.15
# Boolean Is the gripper open?
# It is closed when 0
# open when 1
grip_is_open = 1

# Create a matrix from DH params
# (For some reason, Blender uses
# transpose of the "normal"
# matrix representation internally)
def dhmatrix(dh, q):
    a, alpha, d = dh
    theta = q
    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)
    mat = Matrix([ct,-st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0,sa,ca,d],[0,0,0,1])
    mat.transpose()
    return mat

# The matrices for the links
def basemat(q):
    basedh = 0, -1.57, 1.745
    return dhmatrix(basedh,q)

def shouldermat(q):
    shoulderdh = 1.105, 0, 0
    return dhmatrix(shoulderdh,q)

def elbowmat(q):
    elbowdh = 1.105, 0, 0
    return dhmatrix(elbowdh,q)

def wristmat(q):
    wristdh = 0, -1.57, 0
    q = q-(90*pi/180)
    return dhmatrix(wristdh,q)

# The dh params for endeffector are
# taken different from the book
# because the gripper links are
# also being considered here
def eemat(q):
    eedh = 0, (90*pi/180), 0.725
    return dhmatrix(eedh,q)

def gripperRmat(gripVal):
    gripperRdh = 0, 0, gripVal
    return dhmatrix(gripperRdh, 0)

def gripperLmat(gripVal):
    gripperLdh = 0, pi, -gripVal
    return dhmatrix(gripperLdh, 0)

# Updates the scorbot accd to t* values
def fwdkine():
    global table, base, shoulder, elbow, wrist, ee
    global t1, t2, t3, t4, t5, grip, grip_is_open, objpick
    global x,y,z,angle
    global eepos

    # The accumulating matrix
    bm = Matrix()
    # Total transformation matrix at left prong
    bmL = Matrix()
    # Total transformation matrix at right prong
    bmR = Matrix()

    bm.identity()
    table.setMatrix(bm)

    bm = basemat(t1)*bm
    base.setMatrix(bm)

    bm = shouldermat(t2)*bm
    shoulder.setMatrix(bm)

    bm = elbowmat(t3)*bm
    elbow.setMatrix(bm)

    bm = wristmat(t4)*bm
    wrist.setMatrix(bm)

    bm = eemat(t5)*bm
    ee.setMatrix(bm)

    # Decide what value to give to the gripper
    if grip_is_open: # Gripper is open
        grip = 30/200.0
    else: # Gripper is closed
        grip = 0.0
    # Don't let the gripper "squish"
    #   a picked up billet
    print objpick
    if objpick:
        grip = 13/200.0

    bmL = gripperLmat(grip)*bm
    prongL.setMatrix(bmL)

    bmR = gripperRmat(grip)*bm
    prongR.setMatrix(bmR)

    # Update the forward kine sliders
    Object_Slider_Base.val = t1*180/pi
    Object_Slider_Shoulder.val = t2*180/pi
    Object_Slider_Elbow.val = t3*180/pi
    Object_Slider_Wrist.val = t4*180/pi
    Object_Slider_EE.val = t5*180/pi

    # Update the inverse kine numbers
#    Object_Slider_X.val = ee.getLocation()[0]*200
#    x = Object_Slider_X.val
#    Object_Slider_Y.val = ee.getLocation()[1]*200
#    y = Object_Slider_Y.val
#    Object_Slider_Z.val = ee.getLocation()[2]*200
#    z = Object_Slider_Z.val
#    Object_Slider_Orientation.val = (t2+t3+t4)*180/pi
#    angle = Object_Slider_Orientation.val

# Animate the forward kinematics
def run_anim(pos):
    global t1, t2, t3, t4, t5
    print pos
    d = [0.0,0.0,0.0,0.0,0.0]
    d[0] = pos[0]-t1
    d[1] = pos[1]-t2
    d[2] = pos[2]-t3
    d[3] = pos[3]-t4
    d[4] = pos[4]-t5
    print d
    # Divide the whole sequence into
    # 'maximum change'*10 number of slices
    # i.e. we show a 1 radian change by 100 frames
    iters = int(10*max(map(abs,d)))
    print "iters=",iters
    # If the maximum change is zero
    # Then the robot is already at the right position
    if not iters:
        return
    d = map(lambda x:x/float(iters), d)
    print "d=",d
    # Animate!
    for i in range(iters):
        t1 = t1+d[0]
        t2 = t2+d[1]
        t3 = t3+d[2]
        t4 = t4+d[3]
        t5 = t5+d[4]
        fwdkine()
        Blender.Redraw()

######################################
#####   INVERSE KINEMATICS   #########
######################################
# Arm lengths
a1,a2,a3,a5 = 1.745,1.105,1.105,0.725

# The solution number to choose (0 or 1)
solution = 1

# These values are used to store the
# multiple solutions of inverse kine
# Theta5 can take any value from 0 to 570
theta1 = [0,0]
theta2 = [0,0]
theta3 = [0,0]
theta4 = [0,0]

# The target position
# These values are updated by the inv kine gui
# 
x,y,z = 0.0,0.0,0.0
# The orientation angle
# i.e. angle from the X axis towards Y axis (ccw)
angle = 0

# Inverse kinematics using a non-analytical
#     hill-climbing based incremental solution.
#     Stores the solution directly in the t* elements
def myinvkine(target):
    global x,y,z
    global t1,t2,t3,t4,t5,grip
    global table, base, shoulder, elbow, wrist, ee

    # Forward kine slider object globals
    global Object_Slider_Base
    global Object_Slider_Shoulder
    global Object_Slider_Elbow
    global Object_Slider_Wrist
    global Object_Slider_EE

    # "Searches" for the solution
    print "searching inverse kine for ", x, y, z

    # Base is immediately solvable
    t1 = atan2(target[1],target[0])
    fwdkine()

    done = 0
    while not done:
        # Store the current tip position
        tip = getTip()
        oldtip = tip

        # Shoulder
        tip = getTip()
        bm = base.getMatrix().rotationPart()
        axis = rotvec(bm, Vector([0,0,1]))
        joint_center = toVector(base.getLocation())
        totip = tip - joint_center
        totarget = target - tip
        movement_vector = CrossVecs(totip, axis)
        gradient = DotVecs(movement_vector, totarget)
        t2 = t2+gradient*(0.1)
        if 35*pi/180 < t2:
            t2 = 35*pi/180
        elif -130*pi/180 > t2:
            t2 = -130*pi/180
        fwdkine()

        # Elbow
        tip = getTip()
        bm = base.getMatrix().rotationPart()
        axis = rotvec(bm, Vector([0,0,1]))
        joint_center = toVector(shoulder.getLocation())
        totip = tip - joint_center
        totarget = target - tip
        movement_vector = CrossVecs(totip, axis)
        gradient = DotVecs(movement_vector, totarget)
        t3 = t3+gradient*(0.1)
        if 130*pi/180 < t3:
            t3 = 130*pi/180
        elif -130*pi/180 > t3:
            t3 = -130*pi/180
        fwdkine()

        # Wrist
        tip = getTip()
        bm = base.getMatrix().rotationPart()
        axis = rotvec(bm, Vector([0,0,1]))
        joint_center = toVector(elbow.getLocation())
        totip = tip - joint_center
        totarget = target - tip
        movement_vector = CrossVecs(totip, axis)
        gradient = DotVecs(movement_vector, totarget)
        t4 = t4+gradient*(0.1)
        if 130*pi/180 < t4:
            t4 = 130*pi/180
        elif -130*pi/180 > t4:
            t4 = -130*pi/180
        fwdkine()

        tip = getTip()
        totarget = target - tip
        if totarget.length < 0.01:
            done = 1
            print "Exit due to objective achieved :)"
            return 0
        if (tip-oldtip).length < 0.001:
            # If insufficient progress
            print "Exit due to insufficient progress :("
            done = 1
            return 1
        Blender.Redraw()

# Converts the 5DOF problem to 3DOF
# Assumes theta5 is not changed
# Theta1 is calculable instantly
# Finds out all the solutions and
#     stores them in the theta* lists
# Doesn't work very good!!!! :(((((
# Now superceded by the new method
def invkine_2():
    global x,y,z
    global angle
    global theta1, theta2, theta3, theta4
    global t1,t2,t3,t4,t5,grip
    global solution

    # Forward kine slider object globals
    global Object_Slider_Base
    global Object_Slider_Shoulder
    global Object_Slider_Elbow
    global Object_Slider_Wrist
    global Object_Slider_EE

    # "Solves" for the solution
    print "solving inverse kine (2) for ", x, y, z

    # Reinitialise the angles to zero
    theta1 = []
    theta2 = []
    theta3 = []
    theta4 = []

    # Theta1 is immediately solvable
    theta1 = [atan2(y,x),atan2(y,x)]

    # Calculations
    ux = cos(angle)
    uy = sin(angle)
    vx = -sin(angle)
    vy = cos(angle)

    qx = x/cos(theta1[0])
    qy = a1-z

    phi = atan2(uy,ux)

    px = qx - a5*ux
    py = qy - a5*uy

    k = (px*px + py*py - a2*a2 - a3*a3)/(2*a2*a3)
    if abs(k)>1:
        Draw.PupMenu("ERROR! No Solution Found")
        theta2 = [0,0]
        theta3 = [0,0]
        theta4 = [0,0]
        return
    th3 = acos(k)
    theta3.append(-th3)
    if abs(k) < 1:
        # Multiple solutions
        # 'Elbow up' and 'Elbow down'
        theta3.append(th3)
    else:
        theta3.append(-th3)
    for th3 in theta3:
        c3 = cos(th3)
        s3 = sin(th3)

        c2 = (px*(a2+a3*c3)+py*a3*s3)/(px*px+py*py)
        s2 = (py*(a2+a3*c3)-px*a3*s3)/(px*px+py*py)
        th2 = atan2(s2,c2)
        th4 = phi-th2-th3
        theta2.append(-th3)
        theta4.append(-th4)

    # Update the t* values to show the appropriate solution
    t1,t2,t3,t4 = theta1[solution],theta2[solution],theta3[solution],theta4[solution]

    # Update the forward kine sliders
    Object_Slider_Base.val = t1*180/pi
    Object_Slider_Shoulder.val = t2*180/pi
    Object_Slider_Elbow.val = t3*180/pi
    Object_Slider_Wrist.val = t4*180/pi
    Object_Slider_EE.val = t5*180/pi

    fwdkine()

# The inverse kine for 5DOF manipulator
# Adaptation of the book inverse kine
# Assumes theta5 to be zero
# Finds out all the solutions and
#     returns a generator with the results
# Note: In this function, theta* values are
#   not the global ones!! My mistake..
#      I should have named them different
def invkine_helper(dx,dy,dz, angle):
    global a1,a2,a3,a5
    global t1,t2,t3,t4,t5,grip

    # A zero is added to put the first angle at L[1]
    L = [0,a1,a2,a3,0,a5]

    # Take theta5 = 0
    # Actual equation would have been
    #  theta5 = atan2((-s1*nx + c1*ny), (-s1*ox + c1*oy))
    theta5 = 0

    # Theta1 should have multiple solutions given by -
    # Theta1 = [atan2(dy,dx), pi-atan2(dy,dx)]
    # But it never seems to give correct solutions for
    #   the second value of theta1
    # Thus we take only the first one
    theta1 = atan2(dy,dx)
    s1, c1 = sin(theta1), cos(theta1)

    # Try for the supplied theta234 (angle)
    # Actual equation would have been
    #    theta234 = atan2(-az, c1*ax+s1*ay)
    theta234 = angle
    s234, c234 = sin(theta234), cos(theta234)
    k1 = c1*dx + s1*dy - L[5]*c234
    k2 = -dz + L[1] - L[5]*s234
    try:
        theta3 = acos((k1*k1+k2*k2-L[3]*L[3]-L[2]*L[2])/(2*L[2]*L[3]))
    # Bad hack to prevent errors :
    # If theta3 does not exist for the specified angle
    #     then take theta3 to be previous value
    # Ideally the inverse kine target should stop changing further
    except:
        theta3 = t3
        Draw.PupMenu("ERROR! No Solution Found")
    Theta3 = (theta3,-theta3)
    for theta3 in Theta3:
        s3,c3 = sin(theta3), cos(theta3)
        theta2 = atan2(((k2*(L[3]*c3+L[2])-k1*L[3]*s3)/(L[2]*L[2]+L[3]*L[3]+2*L[2]*L[3]*c3)),((k1*(L[3]*c3+L[2])+k2*L[3]*s3)/(L[2]*L[2]+L[3]*L[3]+2*L[2]*L[3]*c3)))
        theta4 = theta234-theta2-theta3

        # Return the angles
        yield (theta1, theta2, theta3, theta4, theta5)


# Uses the new and improved calculations
#     as done in invkine_helper
# Finds out all the solutions and
#     stores them in the theta* lists
def invkine():
    global x,y,z
    global angle
    global theta1, theta2, theta3, theta4
    global t1,t2,t3,t4,t5,grip
    global solution

    # Forward kine slider object globals
    global Object_Slider_Base
    global Object_Slider_Shoulder
    global Object_Slider_Elbow
    global Object_Slider_Wrist
    global Object_Slider_EE

    # "Solves" for the solution
    print "solving inverse kine for", x, y, z

    # Show the first solution
    solution=0

    # Reinitialise the angles to zero
    theta1 = []
    theta2 = []
    theta3 = []
    theta4 = []

    # Get the solutions into theta* lists
    # invkine_helper() returns 2 solutions
    a = invkine_helper(x,y,z,angle)
    while 1:
        try:
            ans = a.next()
            print ans
        except:
            break
        theta1.append(ans[0])
        theta2.append(ans[1])
        theta3.append(ans[2])
        theta4.append(ans[3])

    # Debugging
    print theta1
    print theta2
    print theta3
    print theta4

    # Update the t* values to show the appropriate solution
    t1,t2,t3,t4 = theta1[solution],theta2[solution],theta3[solution],theta4[solution]

    # Update the forward kine sliders
    Object_Slider_Base.val = t1*180/pi
    Object_Slider_Shoulder.val = t2*180/pi
    Object_Slider_Elbow.val = t3*180/pi
    Object_Slider_Wrist.val = t4*180/pi
    Object_Slider_EE.val = t5*180/pi

    pos = (t1,t2,t3,t4,t5)
#    run_anim(pos)
    fwdkine()

######################################
########  PICKING UP BILLETS  ########
######################################

# Try to pick up a billet
def try_pickup_billet():
    global objpick
    global grip
    for b in billets:
        xb,yb,zb = b.getLocation()
        print "Billet at", (xb,yb,zb)
        xe,ye,ze = ee.getLocation()
        print "End effector at", (xe,ye,ze)
        # A billet is between the jaws
        if (abs(xb- xe)<0.5) and (abs(yb- ye)<0.5) and ((ze- zb)<0.5): #and ((ze- zb)>0.5):
            objpick = b
            ee.makeParent([b])
            grip = 15/200.0
            return

def try_drop_billet():
    global objpick
    global grip
    objpick.clrParent(2)
    objpick = None

######################################
#########   SERVER CODE  #############
######################################

# Server multithreading class
class Server(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        print "Welcome to the The R2D3 simulator server"
        print "written by Anupam Jain"
        print ""
        print "CAUTION! The server has ZERO security (no authentication)"
        print "Which means that ANYONE can log in and control the robot"
        print "You have been warned"
        print ""
        print ""

        print 'Starting Server at:', ctime(time())

        HOST = ''
        PORT = 24247
        BUFSIZ = 1024
        ADDR = (HOST, PORT)

        print "Server listening at %d:%d" % ADDR
        print ""

        # Server Socket
        ss = socket(AF_INET, SOCK_STREAM)
        ss.bind(ADDR)
        # Accept atmost two clients at a time
        # Doesn't seem to work though????????
        ss.listen(2)

        while 1:
            print 'Waiting for a client connection...'
            # Get client socket
            sc, addr = ss.accept()
            print '...connected from: ', addr
            print ""

            # Get the commands
            # Commands are separated by SEMICOLON
            # Unrecognised commands are ignored
            while 1:
                data = sc.recv(BUFSIZ)
                if data == "logoff":
                    print "'logoff' command recieved."
                    print "Shutting down client"
                    sc.close()    # Client logoff
                    break
                if data == "quit":
                    print "'quit' command recieved."
                    print "Shutting down server"
                    sc.close()    # Client logoff
                    ss.close()    # Server shutdown
                    return

                # Interpret the data
                interpret(data, sc)

        print 'Stopping Server at:', ctime(time())

def interpret(data,client):
    pass

######################################
#########   EDITOR CODE  #############
######################################
import sys, os, re, Tkinter
import tkFileDialog, tkMessageBox, tkSimpleDialog

__version__ = '0.01'
__license__ = 'GNU General Public License 2'

NAME, URI = 'R2D3 Robotic Development 3nvironment Editor', 'http://projects.sourceforge.net/project/robotsim'
SEL_FIRST, SEL_LAST = Tkinter.SEL+'.first', Tkinter.SEL+'.last'

if sys.platform[:3].lower() != 'win': FontScale = 3
else: FontScale = 0

class TextEditor(Tkinter.Frame):
    startfiledir = '.'
    ftypes = [('R2D3 programs',  '.r2d3'),
              ('Text files',  '.txt .text'),
              ('All files',   '*')]
    color, font = ['black', 'white'], ('courier', 8+FontScale, 'normal')
    filetype = 'robot definition'

    def __init__(self, parent=None, loadFirst=''):
        Tkinter.Frame.__init__(self)
        self.pack(expand=Tkinter.YES, fill=Tkinter.BOTH)
        self.start()
        self.makeMenuBar()
        self.makeToolBar()
        self.makeWidgets()
        self.setFileName([None, None])
        self.lastfind = None
        self.openDialog = None
        self.saveDialog = None
        self.text.focus()
        if loadFirst: self.open(loadFirst)
        self.master.title(NAME+' '+__version__)
        self.master.iconname(NAME)
        self.master.protocol('WM_DELETE_WINDOW', self.quit)

    def start(self):
        self.menuBar = [
            ('File', 0, 
                 [('New',         0, self.new), 
                  ('Open...',     0, self.open), 
#                  ('Close',     0, self.close), 
                  ('Save',        0, self.save), 
                  ('Save As...',  5, self.saveAs), 
                  'separator',
                  ('Quit...',     0, self.quit)]), 
            ('Edit', 0,
                 [('Cut',         0, self.cut), 
                  ('Copy',        1, self.copy), 
                  ('Paste',       0, self.paste), 
                  'separator',
                  ('Delete',      0, self.delete), 
                  ('Select All',  0, self.selectAll)]),
            ('Search', 0,
                 [('Find...',        0, self.findPrompt), 
                  ('Find next',      1, self.findAgain), 
                  ('Replace...',     0, self.replace), 
                  'separator', 
                  ('Goto line...',   0, self.gotoLine)]),
            ('Window', 0,
                 [('Next',        0, self.next_win), 
                  ('Prev',      1, self.prev_win), 
                  ('  1  ',        0, self.switch_prog(1)), 
                  ('  2  ',        0, self.switch_prog(2)), 
                  ('  3  ',        0, self.switch_prog(3)), 
                  ('  4  ',        0, self.switch_prog(4)), 
                  ('  5  ',        0, self.switch_prog(5)), 
                  ('  6  ',        0, self.switch_prog(6)), 
                  ('  7  ',        0, self.switch_prog(7)), 
                  ('  8  ',        0, self.switch_prog(8)), 
                  ('  9  ',        0, self.switch_prog(9)), 
                  ('  10 ',        0, self.switch_prog(10))]),
            ('Tools', 0,
                 [('Document Info.', 0, self.docInfo)]), 
            ('Help', 0,
                 [('About '+NAME,    0, self.help), 
                  ('Help',  0, self.documentation)])
           ]
           
        self.toolBar = [
            ('Done',  0, self.run),
            ('Save',        0, self.saveAs), 
            ('  1  ',        0, self.switch_prog(1)), 
            ('  2  ',        0, self.switch_prog(2)), 
            ('  3  ',        0, self.switch_prog(3)), 
            ('  4  ',        0, self.switch_prog(4)), 
            ('  5  ',        0, self.switch_prog(5)), 
            ('  6  ',        0, self.switch_prog(6)), 
            ('  7  ',        0, self.switch_prog(7)), 
            ('  8  ',        0, self.switch_prog(8)), 
            ('  9  ',        0, self.switch_prog(9)), 
            ('  10 ',        0, self.switch_prog(10)), 
            ('About',    0, self.help)
           ]

    

    def makeMenuBar(self):
        menubar = Tkinter.Menu(self.master)
        self.master.config(menu=menubar)

        for (name, key, items) in self.menuBar:
            pulldown = Tkinter.Menu(menubar)
            self.addMenuItems(pulldown, items)
            menubar.add_cascade(label=name, underline=key, menu=pulldown)

    def addMenuItems(self, menu, items):
        import types
        for item in items: 
            if item == 'separator': menu.add_separator({})
            elif type(item) == types.ListType: 
                for num in item: menu.entryconfig(num, state=Tkinter.DISABLED)
            elif type(item[2]) != types.ListType:
                menu.add_command(label = item[0], underline = item[1], \
                   command = item[2]) 
            else:
                p = Tkinter.Menu(menu) # pullover
                self.addMenuItems(p, item[2])
                menu.add_cascade(label = item[0], underline = item[1], menu = p)

    def makeToolBar(self):
        toolbar = Tkinter.Frame(self.master)
        for (name, key, items) in self.toolBar:
            b = Tkinter.Button(toolbar,text=name,command=items)
            b.pack(side=Tkinter.LEFT, padx=2, pady=2)
        toolbar.pack(side=Tkinter.TOP,fill=Tkinter.X)

    def makeWidgets(self): 
        vbar = Tkinter.Scrollbar(self)  
        hbar = Tkinter.Scrollbar(self, orient='horizontal')
        text = Tkinter.Text(self, padx=5, wrap='none', bg='yellow') 

        vbar.pack(side=Tkinter.RIGHT, fill=Tkinter.Y)
        hbar.pack(side=Tkinter.BOTTOM, fill=Tkinter.X)
        text.pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=Tkinter.YES)

        text.config(yscrollcommand=vbar.set)
        text.config(xscrollcommand=hbar.set)

        vbar.config(command=text.yview)
        hbar.config(command=text.xview)

        text.config(font=self.font, bg=self.color[1], fg=self.color[0])
        self.text = text

    # RUN a program
    def run(self):
        global program
        program = self.getAllText()
        print program
        self.quit()

    # File menu commands

    def new(self):
        if self.isEmpty() or tkMessageBox.askyesno(NAME, 'Discard program?'):
            self.setFileName([None, None])
            self.clearAllText()

    def my_askopenfilename(self): 
        if not self.openDialog:
           self.openDialog = tkFileDialog.Open(initialdir=self.startfiledir, 
                                  filetypes=self.ftypes)
        return self.openDialog.show()

    def open(self, loadFirst=''):
        if self.isEmpty() or tkMessageBox.askyesno(NAME, 'Discard text?'):
            file = loadFirst or self.my_askopenfilename()
            if file:
                try: text = open(file, 'r').read()
                except: tkMessageBox.showerror(NAME, 'Could not open file '+file)
                else:
                   self.setAllText(text)
                   print text
                   self.setFileName([file, 'File'])

    def my_asksaveasfilename(self): 
        if not self.saveDialog:
           self.saveDialog = tkFileDialog.SaveAs(initialdir=self.startfiledir, 
                                    filetypes=self.ftypes)
        return self.saveDialog.show()

    def save(self): 
        self.saveAs(self.current[0])

    def saveAs(self, forcefile=None): 
        file = forcefile or self.my_asksaveasfilename()
        if file:
            text = self.getAllText()
            try: open(file, 'w').write(text)
            except: 
               tkMessageBox.showerror(NAME, 'Could not write file '+str(file))
            else: self.setFileName([file, 'File'])

    # Edit menu commands

    def cut(self):
        if not self.text.tag_ranges(Tkinter.SEL):
            tkMessageBox.showerror(NAME, 'No text selected')
        else: 
            self.copy()
            self.delete()

    def copy(self): 
        if not self.text.tag_ranges(Tkinter.SEL): 
           tkMessageBox.showerror(NAME, 'No text selected')
        else:
            text = self.text.get(SEL_FIRST, SEL_LAST)  
            self.clipboard_clear()              
            self.clipboard_append(text)

    def paste(self):
        try: text = self.selection_get(selection='CLIPBOARD')
        except TclError:
            tkMessageBox.showerror(NAME, 'Nothing to paste')
            return
        self.text.insert(Tkinter.INSERT, text)
        self.text.tag_remove(Tkinter.SEL, '1.0', Tkinter.END) 
        self.text.tag_add(Tkinter.SEL, Tkinter.INSERT+'-%dc' % len(text), \
           Tkinter.INSERT)
        self.text.see(Tkinter.INSERT)

    def delete(self): 
        if not self.text.tag_ranges(Tkinter.SEL):
            tkMessageBox.showerror(NAME, 'No text selected')
        else: self.text.delete(SEL_FIRST, SEL_LAST)

    def selectAll(self):
        self.text.tag_add(Tkinter.SEL, '1.0', Tkinter.END+'-1c')
        self.text.mark_set(Tkinter.INSERT, '1.0')
        self.text.see(Tkinter.INSERT)

    # Window menu commands
    def next_win(self):
        pass
    def prev_win(self):
        pass
    def switch_prog(self,n):
        self.setFileName("%d.r2d3" % n)
    
    # Tools menu commands 

    def docInfo(self):
        text  = self.getAllText()
        bytes = len(text), 
        lines, words = len(text.split('\n')), len(text.split())
        index = self.text.index(Tkinter.INSERT)
        where = tuple(index.split('.'))
        m = 'Current location:\n File:\t%s\n Line:\t%s \n Col:\t%s\n\n' % \
               (str(self.current[0]), where[0], where[1])
        m += 'File text statistics:\n Bytes:\t%s\n Lines:\t%s\n Words:\t%s' % \
                (bytes[0], lines, words)
        new = Tkinter.Toplevel()
        text = Tkinter.Text(new, padx=5, wrap='none')
        text.master.title(NAME+' Information')
        text.pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=Tkinter.YES)
        text.config(font=self.font, bg='white', fg='black')
        text.insert(Tkinter.INSERT, m)
        text.config(state=Tkinter.DISABLED)
        text.focus()

    # Search menu commands

    def gotoLine(self):
        line = tkSimpleDialog.askinteger(NAME, 'Enter line number')
        self.text.update() 
        self.text.focus()
        if line is not None:
            maxindex = self.text.index(Tkinter.END+'-1c')
            maxline  = int(maxindex.split('.')[0])
            if (line > 0) and (line <= maxline):
                self.text.mark_set(Tkinter.INSERT, '%d.0' % line)
                self.text.tag_remove(Tkinter.SEL, '1.0', Tkinter.END)
                self.text.tag_add(Tkinter.SEL, Tkinter.INSERT, 'insert + 1l')
                self.text.see(Tkinter.INSERT)
            else: tkMessageBox.showerror(NAME, 'Bad line number')

    def findPrompt(self):
        new = Tkinter.Toplevel(self)
        new.title(NAME+' Find')
        Tkinter.Label(new, text='Find text:').grid(row=0, column=0)
        self.change1 = Tkinter.Entry(new)
        self.change1.grid(row=0, column=1, sticky=Tkinter.EW)
        Tkinter.Button(new, text='Find', default=Tkinter.ACTIVE, 
               command=self.onDoFind).grid(row=0, column=2, sticky=Tkinter.EW)
        Tkinter.Button(new, text='Close',  
               command=new.destroy).grid(row=1, column=2, sticky=Tkinter.EW)
        new.bind("<Return>", self.onDoFind)
        new.columnconfigure(1, weight=1)
        self.change1.focus()

    def find(self, lastkey=None, bust=None):
        key = lastkey
        self.text.update()
        self.text.focus()
        self.lastfind = key
        if key:
            where = self.text.search(key, Tkinter.INSERT, Tkinter.END)
            if not where: 
               if bust: raise 'bust'
               else: tkMessageBox.showerror(NAME, 'String not found')
            else:
                pastkey = where + '+%dc' % len(key)
                self.text.tag_remove(Tkinter.SEL, '1.0', Tkinter.END)
                self.text.tag_add(Tkinter.SEL, where, pastkey)
                self.text.mark_set(Tkinter.INSERT, pastkey)
                self.text.see(where)

    def findAgain(self): 
        self.find(self.lastfind)

    def replace(self): 
        new = Tkinter.Toplevel(self)
        Tkinter.Label(new, text='Find text:').grid(row=0, column=0)
        Tkinter.Label(new, text='Change to:').grid(row=1, column=0)
        self.change1 = Tkinter.Entry(new)
        self.change2 = Tkinter.Entry(new)
        self.change1.grid(row=0, column=1, sticky=Tkinter.EW)
        self.change2.grid(row=1, column=1, sticky=Tkinter.EW)
        Tkinter.Button(new, text='Find Next',  
               command=self.onDoFind).grid(row=0, column=2, sticky=Tkinter.EW)
        Tkinter.Button(new, text='Replace', 
               command=self.doReplace).grid(row=1, column=2, sticky=Tkinter.EW)
        Tkinter.Button(new, text='Replace All', 
               command=self.replaceAll).grid(row=2, column=2, sticky=Tkinter.EW)
        new.columnconfigure(1, weight=1)
        new.focus()

    def onDoFind(self, event=None):
        self.find(self.change1.get())

    def doReplace(self, bust=None):
        self.find(self.change1.get(), bust=bust)
        self.text.update()
        if self.text.tag_ranges(Tkinter.SEL): 
            self.text.delete(SEL_FIRST, SEL_LAST)
            self.text.insert(Tkinter.INSERT, self.change2.get())
            self.text.see(Tkinter.INSERT)

    def replaceAll(self): 
        try: 
            self.doReplace(1)
            self.replaceAll()
        except: pass

    # Others, useful outside this class

    def isEmpty(self):
        return not self.getAllText() 

    def getAllText(self):
        return self.text.get('1.0', Tkinter.END+'-1c')

    def setAllText(self, text):
        self.text.delete('1.0', Tkinter.END)
        self.text.insert(Tkinter.END, text)
        self.text.mark_set(Tkinter.INSERT, '1.0')
        self.text.see(Tkinter.INSERT)

    def clearAllText(self):
        self.text.delete('1.0', Tkinter.END)

    def getFileName(self):
        return self.currrent[0]

    def setFileName(self, name): 
        self.current = name
        self.master.title(NAME+' '+__version__+' - '+str(name[0]))

    def help(self):
        tkMessageBox.showinfo('About '+NAME, __doc__)

    def documentation(self): 
        m = """R2D3 Help
        
        R2D3 stands for the R2D3 Robotic Development 3nvironment
        
        This is a small built in editor that lets you write
        and save (upto 10) robotic programs.
        
        Click on a tab to edit the corresponding number program.
        Click on Save to save that program.
        After you are done with your programs editing, press DONE
        and the Editor will then exit.
        
        You can execute the programs using the simulator GUI
        Select the program number you want to execute and then click RUN
        
        Note: For help on the ACL programming, see the help files for R2D3.
        A brief overview is given here -
            
        """
        new = Tkinter.Toplevel()
        text = Tkinter.Text(new, padx=5, wrap='none')
        text.master.title(NAME+' Help')
        text.pack(side=Tkinter.TOP, fill=Tkinter.BOTH, expand=Tkinter.YES)
        text.config(font=self.font, bg='white', fg='black')
        text.insert(Tkinter.INSERT, m)
        text.config(state=Tkinter.DISABLED)
        text.focus()

def editor_main():
    global root
    if len(sys.argv) > 1: fname = sys.argv[1]
    else: fname = None
    if root == None: root = Tkinter.Tk()
    a = TextEditor(parent=root, loadFirst=fname)
    root.mainloop()
    root.withdraw()
    return

######################################
########   PARSER RUNTIME   ##########
######################################
# Yapps 2.0 Runtime
#
# This module is needed to run generated parsers.
from string import join, count, find, rfind
import re

class SyntaxError(Exception):
    """When we run into an unexpected token, this is the exception to use"""
    def __init__(self, pos=-1, msg="Bad Token"):
        Exception.__init__(self)
	self.pos = pos
	self.msg = msg
    def __repr__(self):
	if self.pos < 0: return "#<syntax-error>"
	else: return "SyntaxError[@ char %s: %s]" % (repr(self.pos), self.msg)

class NoMoreTokens(Exception):
    """Another exception object, for when we run out of tokens"""
    pass

class Scanner:
    def __init__(self, patterns, ignore, input):
	"""Patterns is [(terminal,regex)...]
        Ignore is [terminal,...];
	Input is a string"""
	self.tokens = []
	self.restrictions = []
	self.input = input
	self.pos = 0
	self.ignore = ignore
	# The stored patterns are a pair (compiled regex,source
	# regex).  If the patterns variable passed in to the
	# constructor is None, we assume that the class already has a
	# proper .patterns list constructed
        if patterns is not None:
            self.patterns = []
            for k, r in patterns:
                self.patterns.append( (k, re.compile(r)) )
	
    def token(self, i, restrict=0):
	"""Get the i'th token, and if i is one past the end, then scan
	for another token; restrict is a list of tokens that
	are allowed, or 0 for any token."""
	if i == len(self.tokens): self.scan(restrict)
	if i < len(self.tokens):
	    # Make sure the restriction is more restricted
	    if restrict and self.restrictions[i]:
		for r in restrict:
		    if r not in self.restrictions[i]:
			raise NotImplementedError("Unimplemented: restriction set changed")
	    return self.tokens[i]
	raise NoMoreTokens()
    
    def __repr__(self):
	"""Print the last 10 tokens that have been scanned in"""
	output = ''
	for t in self.tokens[-10:]:
	    output = '%s\n  (@%s)  %s  =  %s' % (output,t[0],t[2],repr(t[3]))
	return output
    
    def scan(self, restrict):
	"""Should scan another token and add it to the list, self.tokens,
	and add the restriction to self.restrictions"""
	# Keep looking for a token, ignoring any in self.ignore
	while 1:
	    # Search the patterns for the longest match, with earlier
	    # tokens in the list having preference
	    best_match = -1
	    best_pat = '(error)'
	    for p, regexp in self.patterns:
		# First check to see if we're ignoring this token
		if restrict and p not in restrict and p not in self.ignore:
		    continue
		m = regexp.match(self.input, self.pos)
		if m and len(m.group(0)) > best_match:
		    # We got a match that's better than the previous one
		    best_pat = p
		    best_match = len(m.group(0))
		    
	    # If we didn't find anything, raise an error
	    if best_pat == '(error)' and best_match < 0:
		msg = "Bad Token"
		if restrict:
		    msg = "Trying to find one of "+join(restrict,", ")
		raise SyntaxError(self.pos, msg)

	    # If we found something that isn't to be ignored, return it
	    if best_pat not in self.ignore:
		# Create a token with this data
		token = (self.pos, self.pos+best_match, best_pat,
			 self.input[self.pos:self.pos+best_match])
		self.pos = self.pos + best_match
		# Only add this token if it's not in the list
		# (to prevent looping)
		if not self.tokens or token != self.tokens[-1]:
		    self.tokens.append(token)
		    self.restrictions.append(restrict)
		return
	    else:
		# This token should be ignored ..
		self.pos = self.pos + best_match

class Parser:
    def __init__(self, scanner):
        self._scanner = scanner
        self._pos = 0
        
    def _peek(self, *types):
        """Returns the token type for lookahead; if there are any args
        then the list of args is the set of token types to allow"""
        tok = self._scanner.token(self._pos, types)
        return tok[2]
        
    def _scan(self, type):
        """Returns the matched text, and moves to the next token"""
        tok = self._scanner.token(self._pos, [type])
        if tok[2] != type:
            raise SyntaxError(tok[0], 'Trying to find '+type)
        self._pos = 1+self._pos
        return tok[3]



def print_error(input, err, scanner):
    """This is a really dumb long function to print error messages nicely."""
    p = err.pos
    # Figure out the line number
    line = count(input[:p], '\n')
    print err.msg+" on line "+repr(line+1)+":"
    # Now try printing part of the line
    text = input[max(p-80, 0):p+80]
    p = p - max(p-80, 0)

    # Strip to the left
    i = rfind(text[:p], '\n')
    j = rfind(text[:p], '\r')
    if i < 0 or (0 <= j < i): i = j
    if 0 <= i < p:
	p = p - i - 1
	text = text[i+1:]

    # Strip to the right
    i = find(text,'\n', p)
    j = find(text,'\r', p)
    if i < 0 or (0 <= j < i): i = j
    if i >= 0:
	text = text[:i]

    # Now shorten the text
    while len(text) > 70 and p > 60:
	# Cut off 10 chars
	text = "..." + text[10:]
	p = p - 7

    # Now print the string, along with an indicator
    print '> ',text
    print '> ',' '*p + '^'
    print 'List of nearby tokens:', scanner

def wrap_error_reporter(parser, rule):
    try:
        return_value = getattr(parser, rule)()
    except SyntaxError, s:
        input = parser._scanner.input
        try:
            print_error(input, s, parser._scanner)
        except ImportError:
            print 'Syntax Error',s.msg,'on line',1+count(input[:s.pos], '\n')
    except NoMoreTokens:
        print 'Could not complete parsing; stopped around here:'
        print parser._scanner
    return return_value
######################################
########   END OF RUNTIME   ##########
######################################

######################################
########  GENERATED PARSER  ##########
######################################
# This parser parses and executes the
# ACL program from the supplied text
# The parser is hand written as Yapps ACL
# Grammar could not be implemented in time

# Command Interpreter
# SUPPORTED COMMANDS:
# HERE POS
# HERER POS
# HERER POS1 POS2
# 
# TEACH POS
# TEACHR POS
# TEACHR POS1 POS2
# 
# SETPV POS
# SETPVC POS
# 
# MOVE POS
# 
# OPEN/CLOSE
def interpret(line):
    # The positions database
    global positions, cpositions
    # The current joint coordinates
    global t1,t2,t3,t4,t5
    # The current linenumber
    global lineno
    lineno = lineno+1
    # The link lengths
    global a1,a2,a3,a5
    # Get the current cartesian coordinates
    global ee
    x = ee.getLocation()[0]*200
    y = ee.getLocation()[1]*200
    z = ee.getLocation()[2]*200
    o = (t2+t3+t4)*180/pi
    p = t5
    # Grip control
    global grip_is_open, objpick
    # The ACL command patterns
    patts = ['\s*here\s+(\w+)',         #1
            '\s*herer\s+(\w+)',         #2
            '\s*herer\s+(\w+)\s+(\w+)', #3
            '\s*teach\s+(\w+)',         #4
            '\s*teachr\s+(\w+)',        #5
            '\s*teachr\s+(\w+)\s+(\w+)',#6
            '\s*setpv\s+(\w+)',         #7
            '\s*setpvc\s+(\w+)',        #8
            '\s*move\s+(\w+)',          #9
            '\s*open',                  #10
            '\s*close',                 #11
            '\s*listpv position',       #12
            '\s*home',                  #13
            '.*']
    i = 0
    for patt in patts:
        i = i+1
        m = re.match(patt,line)
        if not m:
#            print "Match not found for",patt
            continue
        poses = m.groups()
        # HERE POS
        if i==1:
            print "Line %d: interpreted as HERE POS"%lineno
            print "Adding position %s:" %poses[0],t1,t2,t3,t4,t5
            positions[poses[0]]=[t1,t2,t3,t4,t5]
            return
        # HERER POS t1 t2 t3 t4 t5
        elif i==2:
            print "Line %d: interpreted as HERER POS t1 t2 t3 t4 t5"%lineno
            print m.group()
            line = line[len(m.group()):]
            print line
            m = re.match('\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)', line)
            if not m:
                Draw.PupMenu("LINE:%d SYNTAX ERROR!!! Arguments not specified fully" % lineno)
                return
            print m.groups()
            thetas = map(lambda x:int(x)*pi/180.0, m.groups())
            thetas = [thetas[0]+t1,thetas[1]+t2,thetas[2]+t3,thetas[3]+t4,thetas[4]+t5]
            print "Adding position %s:" %poses[0],thetas
            positions[poses[0]]=thetas
            return
        # HERER POS1 POS2 t1 t2 t3 t4 t5
        elif i==3:
            line = line[len(m.group()):]
            m = re.match('\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)', line)
            if not m:
                Draw.PupMenu("LINE:%d SYNTAX ERROR!!! Arguments not specified fully" % lineno)
                return
            thetas = map(lambda x:int(x)*pi/180.0, m.groups())
            p = position[poses[1]]
            for j in range(len(thetas)):
                thetas[j] = thetas[j]+p[j]
            print "Adding position %s:" %poses[0],thetas
            positions[poses[0]]=thetas
            return
        # TEACH POS
        elif i==4:
            print "Line %d: interpreted as TEACH POS"%lineno
            print "Adding position %s:" %poses[0],x,y,z,o,p
            cpositions[poses[0]]=[x,y,z,o,p]
            return
        # TEACHR POS x y z o p
        elif i==5:
            print "Line %d: interpreted as TEACHR POS x y z o p"%lineno
            line = line[len(m.group()):]
            m = re.match('\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)', line)
            if not m:
                Draw.PupMenu("LINE:%d SYNTAX ERROR!!! Not enough arguments given" % lineno)
                return
            xyzop = map(lambda x:int(x)/200, m.groups())
            xyzop = [x+xyzop[0],y+xyzop[1],z+xyzop[2],o+xyzop[3],p+xyzop[4]]
            #xyzop = invkine_helper(thetas[0]+x,thetas[1]+y,thetas[2]+z,thetas[3]+o).next()
            print "Adding position %s:" %poses[0],xyzop
            cpositions[poses[0]]=[xyzop[0],xyzop[1],xyzop[2],xyzop[3],xyzop[4]]
            return
        # TEACHR POS1 POS2 t1 t2 t3 t4 t5
        elif i==6:
            print "Line %d: interpreted as TEACHR POS1 POS2 x y z o p"%lineno
            line = line[len(m.group()):]
            m = re.match('\s*(\d+)\s*(\d+)\s*(\d+)\s*(\d+)\s*(\d+)', line)
            if not m:
                Draw.PupMenu("LINE:%d SYNTAX ERROR!!! Not enough arguments given" % lineno)
                return
            xyzop = map(lambda x:int(x)/200, m.groups())
            if not cpositions.has_key(poses[1]):
                Draw.PupMenu("LINE:%d SYNTAX ERROR!!! Specified position does not exist" % lineno)
                return
            p = cpositions[poses[1]]
            for j in range(len(xyzop)):
                xyzop[j] = xyzop[j]+p[j]
            print "Adding position %s:" %poses[0],xyzop
            cpositions[poses[0]]=[xyzop[0],xyzop[1],xyzop[2],xyzop[3],xyzop[4]]
            return
        # SETPV POS
        elif i==7:
            line = line[len(m.group()):]
            m = re.match('\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)', line)
            if not m:
                Draw.PupMenu("LINE:%d SYNTAX ERROR!!! Not enough arguments given" % lineno)
                return
            thetas = map(lambda x:int(x)*pi/180, m.groups())
            positions[poses[0]]=[thetas[0],thetas[1],thetas[2],thetas[3],thetas[4]]
            return
        # SETPVC POS
        elif i==8:
            line = line[len(m.group()):]
            m = re.match('\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)\s*(-?\d+)', line)
            if not m:
                Draw.PupMenu("LINE:%d SYNTAX ERROR!!! Not enough arguments given" % lineno)
                return
            xyzop = map(lambda x:int(x)/200.0, m.groups())
            cpositions[poses[0]]=[xyzop[0],xyzop[1],xyzop[2],xyzop[3],xyzop[4]]
            return
        # MOVE POS
        elif i==9:
            print "Line %d: interpreted as MOVE POS"%lineno
            p = poses[0]
            if positions.has_key(p):
                print "Moving to joint position",p
                p = positions[p]
                run_anim(p)
            elif cpositions.has_key(p):
                print "Moving to cartesian position",p
                p = cpositions[p]
                run_anim(p)
            else:
                print "LINE:%d SYNTAX ERROR!!! Position does not exist" % lineno
                Draw.PupMenu("LINE:%d SYNTAX ERROR!!! Position does not exist" % lineno)
            return
        # OPEN
        elif i==10:
            global grip_is_open
            grip_is_open = 1
            if objpick:
                try_drop_billet()
            fwdkine()
            return
        # CLOSE
        elif i==11:
            global grip_is_open
            grip_is_open = 0
            if not objpick:
                try_pickup_billet()
            fwdkine()
            return
        # LISTPV POSITION
        elif i==12:
            Draw.PupMenu("Joint coordinates: %f %f %f %f %f" %(t1,t2,t3,t4,t5))
            Draw.PupMenu("Cartersian coordinates: %f %f %f %f %f" %(x,y,z,o,p))
            return
        # HOME
        elif i==13:
            run_anim([0,-120*pi/180,90*pi/180,100*pi/180,0])
            return
        # DEFAULT PATTERN
        elif patt=='.*':
            Draw.PupMenu("LINE:%d SYNTAX ERROR!!! COMMAND not recognised" % lineno)
        return

# Run the program
def run_prog():
    global program, lineno
    lineno = 0
    lines = program.split("\n")
    # Interpret each line
    for line in lines:
        print "Interpreting line: "+line
        interpret(line)

######################################
########      GUI CODE      ##########
######################################

def draw():
    # Inverse kine globals
    global EVENT_SLIDER_X
    global EVENT_TEXT_2
    global EVENT_TEXT_3
    global EVENT_SLIDER_Y
    global EVENT_SLIDER_Z
    global EVENT_SLIDER_ORIENTATION
    global EVENT_BUTTON_SEARCH
    global EVENT_BUTTON_SOLVE
    global EVENT_BUTTON_SOL1
    global EVENT_BUTTON_SOL2
    global EVENT_BUTTON_CURSOR
    global Object_Slider_X
    global Object_Text_2
    global Object_Text_3
    global Object_Slider_Y
    global Object_Slider_Z
    global Object_Slider_Orientation
    global Object_Button_Search
    global Object_Button_Solve
    global Object_Button_Sol1
    global Object_Button_Sol2
    global Object_Button_Cursor
    # Forward kine globals
    global EVENT_SLIDER_BASE
    global EVENT_SLIDER_SHOULDER
    global EVENT_SLIDER_ELBOW
    global EVENT_SLIDER_WRIST
    global EVENT_SLIDER_EE
    global EVENT_BUTTON_GRIPPEROPEN
    global EVENT_BUTTON_GRIPPERCLOSE
    global EVENT_TEXT_7
    global EVENT_TEXT_8
    global Object_Slider_Base
    global Object_Slider_Shoulder
    global Object_Slider_Elbow
    global Object_Slider_Wrist
    global Object_Slider_EE
    global Object_Button_GripperOpen
    global Object_Button_GripperClose
    global Object_Text_7
    global Object_Text_8
    # (Server + Home + Select) globals
    global EVENT_BUTTON_STARTSERVER
    global EVENT_BUTTON_STOPSERVER
    global EVENT_SLIDER_SELECT
    global EVENT_BUTTON_HOME
    global Object_Button_StartServer
    global Object_Button_StopServer
    global Object_Slider_Select
    global Object_Button_Home
    global Object_Text_9
    # Teach Position globals
    global Object_Text_10
    global EVENT_STRING_POSITION
    global EVENT_BUTTON_TEACH
    global EVENT_BUTTON_MOVE
    global Object_String_Position
    global Object_Button_Teach
    global Object_Button_Move

    BGL.glClearColor(0.9,0.9,0.9, 0.0) #earlier 0,0,0.844484,0
    BGL.glClear(GL_COLOR_BUFFER_BIT)

    BGL.glColor3f(0.000000, 0.000000, 0.000000)

    BGL.glRasterPos2i(10, 670)
    Object_Text_3 = Draw.Text("R2D3 Robotic Simulator by Anupam Jain")

    # Inverse Kine
    BGL.glRasterPos2i(10, 650)
    Object_Text_2 = Draw.Text("Inverse Kinematics ------------")
    Object_Slider_X = Draw.Slider("X Loc: ", EVENT_SLIDER_X, 10, 620, 200, 20, Object_Slider_X.val, -587.000000, 587.000000, 1, "X location of the End Effector")
    Object_Slider_Y = Draw.Slider("Y Loc: ", EVENT_SLIDER_Y, 10, 595, 200, 20, Object_Slider_Y.val, -587.000000, 587.000000, 1, "Y Location of the End Effector")
    Object_Slider_Z = Draw.Slider("Z Loc: ", EVENT_SLIDER_Z, 10, 570, 200, 20, Object_Slider_Z.val, -936.000000, 936.000000, 1, "Z Location of the End Effector")
    Object_Slider_Orientation = Draw.Slider("Angle: ", EVENT_SLIDER_ORIENTATION, 10, 545, 200, 20, Object_Slider_Orientation.val, -360.000000, 360.000000, 1, "The Orientation angle is the angle about the Shoulder Z axis of the end effector position")
    Object_Button_Cursor = Draw.Button("Get Cursor Pos", EVENT_BUTTON_CURSOR, 10, 510, 200, 20, "Fill in the X Y and Z values from the 3D cursor position")
    Object_Button_Search = Draw.Button("Search", EVENT_BUTTON_SEARCH, 10, 485, 200, 20, "Searches for an inverse kinematics solution by hill-climbing")
    Object_Button_Solve = Draw.Button("Solve", EVENT_BUTTON_SOLVE, 10, 460, 200, 20, "Solves for the inverse kinematics solution for the target position")
    Object_Button_Sol1 = Draw.Button("<<<<", EVENT_BUTTON_SOL1, 10, 435, 80, 20, "Get Previous Solution")
    Object_Button_Sol2 = Draw.Button(">>>>", EVENT_BUTTON_SOL2, 130, 435, 80, 20, "Get Next Solution")

    # Forward kine
    BGL.glRasterPos2i(10, 395)
    Object_Text_7 = Draw.Text("Forward Kinematics -----------")
    Object_Slider_Base = Draw.Slider("Base: ", EVENT_SLIDER_BASE, 10, 365, 200, 20, Object_Slider_Base.val, 0.000000, 310.000000, 1, "Rotates the Base")
    Object_Slider_Shoulder = Draw.Slider("Shoulder: ", EVENT_SLIDER_SHOULDER, 10, 340, 200, 20, Object_Slider_Shoulder.val, -130.000000, 35.000000, 1, "Rotates the Shoulder")
    Object_Slider_Elbow = Draw.Slider("Elbow: ", EVENT_SLIDER_ELBOW, 10, 315, 200, 20, Object_Slider_Elbow.val, -130.000000, 130.000000, 1, "Rotates the Elbow")
    Object_Slider_Wrist = Draw.Slider("Wrist Rot: ", EVENT_SLIDER_WRIST, 10, 290, 200, 20, Object_Slider_Wrist.val, -130.000000, 130.000000, 1, "Rotates the Wrist")
    Object_Slider_EE = Draw.Slider("Wrist Roll: ", EVENT_SLIDER_EE, 10, 265, 200, 20, Object_Slider_EE.val, 0.000000, 570.000000, 1, "Rolls the Elbow")
    Object_Button_GripperOpen = Draw.Button("Gripper Open", EVENT_BUTTON_GRIPPEROPEN, 10, 240, 90, 20, "Open the Gripper")
    Object_Button_GripperClose = Draw.Button("Gripper Close", EVENT_BUTTON_GRIPPERCLOSE, 120, 240, 90, 20, "Close the Gripper")
    
    # (Server + Home + Select)
    BGL.glRasterPos2i(10, 205)
    Object_Text_9 = Draw.Text("Program Editor---------------")
    Object_Slider_Select = Draw.Slider("Select Prog: ", EVENT_SLIDER_SELECT, 10, 175, 200, 20, Object_Slider_Select.val, 1, 9, 1, "Select the program to run")
    Object_Button_Home = Draw.Button("Home", EVENT_BUTTON_HOME, 10, 150, 200, 20, "Starts the robot homing sequence")
    Object_Button_StartServer = Draw.Button("Start Editor", EVENT_BUTTON_STARTSERVER, 10, 125, 90, 20, "Starts the Scorbot ACL Editor.")
    Object_Button_StopServer = Draw.Button("RUN", EVENT_BUTTON_STOPSERVER, 120, 125, 90, 20, "Runs the edited program")
    
    # Teach + Move Positions
    BGL.glRasterPos2i(10, 90)
    Object_Text_10 = Draw.Text("Teach Positions --------------")
    Object_String_Position = Draw.String("Position Name: ", EVENT_STRING_POSITION, 10, 60, 200, 20,Object_String_Position.val, 10, "Enter the position name here")
    Object_Button_Teach = Draw.Button("Teach", EVENT_BUTTON_TEACH, 10, 35, 90, 20, "Teach the robot the current position by the name specified")
    Object_Button_Move = Draw.Button("Move", EVENT_BUTTON_MOVE, 120, 35, 90, 20, "Move to the specified position")
    
######################################
#######   EVENT HANDLERS     #########
######################################

def event(event, value):
    if event == Draw.ESCKEY and not value: Draw.Exit()

def b_event(event):
    # Inverse kine globals
    global EVENT_SLIDER_X
    global EVENT_TEXT_2
    global EVENT_TEXT_3
    global EVENT_SLIDER_Y
    global EVENT_SLIDER_Z
    global EVENT_SLIDER_ORIENTATION
    global EVENT_BUTTON_SEARCH
    global EVENT_BUTTON_SOLVE
    global EVENT_BUTTON_SOL1
    global EVENT_BUTTON_SOL2
    global EVENT_BUTTON_CURSOR
    global Object_Slider_X
    global Object_Text_2
    global Object_Text_3
    global Object_Slider_Y
    global Object_Slider_Z
    global Object_Slider_Orientation
    global Object_Button_Search
    global Object_Button_Solve
    global Object_Button_Sol1
    global Object_Button_Sol2
    global Object_Button_Cursor
    # Forward kine globals
    global EVENT_SLIDER_BASE
    global EVENT_SLIDER_SHOULDER
    global EVENT_SLIDER_ELBOW
    global EVENT_SLIDER_WRIST
    global EVENT_SLIDER_EE
    global EVENT_BUTTON_GRIPPEROPEN
    global EVENT_BUTTON_GRIPPERCLOSE
    global EVENT_TEXT_7
    global EVENT_TEXT_8
    global Object_Slider_Base
    global Object_Slider_Shoulder
    global Object_Slider_Elbow
    global Object_Slider_Wrist
    global Object_Slider_EE
    global Object_Button_GripperOpen
    global Object_Button_GripperClose
    global Object_Text_7
    global Object_Text_8
    # (Server + Home + Select) globals
    global EVENT_BUTTON_STARTSERVER
    global EVENT_BUTTON_STOPSERVER
    global EVENT_SLIDER_SELECT
    global EVENT_BUTTON_HOME
    global Object_Button_StartServer
    global Object_Button_StopServer
    global Object_Slider_Select
    global Object_Button_Home
    global Object_Text_9
    # Teach Position globals
    global Object_Text_10
    global EVENT_STRING_POSITION
    global EVENT_BUTTON_TEACH
    global EVENT_BUTTON_MOVE
    global Object_String_Position
    global Object_Button_Teach
    global Object_Button_Move

    # Other globals
    global x,y,z,angle
    global t1,t2,t3,t4
    global theta1,theta2,theta3,theta4
    global t1, t2, t3, t4, t5, grip, grip_is_open
    global solution

    if event == 0: pass
    # Inverse kine
    elif event == EVENT_SLIDER_X:
        x = Object_Slider_X.val/200.0
        target.setLocation(x,y,z)
#        myinvkine(getTarget())
        invkine()
    elif event == EVENT_SLIDER_Y:
        y = Object_Slider_Y.val/200.0
        target.setLocation(x,y,z)
#        myinvkine(getTarget())
        invkine()
    elif event == EVENT_SLIDER_Z:
        z = Object_Slider_Z.val/200.0
        target.setLocation(x,y,z)
#        myinvkine(getTarget())
        invkine()
    elif event == EVENT_SLIDER_ORIENTATION:
        angle = Object_Slider_Orientation.val*pi/180
        invkine()
    elif event == EVENT_BUTTON_SEARCH:
        myinvkine(getTarget())
        Draw.PupMenu("Searching finished")
    elif event == EVENT_BUTTON_SOLVE:
        invkine()
    elif event == EVENT_BUTTON_SOL1:
        solution = 0
        # Update the t* values to show the appropriate solution
        t1,t2,t3,t4 = theta1[solution],theta2[solution],theta3[solution],theta4[solution]
        fwdkine()
    elif event == EVENT_BUTTON_SOL2:
        solution = 1
        # Update the t* values to show the appropriate solution
        t1,t2,t3,t4 = theta1[solution],theta2[solution],theta3[solution],theta4[solution]
        fwdkine()
    elif event == EVENT_BUTTON_CURSOR:
        x,y,z = Blender.Window.GetCursorPos()
        Object_Slider_X.val = x*200
        Object_Slider_Y.val = y*200
        Object_Slider_Z.val = z*200
        x = Object_Slider_X.val/200.0
        y = Object_Slider_Y.val/200.0
        z = Object_Slider_Z.val/200.0
        target.setLocation(x,y,z)
    # Forward kine
    elif event == EVENT_SLIDER_BASE:
        t1 = Object_Slider_Base.val*pi/180
        fwdkine()
    elif event == EVENT_SLIDER_SHOULDER:
        t2 = Object_Slider_Shoulder.val*pi/180
        fwdkine()
    elif event == EVENT_SLIDER_ELBOW:
        t3 = Object_Slider_Elbow.val*pi/180
        fwdkine()
    elif event == EVENT_SLIDER_WRIST:
        t4 = Object_Slider_Wrist.val*pi/180
        fwdkine()
    elif event == EVENT_SLIDER_EE:
        t5 = Object_Slider_EE.val*pi/180
        fwdkine()
    elif event == EVENT_BUTTON_GRIPPEROPEN:
        grip_is_open = 1
        if objpick:
            try_drop_billet()
        fwdkine()
    elif event == EVENT_BUTTON_GRIPPERCLOSE:
        print "GRIPPER CLOSING!!!!"
        grip_is_open = 0
        if not objpick:
            try_pickup_billet()
        fwdkine()
    elif event == EVENT_TEXT_7:
        pass
    elif event == EVENT_TEXT_8:
        pass
    # (Server + Home + Select)
    # Button to start the editor
    elif event == EVENT_BUTTON_STARTSERVER:
        editor_main()
    # Button to run the program
    elif event == EVENT_BUTTON_STOPSERVER:
        run_prog()
    # Button to select the program to run
    elif event == EVENT_SLIDER_SELECT:
        print Object_Slider_Select.val
        for i in range(10):
            billets[i].select(0)
        b = billets[Object_Slider_Select.val-1]
        b.select(1)
        b = b.getLocation() # Reusing variable names (Shame on me)
        Blender.Window.SetCursorPos(b[0],b[1],b[2])
#        print dir(b)
    elif event == EVENT_BUTTON_HOME:
        run_anim([0,-120*pi/180,90*pi/180,100*pi/180,0])
    # Teach Positions
    elif event == EVENT_STRING_POSITION:
        pass
    elif event == EVENT_BUTTON_TEACH:
        Draw.PupMenu("Adding position %s: %f %f %f %f %f" %(Object_String_Position.val,t1,t2,t3,t4,t5))
        positions[Object_String_Position.val]=[t1,t2,t3,t4,t5]
    elif event == EVENT_BUTTON_MOVE:
        p = Object_String_Position.val
        if positions.has_key(p):
            p = positions[p]
            run_anim(p)
        elif cpositions.has_key(p):
            p = cpositions[p]
            run_anim(p)
        else:
            Draw.PupMenu("ERROR!!! Position %s does not exist" % Object_String_Position.val)
        return
    Draw.Draw()
    Blender.Redraw()

######################################
#####  REGISTER THE GUI CODE  ########
######################################
Draw.Register(draw, event, b_event)