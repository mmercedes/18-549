#import smbus
import time
import sys
from xml.dom import minidom
from svg.path import Path, Line, Arc, CubicBezier, QuadraticBezier, parse_path
import turtle
import math

DRAWING_WIDTH = 1000.0
DRAWING_HEIGHT = 1000.0

STOP = 0
PEN_UP = 1
PEN_DOWN = 2
CALIBRATE = 3 
DRAW = 4
# for RPI version 1, use "bus = smbus.SMBus(0)"

# bus = smbus.SMBus(1)

# # This is the address we setup in the Arduino Program
# address = 0x04

#!/usr/bin/python

def parser(filename):
    parsedList = parse(filename)
    x = convertToSteps(parsedList)
    y = x[0]
    del y[0:6]
    x[0] = y
    return x
    #print parsedList
    #drawImage(parsedList)
     

        
def parse(filename):
    doc = minidom.parse(filename)
    svg = doc.getElementsByTagName('svg')
    if len(svg) == 0:
        print 'No SVG data found in file'
        return
    width = svg[0].getAttribute('width')
    height = svg[0].getAttribute('height')
    print width, height
    if width is "" or height is "":
        width = DRAWING_WIDTH
        height = DRAWING_HEIGHT
        # https://developer.mozilla.org/en-US/docs/Web/SVG/Attribute/viewBox
        viewbox = svg[0].getAttribute('viewBox')
        print 'VIEWBOX ' + viewbox
        if viewbox is not "":
            viewbox = viewbox.split()
            if len(viewbox) == 4:
                width = float(viewbox[2])
                height = float(viewbox[3])
    else:
        width = float(width)
        height = float(height)
    print width, height
    path_strings = [path.getAttribute('d') for path in doc.getElementsByTagName('path')]
    paths = []
    for path_string in path_strings:
        paths.append(parse_path(path_string))
    doc.unlink()
    
    x =[]
    for path in paths:
        pathList = []
        #print "PATH START"
        for segment in path:
            temp = parseSegment(segment, width, height)
           # print temp
            pathList = pathList + temp
        #print "PATH END"
        x.append(pathList)
    return x
        #go through path adding points to a list once path is done add that to another list

def parseSegment(segment, width, height):
    if width > height:
        scale = DRAWING_WIDTH/width
    else:
        scale = DRAWING_HEIGHT/height 
    if isinstance(segment, Line):
        (start, end) = (segment.start, segment.end)
        (sx, sy) = (start.real*scale, start.imag*scale)
        (ex, ey) = (end.real*scale, end.imag*scale)
        return [[sx, sy]] #"LINE (%f, %f) (%f, %f)" % (sx, sy, ex, ey)
    if isinstance(segment, CubicBezier):
        (start, c1, c2, end) = (segment.start, segment.control1, segment.control2, segment.end)
        (sx, sy, c1x, c1y) = (start.real*scale, start.imag*scale, c1.real*scale, c1.imag*scale)
        (c2x, c2y, ex, ey) = (c2.real*scale, c2.imag*scale, end.real*scale, end.imag*scale)
        #data = "CURVE (%f, %f) (%f, %f) " % (sx, sy, c1x, c1y)
        #data += "(%f, %f) (%f, %f)" % (c2x, c2y, ex, ey)
        data = getlines(sx, sy, c1x, c1y, c2x, c2y, ex, ey, 100)
        return data
    else:
        return str(type(segment))


def convertToSteps(paths):
    spaths = [] 
    for path in paths:
        temp = []
        for point in path:
            y = point[1]
            x = point[0]
            temp.append(calcNewDist(x,y))
        spaths.append(temp)
    return spaths


def drawImage(paths):
    myTurtle = turtle.Turtle(shape="turtle")
    turtle.screensize(2000,2000)
    myTurtle.pendown()
    myTurtle.penup()
    myTurtle.setposition(0, 0)
    y = 0
    for path in paths:
        myTurtle.penup()
        temp = path[0]
        myTurtle.setposition(temp[0], temp[1])
        myTurtle.pendown()
        y = 1
        for x in paths:
            for y in x:
                myTurtle.setposition(y[0], y[1])
        myTurtle.penup()
    turtle.getscreen()._root.mainloop()

def getpoint(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, mu):
    n1 = 0
    n13 = 0
    mu3 = 0
    p = 0

    n1 = 1 - mu
    n13 = n1 * n1 * n1
    mu3 = mu * mu * mu
    px = n13*p1x + 3*mu*n1*n1*p2x + 3*mu*mu*n1*p3x + mu3*p4x;
    py = n13*p1y + 3*mu*n1*n1*p2y + 3*mu*mu*n1*p3y + mu3*p4y;

    return ([px,py])

def getlines(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, x):
    z = 0
    c = 1.00 / x
    z = c
    points = []
    print z
    while (z < 1):
        points.append(getpoint(p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y, z))
        z = z + c
    return points
    #drawcurve(points)
    #flattened = [val for sublist in points for val in sublist]
    #print flattened

def drawcurve(points):
    myTurtle = turtle.Turtle(shape="turtle")
    turtle.screensize(500,500)
    turtle.setworldcoordinates(400,400,500,500)
    myTurtle.penup()
    y = points[0]
    myTurtle.setposition(y[0], y[1])
    myTurtle.pendown()
    for x in points:
        myTurtle.setposition(x[0], x[1])
    turtle.getscreen()._root.mainloop()

def calcNewDist(nx,ny):
    nx1 = 1000.0 - nx
    ny1 = 1000.0 - ny
    nh1 = math.sqrt((nx**2) + (ny**2))
    nh2 = math.sqrt((nx1**2) + (ny**2))
    nh3 = math.sqrt((nx**2) + (ny1**2))
    nh4 = math.sqrt((nx1**2) + (ny1**2))
    return[nh1,nh2,nh3,nh4]
    

# def writeNumber(a1,a2,b1,b2,c1,c2,d1,d2,s):
#         bus.write_i2c_block_data(address,a1, [a2,b1,b2,c1,c2,d1,d2,s])
#         # bus.write_byte_data(address, 0, value)
#         return -1

# def readNumber():
#         number = bus.read_byte(address)
#         # number = bus.read_byte_data(address, 1)
#         return number

def init():
        var = raw_input("Enter a Command or help: ")

        if (var == "STOP"):
                writeNumber(0,0,0,0,0,0,0,0,0)
        elif (var == "PEN UP"):
                writeNumber(0,0,0,0,0,0,0,0,1)
        elif (var == "PEN DOWN"):
                writeNumber(0,0,0,0,0,0,0,0,2)
        elif (var == "CALIBRATE"):
                writeNumber(0,0,0,0,0,0,0,0,3)
        elif (var == "DRAW"):
                var2 = raw_input("Which File do you want to Draw?")
                steps = parser(var2)
                print steps
                draw(steps)

def draw(steps):
    try: 
        for path in steps:
            for stuff in path:
                s1 = tb(stuff[0])
                s2 = tb(stuff[1])
                s3 = tb(stuff[2])
                s4 = tb(stuff[3])
                #writeNumber(s1[0],s1[1],s2[0],s2[1],s3[0],s3[1],s4[0],s4[1],4)
                print "RPI: Hi Arduino, I sent you "
                # sleep one second
        time.sleep(1)
        while True:

            #number = readNumber()
            print "Arduino: Hey RPI, I received a digit ", number
            print
            #if (number == 0):
            #    draw()
    except KeyboardInterrupt:
        init()

def tb(x):
    x = int(x)
    t = [0,0]
    t[0] = x % 256
    t[1] = (x / 256)
    return t

init()