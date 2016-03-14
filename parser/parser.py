#!/usr/bin/python
import sys
from xml.dom import minidom
from svg.path import Path, Line, Arc, CubicBezier, QuadraticBezier, parse_path

DRAWING_WIDTH = 1000.0
DRAWING_HEIGHT = 1000.0

def main():
    if len(sys.argv) == 0:
        print("No input files given")
        return
    filename = sys.argv[1]
    parse(filename)

        
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
    
    for path in paths:
        print "PATH START"
        for segment in path:
            print parseSegment(segment, width, height)
        print "PATH END"

def parseSegment(segment, width, height):
    if width > height:
        scale = DRAWING_WIDTH/width
    else:
        scale = DRAWING_HEIGHT/height
    if isinstance(segment, Line):
        (start, end) = (segment.start, segment.end)
        (sx, sy) = (start.real*scale, start.imag*scale)
        (ex, ey) = (end.real*scale, end.imag*scale)
        return "LINE (%f, %f) (%f, %f)" % (sx, sy, ex, ey)
    if isinstance(segment, CubicBezier):
        (start, c1, c2, end) = (segment.start, segment.control1, segment.control2, segment.end)
        (sx, sy, c1x, c1y) = (start.real*scale, start.imag*scale, c1.real*scale, c1.imag*scale)
        (c2x, c2y, ex, ey) = (c2.real*scale, c2.imag*scale, end.real*scale, end.imag*scale)
        data = "CURVE (%f, %f) (%f, %f) " % (sx, sy, c1x, c1y)
        data += "(%f, %f) (%f, %f)" % (c2x, c2y, ex, ey)
        return data
    else:
        return str(type(segment))
    
if __name__ == "__main__":
    main()

    
'''
- How to handle the fill of each path?
- Should we convert curves to lines or handle directly?
'''
