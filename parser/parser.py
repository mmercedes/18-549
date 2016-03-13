#!/usr/bin/python
import sys
from xml.dom import minidom
from svg.path import Path, Line, Arc, CubicBezier, QuadraticBezier, parse_path


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
    (width, height) =  (svg[0].getAttribute('width'), svg[0].getAttribute('height'))
    path_strings = [path.getAttribute('d') for path in doc.getElementsByTagName('path')]
    paths = []
    for path_string in path_strings:
        paths.append(parse_path(path_string))
    doc.unlink()
    
    for path in paths:
        print "PATH START"
        for segment in path:
            print parseSegment(segment)
        print "PATH END"

def parseSegment(segment):
    if isinstance(segment, Line):
        (start, end) = (segment.start, segment.end)
        return "LINE (%f, %f) (%f, %f)" % (start.real, start.imag, end.real, end.imag)
    if isinstance(segment, CubicBezier):
        (start, c1, c2, end) = (segment.start, segment.control1, segment.control2, segment.end)
        data = "CURVE (%f, %f) (%f, %f) " % (start.real, start.imag, c1.real, c1.imag)
        data += "(%f, %f) (%f, %f)" % (c2.real, c2.imag, end.real, end.imag)
        return data
    else:
        return str(type(segment))
    
if __name__ == "__main__":
    main()

    
'''
- How to handle the fill of each path?
- Should we convert curves to lines or handle directly?
'''
