"""Synthetic S bend geometry; only the simulator knows this map."""
import math
END=2.6
def center(x):
    return .12*(1-math.cos(2*math.pi*(x-.6)/1.8)) if .6<x<2.4 else 0.
def slope(x):
    return .12*2*math.pi/1.8*math.sin(2*math.pi*(x-.6)/1.8) if .6<x<2.4 else 0.
def boundary(x,side):
    a=math.atan(slope(x));return x-side*.1*math.sin(a),center(x)+side*.1*math.cos(a)
def obstacle_segments():
    return [(.48,-.055,.59,-.055),(.59,-.055,.59,.055),(.59,.055,.48,.055),(.48,.055,.48,-.055)]
