'''
Created on Mar 8, 2013

@author: ahmed
'''
import math
class Point(object):
    def __init__(self, x, y):
        self.X = x
        self.Y = y
    def __repr__(self):
        return self.__str__()
    def __eq__(self, other):
        return other.X == self.X and other.Y == self.Y
    @staticmethod
    def distantTo(first, second):
        dx =float( second.X - first.X)
        dy =float( second.Y - first.Y)
        return math.sqrt(dx * dx + dy * dy)
    @staticmethod
    def thetaTo(first, second):
        dx =float( second.X - first.X)
        dy =float( second.Y - first.Y)
        theta = -1
        if dx == 0 and dy != 0:
            theta = math.pi / 2
        elif dx == 0:
            theta = 0
        else: 
            theta = math.atan(abs(dy) / abs(dx))
        if dx > 0 and dy > 0:
             return theta
        if dx < 0 and dy > 0:
             return math.pi - theta
        if dx > 0 and dy < 0:
            return 2 * math.pi - theta
        return math.pi + theta
    def __str__(self):
        return '('+str(self.X)+','+str(self.Y)+')'
    @staticmethod
    def tranc(p):
        return Point(int(p.X),int(p.Y))
m = Point(0, 0.5)
m2 = Point(0, 0)
d = Point.thetaTo(m, m2)
print m.X

            
