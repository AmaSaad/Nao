'''
Created on Mar 8, 2013

@author: ahmed
'''
from DataStructure import Point
import math
class robot(object):
    
    def __init__(self,CurrentPos,Theta,Maxdisp,Scale):
        self.currentPos=CurrentPos
        self.currentTheta=Theta
        self.maxdisp=Maxdisp
        self.scale=Scale
        self.rotationTheta=0
        self.dst=None
        self.rotationDone=False
        self.distance=0
    def moveTo(self,dest):
        self.dst = dest;
        self.rotationDone = False
        oldTheta = self.currentTheta
        self.currentTheta = Point.Point.thetaTo(self.currentPos, dest)
        self.rotationTheta = self.currentTheta - oldTheta;
        self.distance = Point.Point.distantTo(self.currentPos, dest)
    def executeStep(self):
            if not self.rotationDone:
                print'>> rotate ' ,((self.rotationTheta / math.pi) * 180.0) , ' degree'
                self.rotationTheta = 0;
                self.rotationDone = True;
                return True
            if self.distance * self.scale <= self.maxdisp and self.distance * self.scale != 0:
                pos=Point.Point.tranc(self.dst)
                print'>> move ' +str( Point.Point.distantTo(self.currentPos, self.dst) * self.scale)+ ' m , current Pos=' +str(pos)
                self.currentPos = self.dst;
                self.dst = None;
                self.distance = 0;
                return True;
            elif self.distance * self.scale > self.maxdisp:
                dis = (self.maxdisp / self.scale) ;
                newcurrentPos = Point.Point((self.currentPos.X + dis * math.cos(self.currentTheta)), (self.currentPos.Y + dis * math.sin(self.currentTheta)))
                tempD = Point.Point.distantTo(self.currentPos, newcurrentPos)
                self.distance -= tempD;
                self.currentPos = newcurrentPos;
                pos=Point.Point.tranc(self.currentPos)
                print'>> move ' +str(tempD*self.scale)+' m , current Pos=' + str(pos) 
                return True;
            return False
x= 0 if False else 1
print x
r= robot(Point.Point(0,0), 0, 2, 0.25)
r.moveTo(Point.Point(-50,92))
while r.executeStep():
    pass
