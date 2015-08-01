'''
Created on Mar 8, 2013

@author: ahmed
'''
from DataStructure import Point
class minHeap(object):
    def __init__(self, L, W):
        self.keys = [0] * (L * W)
        self.Values = [None] * (L * W)
        self.contains = []
        for i in xrange (L):
            self.contains.append([False] * W)
        self.n = 0
    def Contains(self,item): 
        return self.contains[item.Y][item.X]
    def add(self, key, val):
            if self.n == len(self.Values):
                raise Exception("full heap!!")
            self.contains[val.Y][val.X] = True
            self.keys[self.n] = key
            self.Values[self.n] = val
            self.siftUp(self.n)
            self.n += 1   
    def min(self):
        if(self.n == 0):
            raise Exception("empty heap")
        return self.Values[0]
    def isEmpty(self):
        return self.n == 0
    def removemin(self):
        if self.isEmpty():
            raise Exception("Heap is empty!")
        minV = self.Values[0]
        self.keys[0] = self.keys[self.n - 1]
        self.Values[0] = self.Values[self.n - 1]
        self.n -= 1
        if self.n > 0:
            self.siftDown(0)
        self.contains[minV.Y][minV.X] = False
        return minV
    def siftUp(self, index):
        if index > 0:
            parent = (index - 1) / 2;
            if self.keys[parent] > self.keys[index]:
                self.swap(parent, index);
                self.siftUp(parent);
            
    def count(self):
        return self.n    
    def siftDown(self, index):
        leftChild = 2 * index + 1
        rightChild = 2 * index + 2

        if rightChild >= self.n and leftChild >= self.n:
                return
        smallestChild = -1
        if self.keys[rightChild] > self.keys[leftChild]:
             smallestChild = leftChild 
        else:
             smallestChild = rightChild

        if self.keys[index] > self.keys[smallestChild]:
            self.swap(smallestChild, index)
            self.siftDown(smallestChild)
    def swap(self, a, b):
        temp = self.keys[a]
        self.keys[a] = self.keys[b]
        self.keys[b] = temp
        tempV = self.Values[a]
        self.Values[a] = self.Values[b]
        self.Values[b] = tempV

h = minHeap(4, 4)
for i in xrange(4):
    p = Point.Point(3 - i, i)
    h.add(3 - i, p)
while not h.isEmpty():
    p = h.removemin()
    print p.X, ",", p.Y
