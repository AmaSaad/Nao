'''
Created on Mar 8, 2013

@author: ahmed
'''
class List(object):
    def __init__(self):
        self.list=[]
    def add(self,item):
        self.list.append(item)
    def remove(self,item):
        self.list.remove(item)
    def contains(self,item):
        for val in self.list:
            if item==val:
                return True
        return False
    def __str__(self):
       return str(self.list)     
    def __repr__(self):
        return self.__str__()