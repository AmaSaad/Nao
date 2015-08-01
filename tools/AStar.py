'''
Created on Mar 8, 2013

@author: ahmed
'''
import cv2
from DataStructure.List import List
from DataStructure.minHeap import minHeap
from DataStructure.Point import Point
import math
from tools.robot import robot
class AStar(object):
    def __init__(self, mapfilename):
        self.img = cv2.imread(mapfilename, cv2.CV_LOAD_IMAGE_GRAYSCALE)
        self.Length = len(self.img)
        self.Width = len(self.img[0])
        self.Clusters = List()
        self.isNodeGraph = [] 
        self.D = 1
        for i in xrange(self.Length):
            self.isNodeGraph.append([False] * self.Width)
        estimatMaxGraphCount =int(math.sqrt (self.Length * self.Width)*2)
        self.connected = []
        for i in xrange(estimatMaxGraphCount):
            self.connected.append([False] * estimatMaxGraphCount)
        self.graph = []
        S_i = 0
        S_j = 0
        E_i = 0 
        E_j = 0
        wasBlocked = False
        inBlock = False
        meetNewBlock = False
        meetNewFree = False
        for i in xrange(self.Length):
            for j in xrange (self.Width + 1):
                wasBlocked = inBlock;
                inBlock = j == self.Width or self.img[i][j] == 0;
                meetNewBlock = not wasBlocked and inBlock;
                meetNewFree = wasBlocked and not inBlock;
                if meetNewBlock:
                    E_i = i;
                    E_j = j - 1;
                    pp = Point((S_j + E_j) / 2, (S_i + E_i) / 2);
                    self.Add(pp)
                elif meetNewFree:
                    S_i = i;
                    S_j = j;
        S_i = 0
        S_j = 0
        E_i = 0 
        E_j = 0
        wasBlocked = False
        inBlock = False
        meetNewBlock = False
        meetNewFree = False
        for j in xrange (self.Width):
            for i in xrange(self.Length + 1):
                wasBlocked = inBlock;
                inBlock = i == self.Length or self.img[i][j] == 0;
                meetNewBlock = not wasBlocked and inBlock;
                meetNewFree = wasBlocked and not inBlock;
                if meetNewBlock:
                    E_i = i - 1;
                    E_j = j;
                    pp = Point((S_j + E_j) / 2, (S_i + E_i) / 2);
                    self.Add(pp)
                elif meetNewFree:
                    S_i = i;
                    S_j = j;
                    
        self.makeGraph()
        self.connectGraph()
    def Add(self, tm):
        up = Point(tm.X, tm.Y - 1);
        down = Point(tm.X, tm.Y + 1);
        right = Point(tm.X + 1, tm.Y);
        left = Point(tm.X - 1, tm.Y);
        upRight = Point(tm.X + 1, tm.Y - 1);
        upLeft = Point(tm.X - 1, tm.Y - 1);
        downRight = Point(tm.X + 1 + 1, tm.Y + 1);
        downLeft = Point(tm.X - 1, tm.Y + 1);
        added = False;
        for cluster in self.Clusters.list:
            if cluster.contains(up) or cluster.contains(down) or cluster.contains(left) or cluster.contains(right) or cluster.contains(upRight) or cluster.contains(upLeft) or cluster.contains(downRight) or cluster.contains(downLeft):
                cluster.add(tm)
                added = True
                break
        L = List()
        L.add(tm)
        if not added:
            self.Clusters.add(L)
    def connectGraph(self):
        for i in xrange(len(self.graph)):
            for j in xrange(i + 1, len(self.graph)):
                self.connected[i][j] = self.validToConnectDirectly(i, j)
    def makeGraph(self):
        for cluster in self.Clusters.list:
            pr = self.getbyCluster(cluster);
            self.graph.append(pr);
            self.isNodeGraph[pr.Y][pr.X] = True;
    def getbyCluster(self, cluster):
        x = 0;
        y = 0;
        n = 0;
        for p in cluster.list:
            x += p.X;
            y += p.Y;
            n += 1;
        return Point(x / n, y / n);
    def validToConnectDirectly(self, i, j):
        i_st = self.graph[i].Y if self.graph[i].Y < self.graph[j].Y else self.graph[j].Y
        i_end = self.graph[i].Y if self.graph[i].Y > self.graph[j].Y else self.graph[j].Y
        j_st = self.graph[i].X if self.graph[i].X < self.graph[j].X else self.graph[j].X;
        j_end = self.graph[i].X if self.graph[i].X > self.graph[j].X else self.graph[j].X;
        for ii in xrange(i_st, i_end + 1):
            for jj in xrange(j_st, j_end + 1):
                if (ii != self.graph[i].Y or jj != self.graph[i].X) and (ii != self.graph[j].Y or jj != self.graph[j].X) and (self.isNodeGraph[ii][jj]):
                    return False;
                if self.img[ii][jj] == 0:
                    if (self.LineIntersectsRect(self.graph[i], self.graph[j], Point(jj - self.D / 2.0 , ii - self.D / 2.0), Point(jj - self.D / 2.0, ii + self.D / 2.0), Point(jj + self.D / 2.0, ii - self.D / 2.0), Point(jj + self.D / 2.0, ii + self.D / 2.0))):
                        return False
        return True;
    def LineIntersectsLine(self, l1p1, l1p2, l2p1, l2p2):
        q = (l1p1.Y - l2p1.Y) * (l2p2.X - l2p1.X) - (l1p1.X - l2p1.X) * (l2p2.Y - l2p1.Y);
        d = (l1p2.X - l1p1.X) * (l2p2.Y - l2p1.Y) - (l1p2.Y - l1p1.Y) * (l2p2.X - l2p1.X);
        if d == 0:
            return False
        r = q / d;
        q = (l1p1.Y - l2p1.Y) * (l1p2.X - l1p1.X) - (l1p1.X - l2p1.X) * (l1p2.Y - l1p1.Y);
        s = q / d;
        if r < 0 or r > 1 or s < 0 or s > 1:
            return False
        return True;
    def heuristic_cost_estimate(self, p1, p2):
        return Point.distantTo(p1, p2)
    def LineIntersectsRect(self, p1, p2, pr1, pr2, pr3, pr4):
        d11 = Point(p1.X + self.D / 2.0, p1.Y - self.D / 2.0);
        d12 = Point(p1.X + self.D / 2.0, p1.Y + self.D / 2.0);
        d13 = Point(p1.X - self.D / 2.0, p1.Y + self.D / 2.0);
        d21 = Point(p2.X + self.D / 2.0, p2.Y - self.D / 2.0);
        d22 = Point(p2.X + self.D / 2.0, p2.Y + self.D / 2.0);
        d23 = Point(p2.X - self.D / 2.0, p2.Y + self.D / 2.0);
        return self.LineIntersectsLine(d11, d21, pr1, pr2) or self.LineIntersectsLine(d12, d22, pr1, pr2) or self.LineIntersectsLine(d13, d23, pr1, pr2) or self.LineIntersectsLine(d11, d21, pr2, pr3) or self.LineIntersectsLine(d12, d22, pr2, pr3) or self.LineIntersectsLine(d13, d23, pr2, pr3) or self.LineIntersectsLine(d11, d21, pr3, pr4) or self.LineIntersectsLine(d12, d22, pr3, pr4) or self.LineIntersectsLine(d13, d23, pr3, pr4) or self.LineIntersectsLine(d11, d21, pr4, pr1) or self.LineIntersectsLine(d12, d22, pr4, pr1) or self.LineIntersectsLine(d13, d23, pr4, pr1)
    def connectToGraph(self, p):
        if self.isNodeGraph[p.Y][p.X]:
            return;
        self.graph.append(p);
        self.isNodeGraph[p.Y][p.X] = True;
        for i in xrange(len(self.graph) - 1):
            self.connected[i][len(self.graph) - 1] = self.validToConnectDirectly(i, len(self.graph) - 1)
    def getNeighbours(self, current):
        ls = List()
        indx = self.graph.index(current);
        for i in xrange(len(self.graph)):
            if indx < i:
                if self.connected[indx][i] :
                    ls.add(self.graph[i]);
            elif indx > i:
                if self.connected[i][indx]: 
                    ls.add(self.graph[i]); 
        return ls
    def getPath(self, start,  goal):
        self.connectToGraph(start);
        self.connectToGraph(goal);
        openset = minHeap(self.Length,self.Width)
        g_score =[]
        f_score =[]
        came_from=[]
        close_set=[]
        for i in xrange(self.Length):
            g_score.append([0]*self.Width)
            f_score.append([0]*self.Width)
            came_from.append([None]*self.Width)
            close_set.append([False]*self.Width)
        f_score[start.Y][start.X] = g_score[start.Y][start.X ] + self.heuristic_cost_estimate(start, goal);
        openset.add(f_score[start.Y][start.X], start);
        neighbors=None;
        while openset.count() != 0:
            current = openset.removemin();
            if current==goal:
                return self.reconstruct_path(came_from, goal, start);
            close_set[current.Y][current.X] = True;
            neighbors = self.getNeighbours(current);
            for neighbor in neighbors.list:
                tentative_g_score = g_score[current.Y][current.X] + Point.distantTo(current, neighbor);
                if close_set[neighbor.Y][neighbor.X]:
                    if tentative_g_score >= g_score[neighbor.Y][neighbor.X]:
                        continue;
                if not openset.Contains(neighbor) or tentative_g_score < g_score[neighbor.Y][neighbor.X]:
                    came_from[neighbor.Y][neighbor.X] = current;
                    g_score[neighbor.Y][neighbor.X] = tentative_g_score;
                    f_score[neighbor.Y][neighbor.X] = g_score[neighbor.Y][neighbor.X] + self.heuristic_cost_estimate(neighbor, goal);
                    if not openset.Contains(neighbor):
                        openset.add(f_score[neighbor.Y][neighbor.X], neighbor);
        return None;
    def reconstruct_path(self,came_from,current_node, start):
        stk =List()
        t = current_node;
        while t != None:
                stk.add(t);
                t = came_from[t.Y][t.X];
        return stk;
st=AStar('../maps/map.png')   
h=st.getPath(Point(0,0), Point(98,99))
r=robot(h.list[len(h.list)-1], 0, 2, 0.25)
print '=-=-=-=-=-=-=-=-=-=-= start moving =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=\n'
for i in xrange(len(h.list)-1):
    r.moveTo(h.list[len(h.list)-2-i])
    while r.executeStep():
        pass
    
