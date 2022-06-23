from time import time
import numpy as np
import random as rd
import math as m
import os

import matplotlib.pyplot as plt
import imageio.v2 as imageio
import cv2
from PIL import Image


#Classes :

class Point :
    def __init__(self, x, y, z) :
        self.x, self.y, self.z = x, y, z
        self.C = (x, y, z)
    
    def __str__(self) : #For debugging
        return f"({round(self.x, 2)},{round(self.y, 2)},{round(self.z, 2)})"
    
    #Comparaison methods :
    def __eq__(self, other) :
        if not isinstance(other, Point):
            return NotImplemented
        return self.C == self.C
    
    def __lt__(self, other) :   #Overload of '<' operator
        if not isinstance(other, Point):
            return NotImplemented
        return self.C < other.C
    
    def __hash__(self) :
        return hash(self.C)

class Edge :
    def __init__(self, A:Point, B:Point, C:Point=None) :
        self.A = A
        self.B = B
        self.C = C  #Point making the triangle this edge was found with.
    
    def __eq__(self, other) :
        if not isinstance(other, Edge):
            return NotImplemented
        return {self.A, self.B} == {other.A, other.B}
    
    def __hash__(self) :
        return hash((min(self.A, self.B), min(self.A, self.B)))
    
    def __str__(self) :
        return f"({self.A},{self.B})"
    
    def draw(self, subplot:plt.subplot, color:str, width=1) :
        subplot.plot3D([self.A.x, self.B.x], [self.A.y, self.B.y], [self.A.z, self.B.z], c=color, linewidth=width)
        return

class Scatter :
    #Scatter initialization
    def __init__(self) :
        self.points = []
        self.p_list = []
        self.p_arr = None
        
        fig = plt.figure()
        self.ax = fig.add_subplot(projection="3d")
        
        if DoA > 0 :
            self.frame = 0
            fig.set_size_inches(W, H)
            plt.savefig("temp.png")
            if name.endswith(".gif") :
                self.writer = imageio.get_writer(name)
            else :
                self.writer = cv2.VideoWriter(name, 0, 15, Image.open("temp.png").size)
    
    def generate(self) :
        # Adds N points with coordinates within limits to the scatter
        o = Point(0, 0, 0)
        X, Y, Z = 0, 0, 0
        for _ in range(N) :
            x, y, z = 2*rd.random()-1, 2*rd.random()-1, 2*rd.random()-1
            while dist(o, Point(x, y, z)) > 1 :
                x, y, z = 2*rd.random()-1, 2*rd.random()-1, 2*rd.random()-1
            X, Y, Z = X+x, Y+y, Z+z
            self.p_list.append([x, y, z])
            self.points.append(Point(x, y, z))
        self.bary = Point(X/N, Y/N, Z/N)
        self.p_arr = np.asarray(self.p_list)

    #Plotting functions :
    def draw(self, c="blue") :
        self.ax.scatter(self.p_arr[:,0], self.p_arr[:,1], self.p_arr[:,2], c=c, marker='.', linewidths=0.1)

    def draw_hull(self, c="green") :
        for edge in self.hull_edges :
            edge.draw(self.ax, color=c)
    
    #Gif building
    def build_gif(self) :
        plt.savefig("temp.png")
        if name.endswith(".gif") :
            self.writer.append_data(imageio.imread("temp.png"))
        else :
            self.writer.write(cv2.imread("temp.png"))
    
    def start_gif(self) :    #Starts the gif by doing a full rotation around the scatter.
        self.ax.clear()
        self.ax.set_xlim((-0.7, 0.7))
        self.ax.set_ylim((-0.7, 0.7))
        self.ax.set_zlim((-0.6, 0.6))
        self.ax.set_axis_off()
        self.draw()
        for i in range(45) :
            self.ax.view_init(azim = 8*i)
            self.build_gif()
    
    def end_gif(self) :    #Starts the gif by doing a full rotation around the scatter and its hull.
        self.ax.clear()
        self.ax.set_xlim((-0.7, 0.7))
        self.ax.set_ylim((-0.7, 0.7))
        self.ax.set_zlim((-0.6, 0.6))
        self.ax.set_axis_off()
        self.draw()
        self.draw_hull()
        for i in range(45) :
            self.ax.view_init(azim = self.frame + 8*i)
            self.build_gif()

    def draw_step(self, tri=None, k_res=None) :
        self.ax.clear()
        self.ax.set_xlim((-0.7, 0.7))
        self.ax.set_ylim((-0.7, 0.7))
        self.ax.set_zlim((-0.6, 0.6))
        self.ax.set_axis_off()
        self.draw()
        self.draw_hull()

        if tri is not None :
            edge, pivot = tri
            edge.draw(self.ax, "green", width=3)
            Edge(edge.A, pivot).draw(self.ax, "#FFA500")
            Edge(edge.B, pivot).draw(self.ax, "#FFA500")
            
            if k_res is not None :
                k, res = k_res
                if res < 0 :
                    color = "green"
                else :
                    color = "red"
                self.ax.scatter(k.x, k.y, k.z, marker='o', c=color, linewidths=3)
            
        self.ax.view_init(azim = self.frame)

        self.build_gif()

        ## Uncomment the next 2 lines to see every frame independently
        # plt.show(block=False)
        # plt.waitforbuttonpress()

        self.frame += 1

    #Geometrical functions
    def get_pivots(self) :
        self.points.sort(key=lambda p:(p.y, p.x, p.z))
        i, j = self.points[0], self.points[1]
        # Doing 1 iteration of Jarvis' algorithm on the (X Y) plane to have the first edge of the hull
        for k in self.points[2:] :
            res = orient(i, j, k)
            if res == 0 :
                if dist(i, k) < dist(i, j) :
                    j = k
            elif res < 0 :
                j = k
        return i, j
    
    def find_triangle(self, edge:Edge, s) :
        # Each edge is part of 2 triangles of the hull, s choses which one we're searching for.
        # Returns the point completing the triangle.

        #Find a pivot :
        for k in self.points :
            if k not in {edge.A, edge.B} :
                break
        pivot = k

        #Going through the points Jarvis-like :
        for k in self.points :
            if k in {edge.A, edge.B, pivot} :
                continue
            res = s*direct(edge, pivot, k)

            if DoA == 3 :
                self.draw_step((edge, pivot), (k, res))

            if res < 0 :

                if DoA == 2 :
                    self.draw_step((edge, pivot))

                pivot = k
        
        if DoA >= 2 :
            self.draw_step((edge, pivot))
        
        return pivot

    #The main algorithm
    def find_hull(self) :
        i, j = self.get_pivots()
        pivot = Edge(i, j)
        self.hull_edges = {pivot:2}
        unused_edges = []
        
        #Initialization with the pivot :
        for s in {-1, 1} :
            point = self.find_triangle(pivot, s)
            edgeA, edgeB = Edge(pivot.A, point, pivot.B), Edge(pivot.B, point, pivot.A)
            self.hull_edges[edgeA] = 1
            unused_edges.append(edgeA)
            self.hull_edges[edgeB] = 1
            unused_edges.append(edgeB)
            if DoA > 0 :
                self.draw_step()

        #Loop :
        while unused_edges != [] :
            if search_type == "DFS" :
                edge = unused_edges.pop()
            else :
                edge = unused_edges.pop(0)

            #Don't process edges that are already in 2 triangles of the hull
            if self.hull_edges[edge] >= 2 :
                continue
            
            point = self.find_triangle(edge, -direct(edge, edge.C, self.bary))
            #The algorithm will always search in the direction opposite to the known triangle (made by edge.A, edge.B and edge.C)
            new_edges = Edge(edge.A, point, edge.B), Edge(edge.B, point, edge.A)

            has_new_edge = False
            for new_e in new_edges :
                if new_e not in self.hull_edges :
                    self.hull_edges[new_e] = 0
                    unused_edges.append(new_e)
                    has_new_edge = True
                self.hull_edges[new_e] += 1

            if has_new_edge and DoA > 0 :
                self.draw_step()

#Geometrical algebra functions :

def orient(i, j, k):
    xij, yij = j.x - i.x, j.y - i.y
    xik, yik = k.x - i.x, k.y - i.y
    det = xij * yik - yij * xik
    return det # det > 0 <=> k on the left of ij

def direct(edge:Edge, pivot:Point, k:Point) :
    mat = np.asarray([
        [1, edge.A.x, edge.A.y, edge.A.z],
        [1, edge.B.x, edge.B.y, edge.B.z],
        [1, pivot.x, pivot.y, pivot.z],
        [1, k.x, k.y, k.z]
    ])
    return np.sign(np.linalg.det(mat))

def dist(a, b) :
    return ((a.x-b.x)**2+(a.y-b.y)**2+(a.z-b.z)**2)**0.5


#Reading settings :
File = open("settings.txt")
DoA = int(File.readline().strip().split(" : ")[1])
N = int(File.readline().strip().split(" : ")[1])
search_type = File.readline().strip().split(" : ")[1]
name = File.readline().strip().split(" : ")[1]
assert name.endswith(".avi") or name.endswith(".mp4") or name.endswith(".gif")
W, H = map(int, File.readline().strip().split(" : ")[1].split(', '))
File.close()


#Initialization of global variables :
S = Scatter()
S.generate()
#Drawing stuff
plt.get_current_fig_manager().full_screen_toggle()

#Running the thing : 
if DoA > 0 :
    print("Making video intro...")
    t0 = time()
    S.start_gif()
    print(f"took {round(time()-t0, 6)}s")

print("Building hull...")
t0 = time()
S.find_hull()
print(f"took {round(time()-t0, 6)}s (ploting, saving frames and building video included)") 

if DoA > 0 :
    print("Making video outro...")
    t0 = time()
    S.end_gif()
    if name.endswith(".gif") :
        S.writer.close()
    else :
        cv2.destroyAllWindows()
        S.writer.release()
    print(f"took {round(time()-t0, 6)}s")

os.remove("temp.png")

# You can uncomment the following lines to play around with the final result with matplotlib :
# S.ax.clear()
# S.ax.set_axis_off()
# S.draw()
# S.draw_hull()
# plt.show()