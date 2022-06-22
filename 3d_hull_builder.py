from time import time
import numpy as np
import matplotlib.pyplot as plt
import random as rd
import math as m
import imageio.v2 as imageio


#Classes :

class Point :
    def __init__(self, x, y, z) :
        self.x, self.y, self.z = x, y, z
        self.C = (x, y, z)
    
    def __str__(self) : #For debugging
        return f"({round(self.x, 2)}, {round(self.y)}, {round(self.z)})"
    
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
    def __init__(self, A:Point, B:Point) :
        self.A = A
        self.B = B
    
    def __eq__(self, other) :
        if not isinstance(other, Edge):
            return NotImplemented
        return {self.A, self.B} == {other.A, other.B}
    
    def __hash__(self) :
        return hash((min(self.A, self.B), max(self.A, self.B)))
    
    def __str__(self) :
        return f"({self.A}, {self.B})"
    
    def draw(self, subplot:plt.subplot, color:str, width=1) :
        subplot.plot3D([self.A.x, self.B.x], [self.A.y, self.B.y], [self.A.z, self.B.z], c=color, linewidth=width)
        return

class Scatter :
    #Scatter initialization
    def __init__(self) :
        self.points = []
        self.p_list = []
        self.p_arr = None
        self.hull = set()
        self.hull_edges = set()
        
        self.ax = plt.figure().add_subplot(projection="3d")
        
        if DoA > 0 :
            self.frame = 0
            self.writer = imageio.get_writer(f'{name}.gif', mode='I')
    
    def limits(self, dim) :
        values = [p.C[dim] for p in self.points]
        return (min(values), max(values))
    
    def generate(self) :
        # Adds N points with coordinates within limits to the scatter
        for _ in range(N) :
            r, theta, phi = rd.random(), m.pi*rd.random(), 2*m.pi*rd.random()
            x, y, z = r*m.cos(phi)*m.cos(theta), r*m.cos(phi)*m.sin(theta), r*m.sin(phi)
            self.p_list.append([x, y, z])
            self.points.append(Point(x, y, z))
        self.p_arr = np.asarray(self.p_list)
    
        if DoA > 0 :
            self.ax.set_xlim(self.limits(0))
            self.ax.set_ylim(self.limits(1))
            self.ax.set_zlim(self.limits(2))

    #Plotting functions :
    def draw(self, c="blue") :
        self.ax.scatter(self.p_arr[:,0], self.p_arr[:,1], self.p_arr[:,2], c=c, marker='.')

    def draw_hull(self, c="green") :
        for edge in self.hull_edges :
            edge.draw(self.ax, color=c)
    
    #Gif building
    def build_gif(self) :
        plt.savefig(f"temp.png")
        self.writer.append_data(imageio.imread("temp.png"))
    
    def start_gif(self) :    #Starts the gif by doing a full rotation around the scatter.
        self.ax.clear()
        self.ax.set_axis_off()
        self.draw()
        for i in range(45) :
            self.ax.view_init(azim = 8*i, elev = 30)
            self.build_gif()
    
    def end_gif(self) :    #Starts the gif by doing a full rotation around the scatter and its hull.
        self.ax.clear()
        self.ax.set_axis_off()
        self.draw()
        self.draw_hull()
        for i in range(45) :
            self.ax.view_init(azim = self.frame + 8*i, elev = 30*m.cos(self.frame*m.pi/180))
            self.build_gif()

    def draw_step(self, tri=None, k_res=None) :
        self.ax.clear()
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
            
        self.ax.view_init(azim = self.frame, elev = 30*m.cos(self.frame*m.pi/180))

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
        self.hull = {i, j}
        pivot = Edge(i, j)
        self.hull_edges = {pivot:0}
        unused_edges = [pivot]

        while unused_edges != [] :
            if search_type == "DFS" :
                edge = unused_edges.pop()
            else :
                edge = unused_edges.pop(0)

            #Don't process edges that are already in 2 triangles of the hull
            if self.hull_edges[edge] >= 2 :
                continue
            
            for s in {1, -1} :

                point = self.find_triangle(edge, s)
                self.hull.add(point)
                new_edges = Edge(edge.A, point), Edge(edge.B, point)

                is_old_triangle = True  #Checking if the current triangle is already in the hull
                for new_e in new_edges :
                    if new_e not in self.hull_edges :
                        self.hull_edges[new_e] = 0
                        is_old_triangle = False
                        unused_edges.append(new_e)
                
                if not is_old_triangle :
                    for new_e in new_edges :
                        self.hull_edges[new_e] += 1
                        if DoA > 0 :
                            self.draw_step()

                    if edge != pivot :
                        break   #No need to try the other triangle, it will be an old triangle.

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
    return np.linalg.det(mat)

def dist(a, b) :
    return ((a.x-b.x)**2+(a.y-b.y)**2+(a.z-b.z)**2)**0.5


#Reading settings :
File = open("settings.txt")
DoA = int(File.readline().strip().split(" : ")[1])
N = int(File.readline().strip().split(" : ")[1])
search_type = File.readline().strip().split(" : ")[1]
name = File.readline().strip().split(" : ")[1]
File.close()


#Initialization of global variables :
S = Scatter()
S.generate()
plt.get_current_fig_manager().full_screen_toggle()  #To enable matplotlib fullscreen


#Running the thing : 
if DoA > 0 :
    S.start_gif()

print("Starting algorithm")
t0 = time()
S.find_hull()
print(f"building hull took {round(time()-t0, 6)}s (ploting, saving frames and building gif included)") 

if DoA > 0 :
    S.end_gif()

# You can uncomment the following lines to play around with the final result with matplotlib :
# S.ax.clear()
# S.ax.set_axis_off()
# S.draw()
# S.draw_hull()
# plt.show()