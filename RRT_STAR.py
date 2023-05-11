#Importing the library
import numpy as np
import pygame
# from queue import PriorityQueue
import time
import math
import random
from shapely.geometry import Point, LineString


# Initializing the three used colors
color = (255,255,255)
color_2 = (255,200,150)
color_3=(0,0,0)

# Initializing the map
pygame.init()
width_, height_ = 600, 250

# Initializing surface
surface = pygame.Surface((width_,height_))
surface.fill(color_2)

clearance=2
radius=2
cr=clearance+radius
cr2=radius-clearance
#Drawing the map with obstacles
pygame.draw.rect(surface, color, pygame.Rect(cr, cr, width_-2*cr, height_-2*cr))

# Define the hexagon in the center with original dimensions
pygame.draw.rect(surface, color, pygame.Rect(cr, cr, width_-2*cr, height_-2*cr))
bottom_rect_dim = [(250-cr2,200),(265+cr,200),(265+cr,75-cr2),(250-cr2,75-cr2)]
# pygame.draw.polygon(surface, color_2,bottom_rect_dim)
top_rect_dim = [(150-cr2,0),(165+cr,0),
                (165+cr,125+cr),(150-cr2,125+cr)]
pygame.draw.polygon(surface,color_2,top_rect_dim)

# Drawing a circle
pygame.draw.circle(surface, color_2,(300,125),70+cr)

# Convert surface to a 2D array with 0 for the specific color and 1 for other colors
arr = np.zeros((surface.get_width(), surface.get_height()))
pix = pygame.surfarray.pixels3d(surface)
arr[np.where((pix == color_2).all(axis=2))] = 1
obs_np = np.where((pix == color_2).all(axis=2))
obstacles={}
for i in range(obs_np[0].shape[0]):
    obs_key = (obs_np[0][i], obs_np[1][i])  # Create a new tuple (y, x)
    obstacles[obs_key] = None  # Use the tuple as a key and assign a value of None
del pix


# Breseham's line algorithm
def bresenham_line(x0, y0, x1, y1):
    points = []
    dx = np.abs(x1 - x0)
    dy = np.abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy

    while x0 != x1 or y0 != y1:
        points.append((x0, y0))
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

    points.append((x0, y0))
    return points

# Nearest neighbor function
def nearest_neighbor(new_node, nodes):
    #Given a new node and a set of existing nodes, finds the nearest neighbor to the new node in the set of existing nodes.
    dist = np.linalg.norm(nodes - new_node, axis=1)
    nearest_node_idx = np.argmin(dist)
    nearest_node = nodes[nearest_node_idx]
    return nearest_node

# def nearest_neighbors(new_node, nodes, radius):
#     # Given a new node and a set of existing nodes, finds the indices of nodes within a given radius of the new node.
#     distances = np.linalg.norm(nodes - new_node, axis=1)
#     neighbor_indices = np.where(distances <= radius)[0]
#     return neighbor_indices

# RRT algorithm
def rrt(start,goal,goal_tolerance, max_iterations,step_size,radius):#step_size, ,  goal_tolerance goal,
    # Initialize the RRT algorithm with a start node.
    pixels = {}
    pixels[start] = [0,-1]
    coords = {}
    coords[0] = start#cost
    parent = {}
    parent[0] = []
    i=1

    for p in range(1, max_iterations):
        # Generate a new random point in the 2D space.
        while True:
            x = random.randint(0, 600)
            y = random.randint(0, 250)
            q_rand = (x, y)
            q_rand=tuple(q_rand)
            # Find the nearest node to the new point in the set of existing nodes.
            q_near = nearest_neighbor(q_rand, np.array(list(pixels.keys())))
            q_near=tuple(q_near)
            if(q_near!=q_rand):
                break
        
        
        # Extend the tree from the nearest node towards the new point.
        
        v = np.array(q_rand) - np.array(q_near)
        u = v / np.linalg.norm(v)
        q_new = (tuple(np.array(q_near) + step_size * u))
        q_new=tuple(np.round(q_new).astype(int))
        


        if((q_new not in pixels) and q_new not in obstacles):
            

            distances=np.linalg.norm(np.array(list(pixels.keys())) - np.array(q_new), axis=1)
            
            q_near_idx = np.where(distances <= radius)[0]
            
            if len(q_near_idx)>0:
                
                costs={}
                
                for z in q_near_idx:
                    
                    cost=np.linalg.norm(q_new-np.array(coords[z][0]))
                    path = []
                    node=coords[z]
                    
                    while(pixels[node][1]!=-1):
                        
                        cost+=np.linalg.norm(node-np.array(coords[pixels[node][1]]))
                        node=coords[pixels[node][1]]
                        

                        
                    costs[z]=cost
                sorted_keys = [k for k, v in sorted(costs.items(), key=lambda item: item[1], reverse=False)]
                ez=None
                for k in sorted_keys:
                    
                    rode=tuple(bresenham_line(coords[k][0],coords[k][1],q_new[0],q_new[1]))
                    if(not any(arr[x, y] == 1 for x, y in rode)):
                        ez=k
                        break
                    else:
                        continue
                if(ez is not None):
                    
                    coords[i]=q_new          
                    pixels[q_new]=[i,z]
                    i+=1


                    

                    #Rewire
                    dist_q_new=0
                    node=tuple(q_new)
                    
                    while(pixels[node][1]!=-1):
                            
                        dist_q_new+=np.linalg.norm(node-np.array(coords[pixels[node][1]]))
                        node=coords[pixels[node][1]]
                            
                    distances=np.linalg.norm(np.array(list(pixels.keys())) - np.array(q_new), axis=1)
                    
                    q_near_idx = np.where(distances <= radius)[0]
                    
                    if len(q_near_idx)>0:
                        
                        for z in q_near_idx:
                            rode=tuple(bresenham_line(coords[k][0],coords[k][1],q_new[0],q_new[1]))
                            if(not any(arr[x, y] == 1 for x, y in rode)):
                                
                                
                                cost_1=dist_q_new+np.linalg.norm(q_new-np.array(coords[z][0]))
                                
                                node=coords[z]
                                
                                
                                cost=0
                                kulcs=pixels[node][1]
                                
                                while(pixels[node][1]!=-1):
                                    
                                    cost+=np.linalg.norm(node-np.array(coords[pixels[node][1]]))
                                    node=coords[pixels[node][1]]
                                   
                                    
                                if(cost_1<cost):
                                    
                                    pixels[tuple(coords[z])][1]=pixels[q_new][0]


                        
                
                




            else:
                
                rode=tuple(bresenham_line(q_near[0],q_near[1],q_new[0],q_new[1]))
                

                if( not any(arr[x, y] == 1 for x, y in rode)):
                    
                    coords[i]=q_new          
                    pixels[q_new]=[i,pixels[q_near][0]]
                    i+=1
                    if pixels[q_near][0] not in parent:
                        parent[pixels[q_near][0]]=[q_new ]
                    else:
                        parent[pixels[q_near][0]].append(q_new )

                    


            q_m = nearest_neighbor(goal, np.array(list(pixels.keys())))           
            # if np.linalg.norm(np.array(q_new ) - np.array(goal)) <= goal_tolerance:
            path = []
            node=tuple(q_m)
            
    
            while(pixels[node][1]!=-1):
                
                path.append(node)
                node=coords[pixels[node][1]]
                
                
            path.append(node)
            path.reverse()
            
            

       

    # We did not find a path to the goal within the maximum number of iterations.
    return pixels,coords,parent,path

line_width = 2
point_size = 2


# Creating the end display 
s=pygame.display.set_mode((width_,height_))
s.blit(surface,(0,0))
pygame.display.update()
start=(10,10)
goal=(400,90)
pi,c,pa,path=rrt(start,goal,10,3000,5,10)
# print("Path",path)
for point in pi.keys():
    # print(type(point))
    pygame.draw.circle(s, (100,170,210), point, point_size)
    

    pygame.draw.circle(s, (50,200,240), start, 4)
    pygame.draw.circle(s, (50,200,240), goal, 4)
pygame.display.update()

for key, value_list in pa.items():
    # print(f"Key: {key}")
    # Iterate through each element in the list of values for the current key
    for value in value_list:
        # Draw the line on the surface
        # print((c[key]), "",tuple(value))
        pygame.draw.line(s, (255,0,0), (c[key][0]), tuple(value), line_width)

        # Update the display
        pygame.display.update()

pygame.draw.lines(s, (50,50,50), False, path, 4)
pygame.display.update()
# print(c)
        




running = True
pygame.time.wait(10000)
# Game loop
while running:
# For loop through the event queue  
    for event in pygame.event.get():
        
        # Check for QUIT event      
        if event.type == pygame.QUIT:
            pygame.quit()
            running = False
