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

# RRT algorithm
def rrt(start,goal,goal_tolerance, max_iterations,step_size):#step_size, ,  goal_tolerance goal,
    # Initialize the RRT algorithm with a start node.
    pixels = {}
    pixels[start] = [0,-1]
    coords = {}
    coords[0] = [start,0]
    parent = {}
    parent[0] = []

    pixels_2 = {}
    pixels_2[goal] = [0,-1]
    coords_2 = {}
    coords_2[0] = [goal,0]
    parent_2 = {}
    parent_2[0] = []

    for i in range(1, max_iterations):
        # Generate a new random point in the 2D space.
        while True:
            x = random.randint(0, 600)
            y = random.randint(0, 250)
            q_rand = (x, y)
            q_rand=tuple(q_rand)
            # Find the nearest node to the new point in the set of existing nodes.
            q_near = nearest_neighbor(q_rand, np.array(list(pixels.keys())))
            q_near=tuple(q_near)
            q_near_2 = nearest_neighbor(q_rand, np.array(list(pixels_2.keys())))
            q_near_2=tuple(q_near_2)
            if(q_near!=q_rand and q_near_2!=q_rand):
                break
        
        
        
        
        
        # Extend the tree from the nearest node towards the new point.
        
        v = np.array(q_rand) - np.array(q_near)
        u = v / np.linalg.norm(v)
        q_new = (tuple(np.array(q_near) + step_size * u))
        q_new=tuple(np.round(q_new).astype(int))
        o_dist=coords[pixels[q_near][0]][1]+math.dist(q_near,q_new)
        rode=tuple(bresenham_line(q_near[0],q_near[1],q_new[0],q_new[1]))

        v = np.array(q_rand) - np.array(q_near_2)
        u = v / np.linalg.norm(v)
        q_new_2 = (tuple(np.array(q_near_2) + step_size * u))
        q_new_2=tuple(np.round(q_new_2).astype(int))
        o_dist_2=coords_2[pixels_2[q_near_2][0]][1]+math.dist(q_near_2,q_new_2)
        rode_2=tuple(bresenham_line(q_near_2[0],q_near_2[1],q_new_2[0],q_new_2[1]))
       

        if((q_new not in pixels) and q_new not in obstacles and not any(arr[x, y] == 1 for x, y in rode)
           and (q_new_2 not in pixels_2) and q_new_2 not in obstacles and not any(arr[x, y] == 1 for x, y in rode_2)):#and (q_new not in obstacles.keys() )
            
            coords[i]=[q_new,o_dist]          
            pixels[q_new]=[i,pixels[q_near][0]]
            if pixels[q_near][0] not in parent:
                parent[pixels[q_near][0]]=[q_new ]
            else:
                parent[pixels[q_near][0]].append(q_new )
            


            coords_2[i]=[q_new_2,o_dist_2]          
            pixels_2[q_new_2]=[i,pixels_2[q_near_2][0]]
            if pixels_2[q_near_2][0] not in parent_2:
                parent_2[pixels_2[q_near_2][0]]=[q_new_2 ]
            else:
                parent_2[pixels_2[q_near_2][0]].append(q_new_2 )
            

            if(np.linalg.norm(np.array(q_new ) - np.array(q_new_2))<step_size):

                path = []
                node=q_new
                
                
               
                while(pixels[node][1]!=-1):
                    path.append(node)
                    node=coords[pixels[node][1]][0]
                    
                    
                path.append(node)
                path.reverse()

                path_2 = []
                node_2=q_new_2
                
                
               
                while(pixels_2[node_2][1]!=-1):
                    path_2.append(node_2)
                    node_2=coords_2[pixels_2[node_2][1]][0]
                    
                    
                path_2.append(node_2)
                

            

                break
            

       

    # We did not find a path to the goal within the maximum number of iterations.
    return pixels,coords,parent,path, pixels_2,coords_2,parent_2,path_2

line_width = 2
point_size = 2


# Creating the end display 
s=pygame.display.set_mode((width_,height_))
s.blit(surface,(0,0))
pygame.display.update()
start=(10,10)
goal=(400,230)
pi,c,pa,path,pi_2,c_2,pa_2,path_2=rrt(start,goal,15,2000,10)
for point in pi.keys():
    # print(type(point))
    pygame.draw.circle(s, (100,170,210), point, point_size)
    

    pygame.draw.circle(s, (50,200,240), start, 4)
    
pygame.display.update()

for point in pi_2.keys():
    # print(type(point))
    pygame.draw.circle(s, (150,120,110), point, point_size)
    

    pygame.draw.circle(s, (50,200,240), goal, 4)
    
pygame.display.update()



for key, value_list in pa.items():
    
    # Iterate through each element in the list of values for the current key
    for value in value_list:
        # Draw the line on the surface
        
        pygame.draw.line(s, (255,0,0), (c[key][0]), tuple(value), line_width)

        # Update the display
        pygame.display.update()

for key, value_list in pa_2.items():
    # print(f"Key: {key}")
    # Iterate through each element in the list of values for the current key
    for value in value_list:
        # Draw the line on the surface
        
        pygame.draw.line(s, (100,75,75), (c_2[key][0]), tuple(value), line_width)

        # Update the display
        pygame.display.update()

pygame.draw.lines(s, (50,50,50), False, path, 4)
pygame.draw.lines(s, (200,200,200), False, path_2, 4)
pygame.display.update()
print(c)
        




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
