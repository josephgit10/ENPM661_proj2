#!/usr/bin/env python
# coding: utf-8

# In[1]:


import numpy as np
from queue import PriorityQueue
from cv2 import VideoWriter, VideoWriter_fourcc 
import Node

# Creating the map and obstacles:

def create_map():

    # map size of 250x600
    map = np.zeros((250,600))
    #radius of robot
    radius = 0 
    #clearance of robot
    clearance = 5 
    # Total clearance
    total = radius + clearance
    
   # Creating Shapes in the map:

    for i in range(map.shape[1]):
        for j in range(map.shape[0]):

            #Rectangle 1(lower)
            if (i > (100-total) and i < (150+total) and line((100,150),(150,150),i,j,total) < 0 
                and line((100,250),(150,250),i,j,total) > 0):
                map[j,i]=1
                
            #Rectangle 2(upper)           
            if (i > (100-total) and i < (150+total) and line((100,0),(150,0),i,j,total) < 0 
                and line((100,100),(150,100),i,j,total) > 0):
                map[j,i]=1
            
            #Triangle
            if (i>(460-total)and i<(510+total)and j>(25-total) and j< (225+2*total) and line((460,225),(510,125),i,j,total) > 0
               and line((460,25),(510,125),i,j,total) < 0):
                map[j,i]=1
                
            # Hexagon            
            if (i > (235.05-total) and i < (364.95+total) and line((235.05,87.5),(300,50),i,j,total) < 0 
                and line((300,50),(364.95,87.5),i,j,total) < 0 
                and line((235.05,162.5),(300,200),i,j,total) > 0 
                and line((300,200),(364.95,162.5),i,j,total) > 0):
                map[j,i]=1
            
            # For the boundaries
            if (i > 0 and i < total):
                map[j,i] = 1
            if (i < 600 and i > (600-total)):
                map[j,i] = 1
            if (j > 0 and j < total):
                map[j][i] = 1
            if (j < 250 and j >(250 - total)):
                map[j][i] = 1    
    return map


def line(p1, p2, x, y, t):

    a = (p2[1] - p1[1])
    b = x - p1[0]
    c = p2[0] - p1[0]
    d = p1[1] + t - y
    dist = ( a*b ) / ( c) + (d)
    
    return dist


# To check for collision

def collide(point,map):
    flag = False
    
    # checking whether points are coinciding with obstacle
    if map[point[1],point[0]] == 1:
        flag = True
    
    return flag



# To Explore neighbours

def explore(node,map):
    x = node.x
    y = node.y

    Action_Set = {(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1), (1,-1), (-1,-1)} 
    m = [(x, y + 1),(x + 1, y),(x, y -1),(x - 1, y),(x + 1, y + 1),(x + 1, y - 1),(x - 1, y - 1),(x - 1, y + 1)]
    paths = []

    for pos, m in enumerate(m):
        if not (m[0] >= 600 or m[0] < 0 or m[1] >= 250 or m[1] < 0):
            if map[m[1]][m[0]] == 0:
                cost = 1.4 if pos > 3 else 1
                paths.append([m,cost])
    return paths




# To perform Dijkstra Algorithm from the above functions

def Dijkstra(goal_node, map):
 
    flag = False
    while not flag:
        start_node = [int(item) for item in input("\n Start Node (numbers separated using comma but no spaces): ").split(',')]
        #changing frame
        start_node[1] = 250 - start_node[1] 

        # To set boundary for start node
        if (len(start_node) == 2 and (0 <= start_node[0] <= 600) and (0 <= start_node[1] <= 250)):
            if not collide(start_node,map):
                flag = True
            else:   
                print("Collision \n")
        else:
            print("Please enter a valid node.\n")
            flag = False
            
    print("\n Executing Dijkstra Algorithm\n")

    q = PriorityQueue()
    visited = set([])
    node_objects = {}
    distance = {}
    
    distance = {(i, j): float('inf') for i in range(map.shape[1]) for j in range(map.shape[0])}

    distance[str(start_node)] = 0
    visited.add(str(start_node))
    node = Node.Node(start_node,0,None)
    node_objects[str(node.pos)] = node
    
    # Inserting start node to PriorityQueue
    q.put([node.cost, node.pos])

    while not q.empty():
        node_temp = q.get()
        node = node_objects[str(node_temp[1])]  
                                     
        # Check for goal node
        if node_temp[1][0] == goal_node[0] and node_temp[1][1] == goal_node[1]:      
            print(" Success\n")
            node_objects[str(goal_node)] = Node.Node(goal_node,node_temp[0], node)
            break
        
        for next_node, cost in explore(node,map):

            if str(next_node) in visited:
                temp = cost + distance[str(node.pos)]
                if temp < distance[str(next_node)]:
                    distance[str(next_node)] = temp
                    node_objects[str(next_node)].parent = node

            else:
                visited.add(str(next_node))
                a_c = cost + distance[str(node.pos)]
                distance[str(next_node)] = a_c
                new_node = Node.Node(next_node, a_c, node_objects[str(node.pos)])
                node_objects[str(next_node)] = new_node
                q.put([a_c, new_node.pos])
    
    # For backtracking
    reverse_path = []
    goal = node_objects[str(goal_node)]
    reverse_path.append(goal_node)
    parent_node = goal.parent
    while parent_node:                              
        reverse_path.append(parent_node.pos)
        parent_node = parent_node.parent 
        
    path = list(reversed(reverse_path))

    return node_objects, path


# To get the goal node

def goal(map):
    
    flag = False
    while not flag:
        goal_node = [int(item) for item in input("\n Goal Node (numbers separated using comma but no spaces): ").split(',')] 
       
        # Converting to given frame of the map
        goal_node[1] = 250 - goal_node[1]
        
        # Setting boundries
        if (len(goal_node) == 2 and (0 <= goal_node[0] <= 600) and (0 <= goal_node[1] <= 250)):
            if not collide(goal_node,map):
                flag = True
            else:
                print("Collision \n")
        else:
            print("Please enter a valid node.\n")
            flag = False
        
    return goal_node


# To display the animation video
def Animation_video(node_objects, path, map):
                                                
    fourcc = VideoWriter_fourcc(*'mp4v')
    video = VideoWriter('./Animation_Video.mp4', fourcc, float(300), (600, 250))
    
    nodes = node_objects.values()                                                   
    nodes = list(nodes)
    image = np.dstack([map.copy() * 200, map.copy() * 0, map.copy() * 255])
    image = np.uint8(image)
    video.write(image)
    
    # To add visited nodes to video frame
    for i in range(len(nodes)):
        image[nodes[i].pos[1], nodes[i].pos[0], :] = np.array([0,255,0])
        video.write(image)
   
     # To add generated path to video frame 
    for i in range(len(path) - 1):                                                  
        image[path[i][1], path[i][0], :] = np.array([102,0,51])
        video.write(image)
        
    video.release()
    print("Animation video saved.")



map = create_map()
goal_node = goal(map)
node_objects, path = Dijkstra(goal_node, map)
Animation_video(node_objects, path, map)


# In[ ]:




