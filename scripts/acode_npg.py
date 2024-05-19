#______________________________________________________________________________________________________
# GITHUB LINK

# https://github.com/ruthwikdasyam/Astar_gazebo.git
#______________________________________________________________________________________________________
import heapq
import numpy as np
import pygame
import time

print("\n-----------------------------------------------------------------------\n")
print("                        A* PATH PLANNING                                ")
print("\n-----------------------------------------------------------------------\n")

start_time = time.time() #  Initiating time

#__________________________________________________________________________________________________________________
#Defining Actions

#Node_State comprises of Node point and its orentation
#point consists of x and y coordinate
# Global Constants
wheel_radius = 3.3 # r
track_width = 28.7 # L 
no_of_nodes = 0
weight = 1

#Class Node
class node_class:
    all_states = []
    def __init__(self, C2C : float, C2G : float, Node_Index: int, Parent_Index: int, State: np.array) -> None:
        #Assign to Self Object
        self.C2C = C2C  # Cost to come
        self.C2G = C2G  # Cost to goal
        self.Node_Index = Node_Index # Node index
        self.Parent_Index = Parent_Index # parent node index
        self.Node_State = State  # Node state - coordinate values
        global no_of_nodes
        node_class.all_states.append(self)
        no_of_nodes += 1
    
    # function for representation
    def __repr__(self):
        return f"{self.C2C}, {self.C2G}, {self.Node_Index}, {self.Parent_Index}, {self.Node_State}" # Defining representation of object to be its state, index and parent index
      
    # function fot comparision  
    def __lt__(self, other):
        # First, try to compare based on total cost
        if weight*self.C2G + self.C2C != weight*other.C2G + other.C2C:
            return weight*self.C2G + self.C2C < weight*other.C2G + other.C2C
        # C2G comparision
        elif self.C2G != other.C2G:
            return self.C2G < other.C2G
        # If total cost and C2G are equal, then compare based on C2C
        return self.C2C < other.C2C

    #function returning a child, for input parent and action
    def child(self,ul,ur):
        ontheway=[]
        Node_State = (self.Node_State).copy()
        total = 0
        step =0
        dt = timestep/11 # total of 10 lines
        Xn = Node_State[0]
        Yn = Node_State[1]
        thetan = Node_State[2]
        thetan = (3.14/180)*thetan 
        ontheway.append((Xn, 299-Yn)) # only for display
        while total<11:
            total += 1 # total no of lines that give a curve
            Xs = Xn
            Ys = Yn
            thetan += (wheel_radius/track_width)*(ur - ul)*dt
            Xn += (wheel_radius/2)*(ul+ur)*np.cos(thetan)*dt # updating x
            Yn += (wheel_radius/2)*(ul+ur)*np.sin(thetan)*dt # updating y
            ontheway.append((Xn, 299-Yn)) # only for display
            step+= np.sqrt((Xn-Xs)**2+(Yn-Ys)**2) # calculating length of curve

            # pygame.draw.line(window, (0,0,0), (Xs, Ys), (Xn, Yn) , width=1)

        # if ul == ur:
        #     step = step*0.8
        thetan = (180/3.14)*thetan # theta in degrees
        new_state = [Xn, Yn, thetan] # final state and orentation
        return new_state, ontheway, step # reutning new state, all points on  the way, step= length of curve
    

    #checking if a node is goal node
    def ifgoalstate(self, Goal_Threshold, Goal_State):
        # finding distance from goal state to current, considering goal threshold
        dist = np.sqrt((self.Node_State[0]-Goal_State[0])**2+(self.Node_State[1]-Goal_State[1])**2)
        if dist < Goal_Threshold:
            return True
        else:
            return False


def backtrack(closedlist): # Function for backtracking
    print(" Backtracking ....")
    states=[] # list to store all points that fall in the way
    parentindex = closedlist[-1][1] # parent state of goal node
    states.append(closedlist[-1][2])
    while parentindex != 0: #stopping when parent state is start state
        for node in closedlist:
            if node[0] == parentindex: #if parentstate equal to a node state from closed list
                # print(parentstate)
                states.append(node[2]) #storing all parent states
                parentindex = node[1]
                break
            
    states.append(sc(start_state))
    states.reverse() # reversing the list
    return states

# fuction for state correction to put nodes in any one of the node in 650*200*12
def sc(Node):
    state=Node.copy()
    # state[0], state[1], state[2] = state[0], state[1], state[2]
    if state[0]%1 <= 0.5: # approximating x value
        state[0] = int(state[0]//1)
    else:
        state[0] = int((state[0]//1)+1)
    if state[1]%1<= 0.5: # approximating y value
        state[1] = int(state[1]//1)
    else:
        state[1] = int((state[1]//1)+1)
    if state[2]%30 <=15: # approximating theta
        state[2] = int(state[2] - state[2]%30)%360
    else:
        state[2] = int(state[2] + 30 - state[2]%30)%360
    state[2] = state[2]//30    
    return state # returing corrected state


#_________________________________________________________________________________________________________________________________________________________
#               END OF FUNCTIONS FOR ACTIONS AND BACKTRACKING 

#Defining Obstacle Space
# print("****************** Map *****************")

# print("Input Clearence Value ---")
# clearence = 
# int(input(" "))
clearence = 22


# print(" \nPlease wait, Preparing Map ..... ")


#OBSTACLE SPACE FOR EXAMPLE MAP
def obstacle(x,y):
    check = 0
    check +=1 if x>=60 and x<90 and y >= 40 and y< 70 else 0 
    check +=1 if x>=210 and x<240 and y >= 40 and y< 70 else 0 
    check +=1 if x>=210 and x<240 and y >= 210 and y< 240 else 0 
    check +=1 if x>=360 and x<390 and y >= 40 and y< 70 else 0 
    check +=1 if x>=360 and x<390 and y >= 210 and y< 240 else 0 
    check +=1 if x>=510 and x<540 and y >= 40 and y< 70 else 0 
    check +=1 if x>=510 and x<540 and y >= 210 and y< 240 else 0 
    
    check +=1 if x>=150 and x<152 and y >= 120 and y< 300 else 0 
    check +=1 if x>=450 and x<452 and y >= 120 and y< 300 else 0 
    check +=1 if x>=300 and x<302 and y >= 0 and y< 180 else 0 
    return check


matrix = np.zeros((650,300,12)) # Defining a matrix representing canvas 1200 x 500 with zeros

for i in range(650): # looping through all elements in matrix
        for j in range(300):
            if obstacle(i,j) != 0:  # element changes to 1 if index shows obstacle space
                for k in range(12):
                    matrix[i,j,k]=1 # 1 means obstacle


# print("created normal map")
# looking for corner pixels
corner_obstacle_pixels=[]
for i in range(650): # looping through all elements in matrix
        for j in range(300):
            # if i==0 or i==649 or j==0 or j==299:
                # corner_obstacle_pixels.append((i,j))
            try:
              if matrix[i,j,0] == 1:
                # if matrix[i,j-1,0] == 0 or matrix[i,j+1,0] == 0 or matrix[i-1,j,0] == 0 or matrix[i+1,j,0] == 0:
                if matrix[i-1,j,0] == 0 and matrix[i,j-1,0] == 0 or matrix[i+1,j,0] == 0 and matrix[i,j+1,0] == 0 or matrix[i-1,j,0] == 0 and matrix[i,j+1,0] == 0 or matrix[i+1,j,0] == 0 and matrix[i,j-1,0] == 0:
                    corner_obstacle_pixels.append((i,j))
            except:
               pass

print(len(corner_obstacle_pixels))
# print("checkpoint 1")
# TO have 5mm clearence means no pixel which is in distance of 5 from obstacle, should be avoided
# Below loop checks for such pixels and adjusts their matrix value to 2, which means obstacle


for i,j in corner_obstacle_pixels:
              #if this element in obstacle space is in the corners of obstacle, (to save computation)
                for i1 in range(i-clearence, i+1+clearence):   # clearence of 5         
                    for j1 in range(j-clearence,j+1+clearence):
                          try:
                            if matrix[i1,j1, 0]==0: # if its a unassigned pixel 
                             if ((i1-i)**2 + (j1-j)**2) <= clearence**2: # circle radius check   
                              for k in range(12):
                                matrix[i1,j1,k]=2 # assign it to 2
                          except:
                              pass


for i in range(150-clearence, 152+clearence): # looping through all elements in matrix
        for j in range(120,300):
            for k in range(12):
                matrix[i,j,k]=2

for i in range(450-clearence, 452+clearence): # looping through all elements in matrix
        for j in range(120,300):
            for k in range(12):
                matrix[i,j,k]=2

for i in range(300-clearence, 302+clearence): # looping through all elements in matrix
        for j in range(0,180):
            for k in range(12):
                matrix[i,j,k]=2



# print("done bloating")
#_________________________________________________________________________________________________________________________________________________________
#END OF OBSTACLE SPACE

# Defining initial and final nodes
# invalid_start = True
# while invalid_start:
#     print("\n_____START NODE ______")
#     start_node_input_x = int(input("Start Node 'X' : "))
#     start_node_input_y = int(input("Start Node 'Y' : "))
#     start_node_input_0 = int(input("Start Node 'theta' : "))//30
#     if start_node_input_x>=0 and start_node_input_x<650 and start_node_input_y>=0 and start_node_input_x<200 and matrix[start_node_input_x, start_node_input_y, start_node_input_0] == 0:
#             invalid_start = False
#             start_state = [start_node_input_x, start_node_input_y, start_node_input_0]
#     else:
#         print("Invalid Start Node, Input again")


# invalid_goal = True
# while invalid_goal:
#     print("\n_____GOAL NODE ______")
#     goal_node_input_x = int(input("Goal Node 'X' : "))
#     goal_node_input_y = int(input("Goal Node 'Y' : "))
#     # goal_node_input_0 = int(input("Goal Node 'theta' : "))//30
#     goal_node_input_0 = 0
#     if goal_node_input_x>=0 and goal_node_input_x<6500 and goal_node_input_y>=0 and goal_node_input_y<2000 and matrix[goal_node_input_x//10, goal_node_input_y//10, 0] == 0:
#             invalid_goal = False   
#             goal_state = [goal_node_input_x//10, goal_node_input_y//10, goal_node_input_0]
#     else:
#         print("Invalid Goal Node, Input again")
                          
# print("Process finished --- %s seconds ---\n" % (time.time() - start_time)) # DIsplays run time
# print("__________________________")
# print("  Nodes Accepted  \n")

# rpm1 = 37
# rpm2 = 18

# rpm1 = int(input("Input RPM 1  : "))   
# rpm2 = int(input("Input RPM 2  : "))   

rpm1 = 35
rpm2 = 75

#conevrting rpm to angular velocities
u1 = rpm1*(2*np.pi/60)
u2 = rpm2*(2*np.pi/60)

#declaring variables
goal_treshold = 10 # Goal_ threshold
timestep = 1

# actions = [[0, u1],[0,u2],[u1,u2],[u1,u1],[u2,u2],[u2,u1],[u2,0],[u1,0]]
actions = [[u1, u2],[u2, u2],[u2, u1]] 

print("__________________________")
# print("  RPM Accepted  ")
# print("\n  Computing Path .....  ")

#_________________________________________________________________________________________________________________________________________________________
#END OF DEFINING START AND END GOAL

#Defining initial and final nodes

start_state = [0,150,0] #state of start point
goal_state = [645,90,0] #state of Goal point

# start_state = [50,100,0] #state of start point
# goal_state = [goal_state[0]/10,goal_state[1]/10] #state of Goal point


closed_list = [] # closed list stores ecplored states, containing its index, parent index and corrected state
open_heap = [] # heap for open list
open_dict = {} # dict | key = child state | value = Parent object of node_class class 


from_action={} # dict | key = child state | value = the action from which this is resulted -> gazebo needs this
all_ontheway=[] # stores explored states points for curve, used for printing states in curves
all_children_plot_dict = {} #dict | key = parent node | value = all children curves5



start_node=node_class(0,0,no_of_nodes,0,start_state) # start node object
from_action[tuple(start_state)]=actions[1]

open_dict[tuple(start_state)]=start_node
open_heap.append(start_node) # append to open heap list

#MATRIX CONVENTIONS
#    0 = unexplored
#    1 = obstacle
#    2 = bloat
#    3 = closed list
#    4 = in open list
#_____________________________________________________________________________________________________________________________________________________
#    START OF LOOP

print("A* . . . .")
a=0
loop = True
while loop: 
    if len(open_heap) !=0: # cheking if open list is empty
        current_node = heapq.heappop(open_heap) #extracting node object with least total cost
    else:
        print("No Solution Found") # returning no solution if open list is empty
        break
    
    current_state_corrected = sc(current_node.Node_State) # correcting current node state

     # Marking closed list in MATRIX CONVENTION
    matrix[current_state_corrected[0],current_state_corrected[1], current_state_corrected[2]] = 3
    closed_list.append([current_node.Node_Index, current_node.Parent_Index, tuple(current_state_corrected)]) # adding popped node into closed list    #closed list format

    if node_class.ifgoalstate(current_node, goal_treshold, goal_state): #checking if the current state is goal state
        print("\nGoal Reached")
        points_list = backtrack(closed_list)
        break

    # children_plot=[] # curve plots for all children
    for action in actions: # for all possible actions
      child, ontheway, step = current_node.child(action[0],action[1]) # extracting child state, its curve plot, cost - curve length
    #   children_plot.append(ontheway) # for explored states plotting
      child_corrected = sc(child) #corrected state for children
      
      if 0<= child_corrected[0] < 650 and 0<= child_corrected[1] <200: 

            # if action == from_action[tuple(current_state_corrected)]:
            #     step = step*0.95

            # checking if child is in open list, this means its not in obstacle
            if matrix[child_corrected[0],child_corrected[1],child_corrected[2]] == 4:
                    existing_node = open_dict[tuple(child_corrected)] # extracting child node object, if in open list

                    if current_node.C2C + step < existing_node.C2C:  # comparing cost to come
                        # from_action[tuple(child_corrected)]= action  # updating its from action
                        existing_node.C2C = current_node.C2C + step  # updating its cost to come
                        existing_node.Parent_Index = current_node.Node_Index  # updating its parent state       

            # if not in open list, then it should be unexplored node
            elif matrix[child_corrected[0],child_corrected[1],child_corrected[2]] == 0: # if the state is not in open list, then creating a state, adding it to open list
                    # from_action[tuple(child_corrected)]= action # updating its from action
                    # all_ontheway.append(ontheway) # storing its curve plots
                    # finding distance, C2G
                    dist = np.sqrt((child_corrected[0]-goal_state[0])**2+(child_corrected[1]-goal_state[1])**2) 
                    # Creating new child object
                    new_node = node_class(current_node.C2C+step, dist, no_of_nodes, current_node.Node_Index, child)
                    # adding this to open dict and open heap - open lists
                    open_dict[tuple(child_corrected)]=new_node
                    heapq.heappush(open_heap, new_node)
                    # udating matrix convention to ' open node '
                    matrix[child_corrected[0],child_corrected[1],child_corrected[2]] = 4
    
    #plots for all children, for current state
    # all_children_plot_dict[tuple(current_state_corrected)]=children_plot

#________________________________________________________________________________________________________________
#    END OF LOOP
print("Process finished in --- %s seconds ---\n" % (time.time() - start_time)) # DIsplays run time


# #_________________________________________________________________________________________________________________________________________________________
# #    INITIALIZING PYGAME

# print("No of explored ",no_of_nodes)

# pygame.init()
# #initializing window
# window = pygame.display.set_mode((650,300)) # window size
# window.fill((255,255,255)) # filling it with color
# #initializing color
# white=(230,230,230)
# black = (50,50,50)
# grey = (150,150,150)
# red = (205,50,50)
# blue = (105,135,235)

# # LOOP to transform matrix into this window
# for i in range(650):
#     for j in range(300):
#             if matrix[i,j,0]==1: # 1 -> red color showing obstacles
#                 window.set_at((i,299-j),red)
#             elif matrix[i,j,0]==2: # 2-> black showing bloating part
#                 window.set_at((i,299-j),black)
#             elif matrix[i,j,0]==5:
#                 window.set_at((i,299-j),grey)

# pygame.display.flip() #updating window

# #______________________________________WINDOW IS CREATED
# Loop to reflect all explored nodes from closed list to pyagme windoe
# for i,oneset in enumerate(all_ontheway):
#         pygame.draw.lines(window, blue, False, oneset, width = 1 )
#         if i%150==0: # rate to reflect the updates on the window
#             pygame.display.flip()

# plotting start node
# pygame.draw.circle(window,black,(start_state[0],299-start_state[1]),5)
# pygame.draw.circle(window,(50,220,50),(start_state[0],299-start_state[1]),3)
# time.sleep(1)

# loop to print all states in the path
# for i in points_list[:-1]:
#     all_children = all_children_plot_dict[tuple(i)] # to get all 8 curves
#     for points_set in all_children: 
#         pygame.draw.lines(window, black, False, points_set, width = 1 )
#         pygame.display.flip()


# for i in range(len(points_list)-1):
#     # all_children = all_children_plot_dict[tuple(i)] # to get all 8 curves
#     # for points_set in all_children: 
#         pygame.draw.line(window, red, tuple((points_list[i][0],299 - points_list[i][1])),tuple((points_list[i+1][0], 299 - points_list[i+1][1])) ,width = 1 )
#         pygame.display.flip()

# loop for printing the path
# action_list=[]
# for i in range(len(points_list)-1):
#     camefrom_action = from_action[tuple(points_list[i+1])]
#     action_list.append(camefrom_action)

# plotting goal state
# pygame.draw.circle(window,black,(goal_state[0],299-goal_state[1]),5) #plotting end node
# pygame.draw.circle(window,red,(goal_state[0],299-goal_state[1]),3)
# pygame.display.update()

# print("Points list")
# print(points_list)
# print("len points list - ",len(points_list))
# print("No of Actions :",len(action_list))
# print(action_list)
# action_list.append([0,0])
# print("Weight ", weight)
print(" ------------------------ SUCCESSFULLT TRACKED ------------------------")
# print("------- A*-------------")

#LOOP to keep py game running untill manually closed

# run = True
# while run: 
#     for event in pygame.event.get():
#         if event.type ==pygame.QUIT:
#     # time.sleep(5)
#             run = False
#         pygame.display.update()

# # time.sleep(5)
# pygame.quit()
#_________________________________________________________________________________________________________________________________________________________
#    END OF PYGAME