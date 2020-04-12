import sys
import cv2
import numpy as np
import heapq
import time
import math
global l, final_path, img, vidWriter, node_cnt,trsh,step_size,theta, a
sys.setrecursionlimit(10**9)
#sys.settrace(exception)


class node:
    
    def __init__(self, location, parent, move):
        global a, node_cnt,angle_thrsh
        self.loc = location
        self.iloc = [int(round(location[0])), int(round(location[1])), threshold(check_round(location[2]))]
        self.value_to_come = a.map[int(round(location[0]))][int(round(location[1]))][angle_thrsh+1]
        self.value_to_go = a.map[int(round(location[0]))][int(round(location[1]))][angle_thrsh+2]
        self.move = move
        self.parent = parent
        self.counter = node_cnt
        node_cnt += 1


class MapMake:

    def __init__(self, width_x, length_y):
        global angle_thrsh
        self.width_x = width_x
        self.length_y = length_y
        self.map = np.zeros([width_x, length_y, angle_thrsh+3])
        self.map[:,:, angle_thrsh+1] = np.inf                   # stores the cost to come
        self.map[:,:,angle_thrsh+2] = np.inf                    # stores the cost to go

    def circle_obstacle(self, xpos, ypos, radius):  # makes a circle obstacle
        for i in np.arange(xpos-radius,xpos+radius,1):
            for j in np.arange(ypos-radius,ypos+radius,1):
                if np.sqrt(np.square(ypos - j) + np.square(xpos - i)) <= radius:
                    self.map[i, j, angle_thrsh] = 1
                    img[i,j,0:3] = [0,0,255]

    def oval_obstacle(self, xpos, ypos, radius_x, radius_y):  # makes oval obstacle
        for i in np.arange(xpos-radius_x,xpos+radius_x,1):
            for j in np.arange(ypos-radius_y,ypos+radius_y,1):
                first_oval_term = np.square(i - xpos) / np.square(radius_x)
                second_oval_term = np.square(j - ypos) / np.square(radius_y)
                if first_oval_term + second_oval_term <= 1:
                    self.map[i, j, angle_thrsh] = 1
                    img[i,j,0:3] = [0,0,255]

    def triangle_obstacle(self,three_points): # makes triangle obstacle
        v1 = three_points[0]
        v2 = three_points[1]
        v3 = three_points[2]
        min_point,max_point = max_and_min(three_points)
        for i in np.arange(min_point[0],max_point[0],1):
            for j in range(int(min_point[1]),int(max_point[1]),1):
                pt = [i,j]
                d1 = sign(pt, v1, v2)
                d2 = sign(pt, v2, v3)
                d3 = sign(pt, v3, v1)
                neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
                pos = (d1 > 0) or (d2 > 0) or (d3 > 0)

                if not (neg and pos):
                    a.map[int(i)][int(j)][angle_thrsh] = 1
                    img[int(i), int(j), 0:3] = [0, 0, 255]

    def clearance(self, clearance_distance):

        self.map[self.width_x-clearance_distance:self.width_x,:,angle_thrsh] = 2  # makes edge clearance in map
        self.map[0:clearance_distance,:,angle_thrsh] = 2
        self.map[:, 0:clearance_distance, angle_thrsh] = 2
        self.map[:, self.length_y-clearance_distance:self.length_y, angle_thrsh] = 2

        img[0:clearance_distance, :, 0:3] = [0, 69, 200]                 # makes edge clearance in image
        img[self.width_x-clearance_distance:self.width_x, :, 0:3] = [0, 69, 200]
        img[:, 0:clearance_distance, 0:3] = [0, 69, 200]
        img[:, self.length_y-clearance_distance:self.length_y, 0:3] = [0, 69, 200]

        obstacles = np.where(self.map[:, :, angle_thrsh] == 1)
        obstacles = np.array(obstacles)
        obstacles = obstacles.T  # get all points that are in obstacles

        circle_list = []     # makes all the points that falls within a circle and uses it to find clearance
        for i in np.arange(-clearance_distance,clearance_distance,1):
            for j in np.arange(-clearance_distance,clearance_distance,1):
                dist = np.sqrt(np.square(i) + np.square(j))
                if  dist <= clearance_distance:
                    circle_list.append([i,j])

        for obstacle_point in obstacles:
            bound_list = obstacle_point+circle_list
            for bound in bound_list:
                if a.map[bound[0], bound[1],angle_thrsh] == 0:
                    a.map[bound[0], bound[1],angle_thrsh] = 2
                    img[bound[0], bound[1], 0:3] = [0, 69, 200]


def sign(point1,point2,point3):
    return (point1[0]-point3[0])*(point2[1]-point3[1])-(point2[0]-point3[0])*(point1[1]-point3[1])


def max_and_min(point_list):
    point_list = np.array(point_list)
    max_x = max(point_list[:,0])
    min_x = min(point_list[:,0])
    max_y = max(point_list[:,1])
    min_y = min(point_list[:,1])
    return (min_x,min_y),(max_x,max_y)


def define_map(clear_r):
    global a
    global trsh
    a = MapMake(10*trsh, 10*trsh)
    a.circle_obstacle(5*trsh, 5*trsh, 1*trsh)
    a.circle_obstacle(7*trsh, 8*trsh, 1*trsh)
    a.circle_obstacle(3*trsh, 2*trsh, 1*trsh)
    a.circle_obstacle(7*trsh, 2*trsh, 1*trsh)

    a.triangle_obstacle(([2.25*trsh,8.75*trsh],[3.75*trsh,8.75*trsh],[3.75*trsh,7.25*trsh]))
    a.triangle_obstacle(([2.25*trsh,8.75*trsh],[2.25*trsh,7.25*trsh],[3.75*trsh,7.25*trsh]))

    a.triangle_obstacle(([0.25*trsh,5.75*trsh],[1.75*trsh,5.75*trsh],[1.75*trsh,4.25*trsh]))
    a.triangle_obstacle(([0.25*trsh,5.75*trsh],[0.25*trsh,4.25*trsh],[1.75*trsh,4.25*trsh]))

    a.triangle_obstacle(([8.25*trsh,5.75*trsh],[9.75*trsh,5.75*trsh],[9.75*trsh,4.25*trsh]))
    a.triangle_obstacle(([8.25*trsh,5.75*trsh],[8.25*trsh,4.25*trsh],[9.75*trsh,4.25*trsh]))

    if clear_r != 0: # puts in clearance and radius if present
        a.clearance(clear_r)
    return


def visualize_map():   # a function to visualize the initial map generated with just obstacles
    global img
    cv2.imshow('map',cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE))
    cv2.waitKey()


def point_in_obstacle(point):  # checks is a point is in an obstacle
    if a.map[point[0]][point[1]][angle_thrsh ] != 0:
        return True
    else:
        return False

def check_round(angle):
    if angle >= 360:
        angle = angle - 360
    elif angle < 0:
        angle = angle + 360
    return angle

def threshold(angle):
    global angle_thrsh, angle_res          # eg: if angle_thrsh = 30 and angle = 316, expected output is 330.  
    if ( (angle%angle_res - (angle_res/2)) > 0 ):        # if ( 16 - 15 > 0)
        # print(1, angle)
        angle = (angle - angle%angle_res + angle_res)    #   316 - 16 + 30
        # print(2, angle)
    else:
        angle = angle - angle%angle_res                   
    return angle         

def find_pt(x, y, th, Lrpm, Rrpm, plot = False):
    global bot_r, bot_L,trsh
    # **********
    t = 1
    x_ = x
    y_ = y
    th_ = th
    amove = []
    move = []
    x_moves = []
    y_moves = []
    cost = 0
    Lrpm = (Lrpm*2*np.pi)/60
    Rrpm = (Rrpm*2*np.pi)/60
    while(t>0):
        x_ += trsh*(0.5 * bot_r * ( Lrpm + Rrpm) * np.cos(np.deg2rad(th_)) * 0.1)
        y_ += trsh*(0.5 * bot_r * ( Lrpm + Rrpm) * np.sin(np.deg2rad(th_)) * 0.1)
        th_ += (180/np.pi)*((bot_r/bot_L) * (Rrpm - Lrpm) * 0.1)
        t -= 0.1
        x_moves.append(int(round(x_)))
        y_moves.append(int(round(y_)))

    amove += [(x_)]
    amove += [(y_)]
    amove += [(check_round(th_))] # ------->>>>>>>>>>
    
    move += [int(round(x_))]
    move += [int(round(y_))]
    move += [threshold(check_round(th_))]
    # print (amove, move)
    # input()
    # print (amove)
    if plot == True:
        return x_moves, y_moves
    return amove,move#,cost         

def allowable_check(point):
    global a,rpm1, rpm2, angle_thrsh, angle_res
    # cost = []
    amoves = []
    moves = []
    v,w = find_pt(point[0], point[1], point[2], 0, rpm1)
    amoves += v
    moves += w
    # cost.append(c)
    v,w = find_pt(point[0], point[1], point[2], rpm1, 0)
    amoves += v
    moves += w
    # cost.append(c)
    v,w = find_pt(point[0], point[1], point[2], rpm1, rpm1)
    amoves += v
    moves += w
    # cost.append(c)
    v,w = find_pt(point[0], point[1], point[2], rpm2, 0)
    amoves += v
    moves += w
    # cost.append(c)
    v,w = find_pt(point[0], point[1], point[2], 0, rpm2)
    amoves += v
    moves += w
    # cost.append(c)
    v,w = find_pt(point[0], point[1], point[2], rpm2, rpm2)
    amoves += v
    moves += w
    # cost.append(c)
    v,w = find_pt(point[0], point[1], point[2], rpm1, rpm2)
    amoves += v
    moves += w
    # cost.append(c)
    v,w = find_pt(point[0], point[1], point[2], rpm2, rpm1)
    amoves += v
    moves += w
    # cost.append(c)

    # *******
    moves = np.reshape(moves,(8,3)).astype(int)
    amoves = np.reshape(amoves,(8,3))
    idxs = np.arange(8)
    # cost = np.asarray(cost)
    #print(moves)
    allowable_actions = []
    allowable_ids = []
    # allowable_cost = []
    for move,amove, idx in zip(moves,amoves, idxs):
        #print(move[0],move[1],int(move[2]/angle_thrsh))
        # print(int(move[2]/angle_res))
        # print(move[0], move[1], int(round(move[2]/angle_res)),a.map[move[0],move[1],int(round(move[2]/angle_res))], "allowable")
        # print(move[0],move[1],int(round(move[2]/angle_res), a.map[move[0],move[1]]))

        if a.map[move[0],move[1],int(round(move[2]/angle_res))]==0:  #check if visited
            # print("done")
            if a.map.shape[0]>move[0] >=0:              #check if on map X
                if a.map.shape[1] > move[1] >= 0:       #check if on map Y
                    if a.map[move[0],move[1],angle_thrsh] == 0:    #check if obstacle
                        allowable_actions.append(amove)
                        allowable_ids.append(idx)
                        # allowable_cost.append(ct)  

    # print((allowable_actions))
    return allowable_actions, allowable_ids

def is_goal(curr_node):              # checks if the current node is also the goal
    global trsh
    #if curr_node.loc[0] == end_pt[0] and curr_node.loc[1] == end_pt[1]:
    #    return True
    if (curr_node.loc[0]-end_pt[0])**2 + (curr_node.loc[1]-end_pt[1])**2 < trsh:
        return True


def find_path(curr_node): # A function to find the path until the root by tracking each node's parent

    global final_path
    while(curr_node!=None):
        final_path.insert(0, curr_node)
        curr_node = curr_node.parent
    # print (final_path)    
    for i, value in enumerate(final_path):
        img[value.iloc[0], value.iloc[1], 0:3] = [255,0,0]
        action = 0
        curr = value
        action  = curr.move
        if i !=0:
            nodes_file.write(str(prev_loc[0])+ "," + str(prev_loc[1]) + "," + str(prev_loc[2]) + "," + str(action) + "\n" )
            x_intr , y_intr = find_pt(prev_loc[0], prev_loc[1], prev_loc[2], action[0], action[1], plot = True)
            img[x_intr, y_intr, 0:3] = [255, 0, 0]
        prev_loc = curr.loc
        
        for j in range(10):
            vidWriter.write(cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE))
    nodes_file.close()
    vidWriter.release()         
    return


def find_children(curr_node):
    global a
    test_node = curr_node
    child_loc, ids = allowable_check(curr_node.loc)
    # gets allowable cardinal moves
    # print(len(child_loc))
    #child_cost_to_come = test_node.value_to_come + step_size            # square move cost
    # print (child_cost_to_come)
    children_list = []
    for idx, state_loc in zip(ids, child_loc):
        #print(state_loc,curr_node.loc)
        go_cost = math.sqrt((state_loc[0]-end_pt[0])**2 + (state_loc[1]-end_pt[1])**2)
        child_cost_to_come = curr_node.value_to_come + math.sqrt((state_loc[0]-curr_node.loc[0])**2 + (state_loc[1]-curr_node.loc[1])**2)
        # child_cost_to_come = state_cost
        # print(i, a.map[int(state_loc[0])][int(state_loc[1])][angle_thrsh+2] + a.map[int(state_loc[0])][int(state_loc[1])][angle_thrsh+3])
        # print( i, state_loc[0], state_loc[1], child_cost_to_come, go_cost )
        
        # input()
        if (a.map[int(round(state_loc[0]))][int(round(state_loc[1]))][angle_thrsh+1] + a.map[int(round(state_loc[0]))][int(round(state_loc[1]))][angle_thrsh+2] >= child_cost_to_come + go_cost):  # if the child cost is less from the current node
            # print(i)
            a.map[int(round(state_loc[0]))][int(round(state_loc[1]))][angle_thrsh+1] = child_cost_to_come              # update map node to lesser cost
            a.map[int(round(state_loc[0]))][int(round(state_loc[1]))][angle_thrsh+2] = go_cost
            # print(which_move(idx))
            # input()
            child_node = node(state_loc,curr_node, which_move(idx))              # create new child node
            add_image_frame(child_node, curve = True)
            # print("a*", child_node.value_to_come ,child_node.value_to_go, state_loc[0], state_loc[1], state_loc[2])
            children_list.append((child_node.value_to_come + child_node.value_to_go, child_node.counter, child_node))
    # print(len(children_list))
    return children_list  # list of all children with a lesser cost for the current node

def which_move(idx):
    global rpm1, rpm2
    moves = {
        0 : (0, rpm1), 
        1 : (rpm1, 0),
        2 : (rpm1, rpm1),
        3 : (rpm2, 0),
        4 : (0, rpm2),
        5 : (rpm2, rpm2),
        6 : (rpm1, rpm2),
        7 : (rpm2, rpm1)
    }
    return moves[idx]

def add_image_frame(curr_node, curve = False): # A function to add the newly explored state to a frame. This would also update the color based on the cost to come
    global img, vidWriter, ctr
    if curve == True:
        node = curr_node.parent
        x_intr, y_intr = find_pt(node.loc[0], node.loc[1], node.loc[2], curr_node.move[0], curr_node.move[1], plot = True)
        # print(x_intr, y_intr)
        img[x_intr, y_intr, 0:3] = [0, 255, np.min([100 + curr_node.value_to_come*2, 255])]

    elif curve == False:
        img[curr_node.iloc[0], curr_node.iloc[1],0:3] = [0,255, np.min([100 + curr_node.value_to_come*2, 255])] #np.min([200 + curr_node.value_to_come*2, 255])

    # if ctr == 40:
    #     draw_vector(curr_node)
    #     ctr = 0
    #     # cv2.imshow("test", cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE))
        # cv2.waitKey(0)
    vidWriter.write(cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE))
    ctr = ctr + 1
    return
    

def solver(curr_node):  # A function to be recursively called to find the djikstra solution
    global a
    # counter = 0
    while(1):
        # counter +=1
        #visitedNode.update({curr_node: "s"})
        a.map[curr_node.iloc[0],curr_node.iloc[1],int((curr_node.iloc[2]/angle_res))]=1 # --------->>>
        # print("popped element", curr_node.loc[0], curr_node.loc[1],curr_node.loc[2])
        # print(a.map[curr_node.iloc[0],curr_node.iloc[1],int((curr_node.iloc[2]/angle_res))], "map check")
        # print(a.map[30,40,0])
        # print(curr_node.value_to_come)
        global l
        if (is_goal(curr_node)):
            print("Path found, Plotting optimal path.") 
            find_path(curr_node) # find the path to the start node by tracking the node's parent
            break
        add_image_frame(curr_node)
        children_list = find_children(curr_node) # a function to find possible children and update cost
        
        l = l + children_list                  # adding possible children to the list
        # print(l)
        if (l == []):
            return 0
        heapq.heapify(l)                    # converting to a list
        # print(l)
        curr_node = l.pop(0)[2]            # recursive call to solver where we pass the element with the least cost 
        # print("popped element - 0", curr_node.loc[0], curr_node.loc[1],curr_node.loc[2])
    return 1        


if __name__=="__main__":
    global start_pt
    global end_pt
    global vidWriter
    global path
    global node_cnt
    global final_path
    global visitedNode
    global step_size
    global theta
    global trsh
    global ctr
    global rpm1, rpm2
    global bot_r, bot_L
    global angle_thrsh, angle_res
    global a

    trsh = 20
    #step_size = 1*trsh
    #theta = 30
    ctr = 0
    node_cnt = 0
    final_path = []
    visitedNode = {}
    vidWriter = cv2.VideoWriter("Astar_curve.mp4", cv2.VideoWriter_fourcc(*'mp4v'), 240, (int(round(10*trsh)),int(round(10*trsh)))) # --->>>
    img = np.zeros([10*trsh,10*trsh,3], dtype=np.uint8)
    img[:,:,0:3] = [0,255,0]
    nodes_file = open("nodes_optimal.txt", 'w')
    nodes_file.write("X_Pos, Y_Pos, Theta, Action \n")
    # cv2.arrowedLine(img, (50, 50),(100, 100), (255, 144, 30), thickness = 5) 
    # cv2.imshow("try", img)
    # cv2.waitKey(0)
    rpm1,rpm2,robot_r,clear_r,theta = 40,50,1,1,0
    #rpm1 = int(input("Enter RPM1 for the robot: "))
    #rpm2 = int(input("Enter RPM2 for the robot: "))
    #robot_r = int(input("Enter robot radius (to be harcoded from datasheet): ")) 
    bot_r = 0.038
    bot_L = 0.354
    angle_thrsh = int(360/1)
    angle_res = 1
    # angle_thrsh = int(360/angle_res)
    print (angle_thrsh)
    print ( angle_res)
    #print(angle_thrsh)
    #clear_r = int(input("Enter the clearance: "))
    #step_size = int(input("Enter step size: "))
    #step_size = step_size*trsh
    #theta = int(input("Enter start orientation in degrees: "))
    total_clear = robot_r+clear_r
    define_map_start = time.time()
    define_map(total_clear)
    t1 = time.time()-define_map_start
    print("Time to define map: " + str(t1))
    solve_problem_start = time.time()
    # visualize_map()

    start_pt = [1,2]
    end_pt = [9,8]
    valid_points = False
    while  valid_points == False:
        #start_pt = (input("Enter start point in form # #: "))
        #start_pt = [int(start_pt.split()[0]), int(start_pt.split()[1])]
        theta = 0
        start_pt = [int(trsh*start_pt[0]), int(trsh*start_pt[1]), theta ]
        img[start_pt[0]][start_pt[1]][0:3] = [0,0,0]
        end_pt = [int(trsh*end_pt[0]), int(trsh*end_pt[1]), 0 ]
        #end_pt = (input("Enter end point in form # # : "))
        #end_pt = [trsh*int(end_pt.split()[0]), trsh*int(end_pt.split()[1]), 0]
            # print (end_pt)
        img[end_pt[0]][end_pt[1]][0:3] = [0,0,255]
        if(point_in_obstacle(start_pt) or point_in_obstacle(end_pt)): # check if either the start or end node an obstacle
            print("Enter valid points... ")
            continue
        else:
            valid_points = True
    
    a.map[start_pt[0], start_pt[1], 1] = 0
    print(start_pt)
    print(end_pt)
    # input()
    # create start node belonging to class node
    start_node = node(start_pt,None, (0,0))
    start_node.value_to_come = 0

    global l
    l = [(start_node.value_to_come, start_node.counter, start_node)]

    # define a priority queue and add first element
    heapq.heapify(l)

    print("Running..")
    # solve using Astar
    flag = solver(heapq.heappop(l)[2])
    # print(flag)
    # if found, visualise the path 
    if flag == 1:
        print("Please watch the video generated.")
        print("Time to define map and solve problem: " + str(time.time() - solve_problem_start + t1))
    # else print path not found    
    else:
        print("Solution not found... ")


