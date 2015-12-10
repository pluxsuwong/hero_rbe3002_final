#!/usr/bin/env python

import actionlib, rospy, roslib, time, math, tf, numpy, copy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from actionlib_msgs.msg import GoalID, GoalStatusArray
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetMap

#---------------------------------------Callback Functions--------------------------------------------

def cost_map_callback(msg):
    global map_width, map_height, cost_map, cell_res, map_origin

    map_width = msg.info.width
    map_height = msg.info.height
    cell_costs = msg.data
    cell_res = msg.info.resolution
    map_origin = msg.info.origin.position

    # Create cost_map
    cost_map = [[cell_costs[y*map_width + x] for y in range(map_height)] for x in range(map_width)]

    detect_frontiers()
    
def cost_map_update_callback(msg):
    global cost_map

    local_map_width = msg.width
    local_map_height = msg.height
    local_cell_costs = msg.data
    local_map_origin = Point(msg.x, msg.y, 0)
    local_grid_origin = world_to_grid(local_map_origin)
    
    i = 0
    for y in range(local_grid_origin.y, local_grid_origin.y + local_map_height):
        for x in range(local_grid_origin.x, local_grid_origin.x + local_map_width):
            if local_cell_costs[i] != 0:
                try:
                    cost_map[x][y] = local_cell_costs[i]
                except IndexError:
                    pass
            i += 1

def local_cost_map_callback(msg):
    global cost_map

    local_map_width = msg.info.width
    local_map_height = msg.info.height
    local_cell_costs = msg.data
    local_map_origin = msg.info.origin.position
    
    try:
        (position, orientation) = odom_list.lookupTransform('odom', 'map', rospy.Time(0))
        local_map_origin.x += position[0]
        local_map_origin.y += position[1]
        local_grid_origin = world_to_grid(local_map_origin)
        
        i = 0
        for y in range(local_grid_origin.y, local_grid_origin.y + local_map_height):
            for x in range(local_grid_origin.x, local_grid_origin.x + local_map_width):
                if local_cell_costs[i] != 0:
                    cost_map[x][y] = local_cell_costs[i]
                i += 1
    except:
        print "Map still not ready"

# Check if turtlebot has 
def move_status_callback(msg):
    global last_active_goal, at_goal
    for goal in msg.status_list:
        if goal.status < 2:
            last_active_goal = goal
            at_goal = False
    if len(msg.status_list) > 0 and not at_goal:
        for goal in msg.status_list:
            if goal.goal_id.id == last_active_goal.goal_id.id:
                if 2 <= goal.status <= 3:
                    at_goal = True
                    print "DEBUG_CB: move_status_callback():", goal.status
                    # Rotate turtlebot 360 deg
                    detect_frontiers()
                    move_to_next_waypoint()
                elif 4 <= goal.status <= 5:  # Goal unreachable or rejected
                    at_goal = True
                    print "DEBUG_CB: move_status_callback():", goal.status
                    # Rotate turtlebot 360 deg
                    detect_frontiers()
                    move_to_next_waypoint()

# Odometry Callback function.
def read_odom(msg):
    global pose
    pose = msg.pose

#-----------------------------------------Helper Functions--------------------------------------------

# (real world) -> (grid)
def world_to_grid(w_pt):
    g_pt = Point()
    g_pt.x = int((w_pt.x - map_origin.x)/cell_res)
    g_pt.y = int((w_pt.y - map_origin.y)/cell_res)
    return g_pt

# (grid) -> (real world)
def grid_to_world(g_pt):
    w_pt = Point()
    w_pt.x = ((g_pt.x*cell_res) + map_origin.x) + (0.5*cell_res)
    w_pt.y = ((g_pt.y*cell_res) + map_origin.y) + (0.5*cell_res)
    return w_pt

# Get cell neighbors
def get_neighbors(cell):
    neighbors = []

    for y in range(cell.y - 1, cell.y + 2):
        for x in range(cell.x - 1, cell.x + 2):
            if y < 0 or x < 0 or (y == cell.y and x == cell.x) or y >= map_height or x >= map_width:
                continue
            else:
                neighbors.append(Point(x, y, 0))

    return neighbors

# Check if cell has cost of -1 and atleast 1 known cell
def is_frontier_cell(cell):
    if cost_map[cell.x][cell.y] < 0:
        cell_neighbors = get_neighbors(cell)
        for n in cell_neighbors:
            if cost_map[n.x][n.y] >= 0:
                return True
    return False

# Merge frontier fragments
def merge_frontiers(frontier_cells):
    frontier_fragments = []
    f_c_buf = copy.deepcopy(frontier_cells)
    print 'DEBUG_1_1: inside merge_frontiers()'
    while len(f_c_buf) > 0:
        f_fragment = [f_c_buf.pop(0)]
        f_c_buf_v = copy.deepcopy(f_c_buf)
        for i in f_c_buf:
            for j in f_fragment:
                if is_neighbor(i, j):
                    if i not in f_fragment:
                        f_fragment.append(i)
                    try:
                        f_c_buf_v.remove(i)
                    except:
                        pass
        f_c_buf = copy.deepcopy(f_c_buf_v)
        tmp_frag = copy.deepcopy(f_fragment)
        frontier_fragments.append(tmp_frag)

    return frontier_fragments

def is_neighbor(i, j):
    result = (abs(i.x - j.x) <= 1) and (abs(i.y - j.y) <= 1)
    return result

# Calculate centroids of frontier fragments
def calc_centroids(frontier_fragments):
    centroids = []
    print 'DEBUG_2_1: inside calc_centroids()'
    '''
    for c_a in frontier_fragments:
        print '{',
        for c_b in c_a:
            print '[', c_b.x, c_b.y, ']',
        print '}'
    '''
    for fragment in frontier_fragments:
        x_sum = 0.0
        y_sum = 0.0
        cells = len(fragment)

        for cell in fragment:
            x_sum += cell.x
            y_sum += cell.y
        x_c = int(x_sum/cells)
        y_c = int(y_sum/cells)
        centroid = Point(x_c, y_c, 0)
        if cost_map[x_c][y_c] < 0 or cost_map[x_c][y_c] > cost_threshold:
            centroids.append(nearest_empty_cell(centroid))
        else:
            centroids.append(centroid)

    return centroids

def nearest_empty_cell(cell):
    explored = [cell]
    unexplored = get_neighbors(cell)

    while True:
        cell_buf = unexplored.pop(0)
        if cell_buf in explored:
            continue
        explored.append(cell_buf)
        if 0 <= cost_map[cell_buf.x][cell_buf.y] <= cost_threshold:
            return cell_buf
        else:
            unexplored.extend(get_neighbors(cell_buf))

def cur_dist_to_cell(cell):
    cur_pos = get_cur_pos()
    return math.sqrt(float((cell.x - cur_pos.x)**2 + (cell.y - cur_pos.y)**2))

def request_map(event):
    get_map_srv = rospy.ServiceProxy('/dynamic_map', GetMap)
    cost_map_callback(get_map_srv().map)

def sort_by_x(cell_list):
    return sorted(cell_list, key=lambda x: x.x, reverse=True)

def get_cur_pos():
    cur_pos = Point()
    cur_pos.x = pose.pose.position.x
    cur_pos.y = pose.pose.position.y
    cur_coords = world_to_grid(cur_pos)
    return cur_coords

def frontier_bfs():
    start = get_cur_pos()

    frontier_cells, visited, queue = [], set(), [start]
    while queue and not rospy.is_shutdown():
        cur_cell = queue.pop(0)
        if cur_cell not in visited:
            visited.add(cur_cell)
            if is_frontier_cell(cur_cell):
                frontier_cells.append(cur_cell)
            elif cost_map[cur_cell.x][cur_cell.y] > cost_threshold:
                pass
            else:
                cur_cell_n = get_neighbors(cur_cell)
                for v in visited:
                    n_visited = False
                    for n in cur_cell_n:
                        if n.x == v.x and n.y == v.y:
                            n_visited = True
                            break
                    if not n_visited:
                        queue.append(n)
    return frontier_cells

#-----------------------------------------Update Grid Functions---------------------------------------

def update_frontier():
    global frontier_pub
    print "DEBUG_0_1: inside update_frontier()"

    ''' Iterate over cost map'''
    frontier_cells = []
    for y in range(map_height):
        for x in range(map_width):
            cell = Point(x, y, 0)
            if is_frontier_cell(cell):
                frontier_cells.append(cell)
    
    
    #frontier_cells = frontier_bfs()
    print "DEBUG_0_2: after loops in update_frontier()"
    frontier_msg = GridCells()
    frontier_msg.header.frame_id = 'map'
    frontier_msg.cell_width = cell_res
    frontier_msg.cell_height = cell_res
    frontier_msg.cells = frontier_cells
    frontier_pub.publish(frontier_msg)

    return frontier_cells

def update_path(waypoints):
    global path_pub

    path_msg = GridCells()
    path_msg.header.frame_id = 'map'
    path_msg.cell_width = cell_res
    path_msg.cell_height = cell_res
    path_msg.cells = waypoints
    path_pub.publish(path_msg)

    return path_msg


#----------------------------------------Navigation Functions-----------------------------------------

# Move to Next Waypoint
def move_to_next_waypoint():
    global move_base
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.header.stamp = rospy.Time.now()
    goal_pose.target_pose.pose.position = grid_to_world(cur_goal)
    # Modify orientation to face movement heading
    dx = goal_pose.target_pose.pose.position.x - pose.pose.position.x
    dy = goal_pose.target_pose.pose.position.y - pose.pose.position.y
    new_ori = math.atan2(dy, dx)
    o_x, o_y, o_z, o_w = quaternion_from_euler(0.0, 0.0, new_ori)
    goal_pose.target_pose.pose.orientation.x = o_x
    goal_pose.target_pose.pose.orientation.y = o_y
    goal_pose.target_pose.pose.orientation.z = o_z
    goal_pose.target_pose.pose.orientation.w = o_w

    # Navigate to goal_pose
    move_base.send_goal(goal_pose)

# Detect Frontier cells
def detect_frontiers():
    global cur_goal
    print "DEBUG_0: inside detect_frontiers()"

    print "DEBUG_1: update_frontiers()"
    frontier_cells = update_frontier()
    
    # Sort frontier_cells by x coord
    sorted_f_cells = sort_by_x(frontier_cells)

    # Group the frontier cells into continuous frontiers and return them as a list
    print "DEBUG_2: merge_frontiers()"
    frontier_fragments = merge_frontiers(sorted_f_cells)

    # Calculate the centroid of all of the frontiers
    print "DEBUG_3: calc_centroids()"
    centroids = calc_centroids(frontier_fragments)

    print "DEBUG_4: update_path()"
    update_path(centroids)

    # Calculate the number of frontier cells in each frontier
    frag_len = map(len, frontier_fragments)

    # Calculate the distance to each centroid
    #distances = map(cur_dist_to_cell, centroids)

    # Weight each centroid by its distance * # of frontier cells
    """
    weighted_centroid = []
    for i in range(len(distances)):
        try:
            weighted_centroid.append(frag_len[i] / distances[i])
        except ZeroDivisionError:
            print 'Division by 0 in detect_frontiers()'
    
    max_wc = 0
    max_i = 0
    for i, wc in enumerate(weighted_centroid):
        if wc > max_wc:
            max_wc = wc
            max_i = i
    """
    max_wc = 0
    max_i = 0
    for i, wc in enumerate(frag_len):
        if wc > max_wc:
            max_wc = wc
            max_i = i

    # The most heavily weighted centroid aka nearest fragment closest to completion
    best_sol = centroids[max_i]
    cur_pos = get_cur_pos()
    cur_goal = Point()
    cur_goal.x = cur_pos.x + int(0.25*(best_sol.x - cur_pos.x))
    cur_goal.y = cur_pos.y + int(0.25*(best_sol.y - cur_pos.y))
    print 'DEBUG_cur_goal_UPDATE:'
    print cur_goal

    print 'frontier cells remaining:', len(frontier_cells)


# -------------------------------------------Main Function---------------------------------------------

if __name__ == '__main__':

    # Create Node
    rospy.init_node('hero_rbe3002_final')

    # Global Variables for Navigation
    global odom_list, at_goal, cur_goal, cost_threshold
    odom_list = tf.TransformListener()
    at_goal = True
    cur_goal = None
    cost_threshold = 30

    # Subscribers
    rospy.Subscriber('/odom', Odometry, read_odom)
    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, cost_map_callback)
    rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, cost_map_update_callback)
    rospy.Subscriber('/move_base/local_costmap/costmap', OccupancyGrid, local_cost_map_callback)
    rospy.Subscriber('/move_base/status', GoalStatusArray, move_status_callback)

    # Publishers
    global wall_pub, path_pub, frontier_pub
    wall_pub = rospy.Publisher('/wall_gc', GridCells, queue_size=1)
    path_pub = rospy.Publisher('/path_gc', GridCells, queue_size=1)
    frontier_pub = rospy.Publisher('/frontier_gc', GridCells, queue_size=1)

    # Move Base Action Library
    move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction) # /movebase_simple/goal/
    move_base.wait_for_server(rospy.Duration(5))
    
    rospy.sleep(rospy.Duration(5,0))
    while cur_goal == None and not rospy.is_shutdown():
        pass

    rospy.Timer(rospy.Duration(5), request_map)
    detect_frontiers()
    move_to_next_waypoint()
    rospy.spin()
