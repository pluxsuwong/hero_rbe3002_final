#!/usr/bin/env python

import actionlib, rospy, roslib, time, math, tf, numpy, copy, random as rand
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Header
from nav_msgs.msg import OccupancyGrid, GridCells, Path, Odometry
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from actionlib_msgs.msg import GoalID, GoalStatusArray
from map_msgs.msg import OccupancyGridUpdate
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.srv import GetMap

#---------------------------------------Callback Functions--------------------------------------------

# Finish everything
def finish():
    global at_goal
    print "==================Exploration Complete==================="
    at_goal = True
    rospy.signal_shutdown("Exploration Complete")
    exit()


# Stores received costmap in global 2-D array structure
def cost_map_callback(msg):
    global map_width, map_height, cost_map, cell_res, map_origin

    map_width = msg.info.width
    map_height = msg.info.height
    cell_costs = msg.data
    cell_res = msg.info.resolution
    map_origin = msg.info.origin.position
    print 'TIMER: [', rospy.get_rostime().secs - START_TIME, ']'
    # Create cost_map 2D array
    cost_map = [[cell_costs[y*map_width + x] for x in range(map_width)] for y in range(map_height)]
    if rospy.get_rostime().secs - START_TIME > 720:
        finish()
    #detect_frontiers()
    move_to_next_waypoint()

# Check if turtlebot has 
def move_status_callback(msg):
    global last_active_goal, at_goal, reachable_goal, life_cntr, checkpoint
    for goal in msg.status_list:
        if goal.status < 2:
            last_active_goal = goal
            at_goal = False
    if len(msg.status_list) > 0 and not at_goal:
        for goal in msg.status_list:
            if goal.goal_id.id == last_active_goal.goal_id.id:
                if  goal.status == 2 or goal.status == 3:
                    at_goal = True
                    life_cntr = 15
                    checkpoint = rospy.get_rostime().secs
                    print "Status Received: GOOD STOP [", goal.status, ']'
                    move_to_next_waypoint()
                elif goal.status == 4 or goal.status == 5:  # Goal unreachable or rejected
                    at_goal = True
                    reachable_goal = False
                    print "Status Received: BAD STOP [", goal.status, ']'
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
def get_neighbors(cell, mode):
    neighbors = []
    if mode == 8:
        # 8 neighbors
        for y in range(cell.y - 1, cell.y + 2):
            for x in range(cell.x - 1, cell.x + 2):
                if y < 0 or x < 0 or (y == cell.y and x == cell.x) or y >= map_height or x >= map_width:
                    continue
                else:
                    neighbors.append(Point(x, y, 0))
    elif mode == 4:
        # 4 neighbors
        x, y = cell.x + 1, cell.y
        if not (y < 0 or x < 0 or y >= map_height or x >= map_width):
            neighbors.append(Point(x, y, 0))
        x, y = cell.x - 1, cell.y
        if not (y < 0 or x < 0 or y >= map_height or x >= map_width):
            neighbors.append(Point(x, y, 0))
        x, y = cell.x, cell.y + 1
        if not (y < 0 or x < 0 or y >= map_height or x >= map_width):
            neighbors.append(Point(x, y, 0))
        x, y = cell.x, cell.y - 1
        if not (y < 0 or x < 0 or y >= map_height or x >= map_width):
            neighbors.append(Point(x, y, 0))
    return neighbors

# Check if cell has cost of -1 and atleast 1 known cell
def is_frontier_cell(cell):
    try:
        if cost_map[cell.y][cell.x] < 0:
            cell_neighbors = get_neighbors(cell, 4)
            for n in cell_neighbors:
                if 0 <= cost_map[n.y][n.x] <= COST_THRESHOLD:
                    return True
    except IndexError:
        print "Stupid Index Value"
    return False

# Merge frontier fragments
def merge_frontiers(frontier_cells):
    frontier_fragments = []
    # Deep copies are used because of my insecurity with pythonic references
    f_c_buf = copy.deepcopy(frontier_cells)
    while len(f_c_buf) > 0 and not rospy.is_shutdown():
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

# Check if two cells are neighbors
def is_neighbor(i, j):
    result = (abs(i.x - j.x) <= 1) and (abs(i.y - j.y) <= 1)
    return result

# Calculate centroids of frontier fragments
def calc_centroids(frontier_fragments):
    centroids = []
    # Useful for debugging centroids
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
        # If calculated centroid is unknown or occupied find the nearest known empty cell
        if cost_map[y_c][x_c] < 0 or cost_map[y_c][x_c] > COST_THRESHOLD:
            centroids.append(nearest_empty_cell(centroid))
        else:
            centroids.append(centroid)

    return centroids

# Locate nearest empty cell to the provided cell
def nearest_empty_cell(cell):
    explored = [cell]
    unexplored = get_neighbors(cell, 8)

    while True and not rospy.is_shutdown():
        cell_buf = unexplored.pop(0)
        if cell_buf in explored:
            continue
        explored.append(cell_buf)
        if 0 <= cost_map[cell_buf.y][cell_buf.x] <= COST_THRESHOLD:
            return cell_buf
        else:
            unexplored.extend(get_neighbors(cell_buf, 8))

# Return current distance to the provided cell
def cur_dist_to_cell(cell):
    cur_pos = get_cur_pos()
    return math.sqrt(float((cell.x - cur_pos.x)**2 + (cell.y - cur_pos.y)**2))

# Request a costmap update
def request_map(event):
    get_map_srv = rospy.ServiceProxy('/dynamic_map', GetMap)
    cost_map_callback(get_map_srv().map)

# Sort a cell list by x coordinates
def sort_by_x(cell_list):
    return sorted(cell_list, key=lambda x: x.x, reverse=True)

# Sort a frontier fragment list by "priority" (length/distance) <Currently high val == high priority>
def sort_by_priority(frag_list):
    return sorted(frag_list, key=lambda x: x[0], reverse=True)

# Get turtlebot's current position
def get_cur_pos():
    cur_pos = Point()
    cur_pos.x = pose.pose.position.x
    cur_pos.y = pose.pose.position.y
    cur_coords = world_to_grid(cur_pos)
    return cur_coords

# Convert a Point() to its index in the costmap data array
def pos_to_index(pos):
    return pos.y*map_width + pos.x

# Convert a costmap data index to a Point()
def index_to_pos(index):
    return Point(index%map_width, int(index/map_width), 0)

# BFS to find frontiers
def frontier_bfs():

    frontier_cells, visited, queue = set([]), set([]), []
    cur_pos = get_cur_pos()
    # All Point() data is converted to index form to allow for comparison of data
    start = pos_to_index(cur_pos)
    # Initialize queue with elements we want (elements around the base of the turtlebot)
    start_n_pt = []
    y_min = cur_pos.y - 2
    y_max = cur_pos.y + 2
    x_min = cur_pos.x - 2
    x_max = cur_pos.x + 2
    for y in range(cur_pos.y - 2, cur_pos.y + 3):
        for x in range(cur_pos.x - 2, cur_pos.x + 3):
            if y == y_min or y == y_max or x == x_min or x == x_max:
                start_n_pt.append(Point(x, y, 0))
            else:
                visited.add(pos_to_index(Point(x, y, 0)))
    start_n = map(pos_to_index, start_n_pt)

    queue.append(start)
    queue.extend(start_n)
    # While there's stuff in the queue
    while queue and not rospy.is_shutdown():
        # Analyze next cell in queue
        cur_cell = queue.pop(0)
        # If cell has not been scrutinized yet
        if cur_cell not in visited:
            # Note that cell has been scrutinized
            visited.add(cur_cell)
            cur_cell_pt = index_to_pos(cur_cell)
            # If the cell is a frontier cell
            if is_frontier_cell(cur_cell_pt):
                # Add it to the list
                frontier_cells.add(cur_cell)
            else:
                # Get neighbors of cell
                cur_cell_n_pts = get_neighbors(cur_cell_pt, 4)
                cur_cell_n = []
                for n in cur_cell_n_pts:
                    n_pt = pos_to_index(n)
                    # Filter out neighbors that are occupied, queued, or have been visited
                    try:
                        if cost_map[n.y][n.x] <= COST_THRESHOLD and n_pt not in queue and n_pt not in visited:
                            cur_cell_n.append(n_pt)
                    except IndexError:
                        print 'Silly index value generated'
                if cur_cell_n:
                    queue.extend(cur_cell_n)

    frontier_cell_pts = map(index_to_pos, list(frontier_cells))
    print "Frontier Cells: ", len(frontier_cell_pts)
    return list(frontier_cell_pts)

# Filter out frontier fragments that are too small for the turtlebot (or not noteworthy)
def filter_fragments(frag_list):
    good_frags = []

    for fragment in frag_list:
        max_dist = 0
        #min_x, max_x, min_y, max_y = 0, 0, 0, 0
        for cell_1 in fragment:
            for cell_2 in fragment:
                dx2 = (cell_2.x - cell_1.x)**2
                dy2 = (cell_2.y - cell_1.y)**2
                a_dist = math.sqrt(dx2 + dy2)
                if a_dist > max_dist:
                    max_dist = a_dist
            '''
            if cell_1.x < min_x:
                min_x = cell_1.x
            if cell_1.x > max_x:
                max_x = cell_1.x
            if cell_1.y < min_y:
                min_y = cell_1.y
            if cell_1.y > max_y:
                max_y = cell_1.y
            '''
        #max_dist = math.sqrt((max_x - min_x)**2 + (max_y - min_y)**2)
        if max_dist > ROBOT_WIDTH:
            good_frags.append(fragment)

    return good_frags

#-----------------------------------------Update Grid Functions---------------------------------------

# Update RViz, and return an array of frontier cells
def update_frontier():
    global frontier_pub

    # Naive iteration over cost map
    '''
    frontier_cells = []
    for y in range(map_height):
        for x in range(map_width):
            cell = Point(x, y, 0)
            if is_frontier_cell(cell):
                frontier_cells.append(cell)
    '''
    # Better BFS method
    frontier_cells = frontier_bfs()
    frontier_msg = GridCells()
    frontier_msg.header.frame_id = 'map'
    frontier_msg.cell_width = cell_res
    frontier_msg.cell_height = cell_res
    displayable_cells = map(grid_to_world, frontier_cells)
    frontier_msg.cells = displayable_cells
    frontier_pub.publish(frontier_msg)

    return frontier_cells

#----------------------------------------Navigation Functions-----------------------------------------

# Move to Next Waypoint
def move_to_next_waypoint():
    global move_base
    detect_frontiers()
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = rospy.Time.now()
    goal_pose.pose.position = grid_to_world(cur_goal)
    # Modify orientation to face movement heading
    dx = goal_pose.pose.position.x - pose.pose.position.x
    dy = goal_pose.pose.position.y - pose.pose.position.y
    new_ori = math.atan2(dy, dx)
    o_x, o_y, o_z, o_w = quaternion_from_euler(0.0, 0.0, new_ori)
    goal_pose.pose.orientation.x = o_x
    goal_pose.pose.orientation.y = o_y
    goal_pose.pose.orientation.z = o_z
    goal_pose.pose.orientation.w = o_w

    # Navigate to goal_pose
    move_base.publish(goal_pose)

# Detect Frontier cells
def detect_frontiers():
    global cur_goal, reachable_goal, life_cntr, checkpoint
    
    # Obtain frontier cells
    frontier_cells = update_frontier()
    
    # Sort frontier_cells by x coord (optimize performance of merge_frontiers())
    sorted_f_cells = sort_by_x(frontier_cells)

    # Group the frontier cells into continuous frontiers and return them as a list
    frontier_fragments = merge_frontiers(sorted_f_cells)

    # Keep frontiers wide enough for the robot
    good_f_fragments = filter_fragments(frontier_fragments)

    # If there are no good_f_fragments, mapping is done
    if len(good_f_fragments) == 0 or life_cntr <= 0:
        finish()
    # Calculate the centroids of all of the frontiers
    centroids = calc_centroids(good_f_fragments)

    # Calculate the number of frontier cells in each frontier
    frag_len = map(len, good_f_fragments)

    # Calculate the distance to each centroid
    distances = map(cur_dist_to_cell, centroids)

    # Calculate priority of fragment based on length / distance
    centroid_priority = []
    for i in range(len(distances)):
        try:
            centroid_priority.append((frag_len[i]**2 / distances[i]))
        except ZeroDivisionError:
            print 'Division by 0 in detect_frontiers()'
    
    frag_tup_list = [[priority, fragment, centroid] for priority, fragment, centroid in zip(centroid_priority, good_f_fragments, centroids)]
    sorted_f_t_l = sort_by_priority(frag_tup_list)
    
    if not reachable_goal:
        # Obtain random goal since current goal is unreachable
        i = rand.randint(0, len(sorted_f_t_l) - 1)
        cur_pos = get_cur_pos()
        cur_goal = Point()
        cur_goal.x = int(cur_pos.x + (sorted_f_t_l[i][2].x - cur_pos.x)*NAV_GAIN)
        cur_goal.y = int(cur_pos.y + (sorted_f_t_l[i][2].y - cur_pos.y)*NAV_GAIN)
        if len(sorted_f_t_l) <= 3:
            i = rand.randint(-7, 7)
            j = rand.randint(-7, 7)
            cur_goal.x += i
            cur_goal.y += j
        life_cntr -= 1
        print 'Unreachable Goal, Replacing With: [', cur_goal.x, ',', cur_goal.y, '] [', life_cntr, ']'
    else:
        # Navigate to nearest fragment that really needs exploring
        cur_pos = get_cur_pos()
        cur_goal = Point()
        cur_goal.x = int(cur_pos.x + (sorted_f_t_l[0][2].x - cur_pos.x)*0.9)
        cur_goal.y = int(cur_pos.y + (sorted_f_t_l[0][2].y - cur_pos.y)*0.9)
        # Use entropy to prevent robot from getting stuck
        i = rand.randint(-1, 1)*(int(math.sqrt(rospy.get_rostime().secs - checkpoint)*NAV_GAIN))
        j = rand.randint(-1, 1)*(int(math.sqrt(rospy.get_rostime().secs - checkpoint)*NAV_GAIN))
        if i < -10:
            i = rand.randint(-10, 0)
        elif i > 10:
            i = rand.randint(0, 10)
        if j < -10:
            j = rand.randint(-10, 0)
        elif j > 10:
            j = rand.randint(0, 10)
        cur_goal.x += i
        cur_goal.y += j
        life_cntr = 15
        print 'Navigating to New Goal: [', cur_goal.x, ',', cur_goal.y, ']'

    print 'Remaining Frontier Fragments: [', len(good_f_fragments), ']'


# -------------------------------------------Main Function---------------------------------------------

if __name__ == '__main__':

    # Create Node
    rospy.init_node('hero_rbe3002_final')

    # Modifiable global variables
    global COST_THRESHOLD, NAV_GAIN, ROBOT_WIDTH, START_TIME
    # data values in the costmap above the cost threshold indicate occupancy by an object
    COST_THRESHOLD = 9
    # 0.7 is an arbitrary gain chosen to prevent concave frontiers from resulting in unreachable goals
    NAV_GAIN = 0.7
    ROBOT_WIDTH = 8
    START_TIME = rospy.get_rostime().secs

    # Global Variables for Navigation
    global at_goal, cur_goal, reachable_goal, life_cntr, checkpoint
    # odom_list = tf.TransformListener()
    at_goal = True
    cur_goal = None
    reachable_goal = True
    life_cntr = 30
    checkpoint = rospy.get_rostime().secs

    # Subscribers
    rospy.Subscriber('/odom', Odometry, read_odom)
    rospy.Subscriber('/move_base/status', GoalStatusArray, move_status_callback)

    # Publishers
    global frontier_pub, move_base
    frontier_pub = rospy.Publisher('/frontier_gc', GridCells, queue_size=1)
    move_base = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=5)

    rospy.sleep(rospy.Duration(5))
    # Initialize costmap
    request_map(None)
    # Give program time to run
    rospy.sleep(rospy.Duration(5))
    # Move to first waypoint
    move_to_next_waypoint()
    # Request costmap update every 5 seconds
    rospy.Timer(rospy.Duration(5), request_map)
    # Keep node running
    rospy.spin()
