# Diploma-Thesis
Code for experimental part of my diploma thesis. 

The main body of the code was taken from open source library https://github.com/AtsushiSakai/PythonRobotics#what-is-this and some edits were made to fit my thesis

Brief explanation of algorithms:

RRT*:

  * The algorithm initiates with start point, goal point, and list of obstacles with centers of circles. 
After a planning function is called, a random node from pre-defined range is picked.
  * Function <b>get_nearest_node_index</b> then finds index of the node which is the nearest to random node.
  * Function <b>steer</b> creates a new node object which is <i>expand_dis</i> far away from the nearest node and contains information about parent and path based on <i>resolution</i> of path. Subsequently, cost of this node object is calculated.
  * Function <b>check_collision</b> then checks path colisions. The colisions are checked for components of attribute path in every node. The number of path components in depends on the path resolution.
  * Function <b>find_near_nodes</b> then finds nodes in the proximity <i>r</i>. This <i>r</i> is defined by a formula found in Sampling-based algorithms for optimal motion planning by karaman 2011. The nodes are determined using equation for circle. 
  * Function <b>choose_parent</b> chooses parent for the new node from near nodes found in previous function. Inside, function <b>steer</b> is used without expand_distance argument to get the node and cost from near nodes to new node. Then, the one with minimum cost is determined and finally <b>steer</b> is used once more to yield a node object which is a new node with updated parent.
  * If <b>choose_parent</b> yields None, then new_node from very first <b>steer</b> call is added to the list. 
  * If however it yields some node, then function <b>rewire</b> is run. For every near node, <b>steer</b> is called to compute path and cost from new node, and if its collision free and cost is lower, then near node is adjusted and finally function <b>propagate_cost_to_leaves</b> is called to update parents and costs of nodes in new path.
  * This concludes the searching part. After all the iterations, function <b>search_best_goal_node</b> is called which computes distance to all the nodes from the goal, chooses the minimum and collision free one and passes it as an argument for <b>generate_final_course</b> which iterates over all parents until it comes to the start.


A*:
  * Algorithm runs using initializing data like start and goal nodes or boundary. In the initialization process, function <b>get_motion_model</b> embodies all possible movements of the robot and <b>calc_obstacle_map</b> creates a binary map with given resolution by checking all the nodes in the grid to list of obstacles and boundary points. Since this works in the grid, which is usually different from the workspace, function <b>calc_grid_position</b> calculates position of grid node in the workspace and then checks colisions.
  * Function <b>planning</b> runs the whole process. First goal node and start node are defined. Function <b>calc_xy_index</b> calculates their grid position from their position in the workspace without grid. Two dictionaries are created : open_list and closed_list. The key in these dictionaries is created by <b>calc_grid_index</b>, which assigns every node a unique index.
  * While loop keeps iterating the subsequent steps until either goal is found or open_list is empty.
  * Current node is chosen based on the lowest cost+heuristic value. Search of neighboors is done based on the <b>motion_model</b>. function <b>verify_node</b> then checks whether the node collides with obstacles or boundary. Then, if node is already in the closed set, it is ignored, if in the open set, cost is improved if possible, and if its not in either, it is added to the open_set.
  * The search continues until termination condition is fulfilled. In the end, function <b>calc_final_path</b> returns the path from start to goal by iterating over neighboors of nodes.



