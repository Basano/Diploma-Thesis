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




