## Motion Planning with PRM
## By Andy Xu

### Introduction
In this assignment, motions will be planned for 2R, 3R, and 4R planar arms. The base of the arm is at location (0, 0), and the joint angles are measured counter-clockwise. There are obstacles in the workspace that consist of polygons that the arm cannot intersect during its path from the start state to the goal state. The motions of the arm are decided using a Probabilistic Roadmap Planner (PRM). 

### Instructions 
`PRMRobotArm.py` contains all the code necessary for the PRM robot arm implementation. Tests can be found near the bottom under the `if __name__ == "__main__":` section. There are tests for 2R, 3R, and 4R planar arms as labeled by the comments. Please uncomment whichever section you'd like to test and comment out other sections before running the code. The output should include a list of angle positions for the arm printed to the terminal, which represents its path, and a series of scatterplots displaying the robot arm and obstacles as the robot arm moves from start state to goal state.
 
### Kinematics
To compute the locations of end points of the links for a configuration given by angles θ1…θn, I wrote the `get_arm_locations` function which takes in the list of angles as a parameter. It then uses a for loop that iterates n times, where n is the number of angles. Each time, it calculates the vertex position of the next vertex using the equation discussed in class (cos for the x value and sin for the y value). It appends each vertex to the list of vertices and returns the full list.

### Obstacles and Collision Detection
To create obstacles, I utilized Shapely's Polygon class. I created three polygon obstacles by listing their coordinates and passed them as a list when instantiating the instance of PRMRobotArm. 

To check for collisions, I first checked if during the process of moving from one state to another, any vertex crossed an obstacle. I did this by first getting the vertices of both states using their angles and calls to the `get_arm_locations` function. Then, I iterated through the number of vertices, constructing a line using Shapely's Linestring class between each vertex's original position and its new one. I used Shapely's `intersects` function to check if each line intersected with any obstacle. If it did, I returned True. If not, I moved to the next step.

The next step consisted of checking if the new position of the arm intersected with any obstacle. To do this, I simply constructed a line between each of the vertices of the arm and checked if each line intersected with any obstacle. If any of the lines intersected with obstacles, I return True. If not, I return False.

### Probabilistic Roadmap (PRM)
I implement the k-PRM version of Probabilistic Roadmap planner, in which the local planner only attempts to connect a vertex to its nearest k neighbors. For testing, I set k to 10. 

The implementation of the first phase of PRM, roadmap generation, can be found in the function `generate_roadmap`. It recieves k as a parameter. The first thing it does is generate a specified number of possible robot arm configurations that don't collide with any obstacles. For testing purposes, I generated 10000 random configurations. Each configuration consisted of a number of angles depending on the number of joints the arm had. I also made sure to add the start and goal state configurations. Then, I utilized SciPy's cdist library to calculate the euclidean distances between every configuration. Finally, I created a map which mapped each configuration to the 10 closest ones based on the distances I had calculated. I return the resulting map.

The implementation of the second phase of PRM, the query phase, can be found in the function `dijkstras`. It recieves the map as the parameter, gets the start and goal states, and utilizes Dijkstra's algorithm to get the shortest path from the start to goal states. It uses backtracking to get the final solution path, which it returns.

I have included examples of what the robot arm movement would look like for 2R, 3R, and 4R planar arms below. These are displayed using matplotlib in the `display` function. For the purpose of this report, I did not include every individual movement, but only included a few I thought were worth showing for each one to demonstrate the idea. (If the images do not show up in the markdown file, please refer to the labeled images included in the submission folder).

2R Planar Arm Movement:  
<img width="636" alt="2r_img1" src="https://github.com/andyxuca/COSC-76-AI/assets/35200467/01af79e1-8b75-48a7-851d-93a55117987b">
<img width="636" alt="2r_img2" src="https://github.com/andyxuca/COSC-76-AI/assets/35200467/8df8070b-fcad-41bc-920d-9113d18c1e0d">
<img width="633" alt="2r_img3" src="https://github.com/andyxuca/COSC-76-AI/assets/35200467/431707cc-21d7-47ad-b8f4-0b4a9318f222">

3R Planar Arm Movement:  
<img width="638" alt="3r_img1" src="https://github.com/andyxuca/COSC-76-AI/assets/35200467/d7dfb982-c655-4315-ac25-837e675a0ef2">
<img width="636" alt="3r_img2" src="https://github.com/andyxuca/COSC-76-AI/assets/35200467/c131203f-6ee4-4a91-b1f7-66f7e2ce3cb3">
<img width="636" alt="3r_img3" src="https://github.com/andyxuca/COSC-76-AI/assets/35200467/64f45d0d-18bf-4036-9f74-48b09d3581a7">
<img width="636" alt="3r_img4" src="https://github.com/andyxuca/COSC-76-AI/assets/35200467/46689b44-905a-4025-9412-e42b6edc6f34">

4R Planar Arm Movement:  
<img width="637" alt="4r_img1" src="https://github.com/andyxuca/COSC-76-AI/assets/35200467/c7da317c-7737-4dbc-abac-c7676afe772c">
<img width="634" alt="4r_img2" src="https://github.com/andyxuca/COSC-76-AI/assets/35200467/c97a0bb8-ed6b-4c4a-92dd-b8d2d4ca3d9d">
<img width="633" alt="4r_img3" src="https://github.com/andyxuca/COSC-76-AI/assets/35200467/51dee5e5-ec5a-45b4-ba01-4d7d9559be1c">
<img width="637" alt="4r_img4" src="https://github.com/andyxuca/COSC-76-AI/assets/35200467/7bd9f1f5-6c08-4254-a9ab-d68a6116c677">

### Extension 1: Literature Review 1 (Previous Work)
Paper Citation:  
L. E. Kavraki, M. N. Kolountzakis and J. . -C. Latombe, "Analysis of probabilistic roadmaps for path planning," in IEEE Transactions on Robotics and Automation, vol. 14, no. 1, pp. 166-171, Feb. 1998, doi: 10.1109/70.660866.

Summary:  
In their 1998 paper published in IEEE, L. E. Kavraki, M. N. Kolountzakis, and J. -C. Latombe analyze probabilistic roadmaps for path planning, providing insights into their effectiveness and performance. The study investigated the impact of path length, distance function from obstacles, and the number of nodes in the probabilistic roadmap on the failure probability of connecting two robot configurations. The analysis revealed that the number of nodes in the roadmap has a big impact, increasing exponentially, while the length of the path has a linear effect. The study also highlighted the method's usefulness in spaces with tight spots and lots of open areas, emphasizing differences from other planning approaches. The provided formulas helped answer questions related to figuring out the number of nodes needed for a high success probability when the path stays a certain distance from obstacles. The research further elaborates that understanding how a robot's environment influences planning methods is essential for future improvements.

### Extension 2: Literature Review 2 (Previous Work)
Paper Citation:  
Kneebone, M., & Dearden, R. (2009). Navigation Planning in Probabilistic Roadmaps with Uncertainty. Proceedings of the International Conference on Automated Planning and Scheduling, 19(1), 209-216. https://doi.org/10.1609/icaps.v19i1.13359

Summary:  
This paper discussed the application of Probabilistic Roadmaps (PRM) for robot navigation in environments with imprecisely known obstacle positions. It addressed the challenge of traversing the PRM graph optimally by considering noisy observations of uncertain edges, presenting a solution using a partially observable Markov decision process (POMDP) representation. The proposed approach, leveraging belief space structure, achieves near-optimal results in most cases with a significant speed-up in policy generation time compared to exact methods. However, the POMDP isn't always viable, and another method called PERSEUS is faster but struggles with certain challenges like low probability of reaching some states or extra structure in some problems. The researcheres described a goal of testing this method on bigger puzzles with more uncertainties to see how well it is able to scale.











