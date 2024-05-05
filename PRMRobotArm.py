# Author: Andy Xu
# Date: 11/10/23

import math
from shapely import Polygon, LineString
import random
import numpy as np
from scipy.spatial.distance import cdist
from collections import deque
import matplotlib.pyplot as plt

class PRMRobotArm:

    def __init__(self, num_angles, arm_lengths, initial_angles, goal_angles, obstacles):
        self.num_angles = num_angles
        self.arm_lengths = arm_lengths
        self.obstacles = obstacles
        self.initial_angles = initial_angles
        self.goal_angles = goal_angles
        self.max_configs = 10000
        self.configs = []

    # function that computes the locations end points of the links for a
    # configuration given by angles θ1…θn using kinematics
    def get_arm_locations(self, angles):
        arm_locations = []
        x1 = 0
        y1 = 0
        a1 = 0
        for i in range(self.num_angles):
            x2 = x1 + self.arm_lengths[i] * math.cos(a1 + angles[i])
            y2 = y1 + self.arm_lengths[i] * math.sin(a1 + angles[i])
            arm_locations.append((x2, y2))
            a1 += angles[i] % (2 * math.pi)
            x1 = x2
            y1 = y2

        return arm_locations

    #function that checks if moving from state to another collides with any obstacles
    def collision(self, cur_angles, new_angles):
        cur_edpts = self.get_arm_locations(cur_angles)

        # check if moving arm crosses obstacle
        if new_angles:
            new_edpts = self.get_arm_locations(new_angles)
            for i in range(len(new_edpts)):
                arm = LineString([cur_edpts[i], new_edpts[i]])

                for obs in self.obstacles:
                    if arm.intersects(obs):
                        return True

        # check if new position of arm intersects obstacle
        else:
            point = (0, 0)
            for l in cur_edpts:
                arm = LineString([point, l])
                point = l

                for obs in self.obstacles:
                    if arm.intersects(obs):
                        return True

        return False

    #function that generates the roadmap for the first phase of PRM
    def generate_roadmap(self, k_neighbors):
        # generate random valid configurations
        random.seed(1)
        self.configs = []

        #generate max_configs amount of random configurations
        for i in range(self.max_configs):
            angles = []
            for j in range(self.num_angles):
                angles.append(2 * math.pi * random.random())

            #check that generated configuration does not collide with any obstacle
            if not self.collision(angles, None):
                self.configs.append(tuple(angles))

        if self.initial_angles not in self.configs:
            self.configs.append(tuple(initial_angles))
        if self.goal_angles not in self.configs:
            self.configs.append(tuple(goal_angles))

        #get distances between each configuration
        distances = cdist(self.configs, self.configs, metric='euclidean')

        #store the k closest neighbors of each configuration in a map
        map = {}
        for i in range(len(self.configs)):
            neigh = np.argsort(distances[i])[1:k_neighbors + 1]  # get k closest neighbors

            if i not in map:
                map[i] = []

            for n in neigh:
                # if a different point and does not collide
                if not n == i and not self.collision(self.configs[i], self.configs[n]):
                    # create an edge
                    if n not in map[i]:
                        map[i].append(n)

        return map

    #function that implements Dijkstra's algorithm for the query phase of PRM
    def dijkstras(self, map):
        #get start and end states, stored at end of configurations list
        start = len(self.configs) - 2
        end = len(self.configs) - 1

        #create deque and lists for visited nodes and backtracking
        to_visit = deque([(0,start)])
        visited = set()
        previous = {node:None for node in map}
        distances = {node: float('inf') for node in map}

        while to_visit:
            #pop node to visit
            (cost, current) = to_visit.popleft()

            if current in visited:
                continue

            visited.add(current)

            if current == end:
                break

            #add neighbors to to_visit list
            for neighbor in map.get(current):
                new_distance = cost + 1
                to_visit.append((new_distance, neighbor))

                #update distances and previous nodes
                if new_distance < distances[neighbor]:
                    distances[neighbor] = new_distance
                    previous[neighbor] = current

        #backtrack to find shortest path
        path = []
        current = end
        while current and current is not start:
            path.insert(0, self.configs[current])
            current = previous[current]
        path.insert(0, self.configs[start])

        return path

    def display(self, arm_verts, obstacles):

        #display arm
        x = [0]
        y = [0]
        for v in arm_verts:
            x.append(v[0])
            y.append(v[1])

        plt.scatter(x, y, color='green', marker='o')

        for i in range(len(arm_verts)):
            plt.plot([x[i], x[i + 1]], [y[i], y[i + 1]], color='green')

        #display obstacles
        for i in range(len(obstacles)):
            obs_verts = list(obstacles[i].exterior.coords)
            x = []
            y = []

            for v in obs_verts:
                x.append(v[0])
                y.append(v[1])

            plt.scatter(x, y, color='black', marker='o')

            for j in range(len(arm_verts) - 1):
                plt.plot([x[j], x[j + 1]], [y[j], y[j + 1]], color='black')

            plt.plot([x[-1], x[0]], [y[-1], y[0]], color='black')
            plt.fill(x, y, color='black')

        plt.show()

if __name__ == "__main__":
    #run tests

    #define obstacles
    o1 = Polygon([(10, 10), (10, 20), (20, 20), (20, 10)])
    o2 = Polygon([(-20, 0), (-10, 0), (-10, 5), (-20, 5)])
    o3 = Polygon([(-18, 15), (-15, 15), (-15, 18), (-18, 18)])

    #2R robot arm
    # initial_angles = (math.pi / 2, math.pi / 2)
    # goal_angles = (math.pi / 4, math.pi / 3)
    # arm_lengths = [8, 10]
    #
    # test_2r = PRMRobotArm(2, arm_lengths, initial_angles, goal_angles, [o1, o2, o3])
    # map = test_2r.generate_roadmap(10)
    # sol = test_2r.dijkstras(map)
    # print(sol)
    # for i in sol:
    #     test_2r.display(test_2r.get_arm_locations(i), [o1, o2, o3])


    #3R robot arm
    # initial_angles = (math.pi / 2, math.pi / 2, 3 * math.pi/4)
    # goal_angles = (math.pi / 4, math.pi / 3, math.pi/6)
    # arm_lengths = [8, 8, 8]
    #
    # test_3r = PRMRobotArm(3, arm_lengths, initial_angles, goal_angles, [o1, o2, o3])
    # map = test_3r.generate_roadmap(10)
    # sol = test_3r.dijkstras(map)
    # print(sol)
    #
    # for i in sol:
    #     test_3r.display(test_3r.get_arm_locations(i), [o1, o2, o3])

    #4R robot arm
    initial_angles = (3 * math.pi / 4, 2 * math.pi / 3, 3 * math.pi / 4, math.pi / 6)
    goal_angles = (math.pi / 4, math.pi / 6, math.pi / 6, math.pi / 6)
    arm_lengths = [8, 6, 5, 4]

    test_4r = PRMRobotArm(4, arm_lengths, initial_angles, goal_angles, [o1, o2, o3])
    map = test_4r.generate_roadmap(10)
    sol = test_4r.dijkstras(map)
    print(sol)

    for i in sol:
        test_4r.display(test_4r.get_arm_locations(i), [o1, o2, o3])


