import time

from Env import Env
from Draw import Draw
import matplotlib.pyplot as plt
from Node import Node
from Tree import Tree,Edge
from Utils import *
import numpy as np
from tqdm import *
from plotting import *
class RT_RRT_Star:
    def __init__(self,pos_start,pos_goal,start_time,step_len,search_radius,goal_sample_rate,k_max,rs,max_iter,fig,ax,env,edges=[],vertex=[]):
        self.pos_start = Node(pos_start)
        self.pos_goal = Node(pos_goal)
        self.start_time = start_time
        self.step_len = step_len
        self.search_radius = search_radius
        self.goal_sample_rate = goal_sample_rate
        self.k_max = k_max
        self.rs = rs
        self.iter_max = max_iter
        self.fig = fig
        self.ax = ax

        self.obs_add = [0, 0, 0]
        self.edges = edges
        self.env = env
        self.utils = Utils(env)
        self.plotting = Plotting(pos_start,pos_goal)
        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary
        self.vertex = vertex
        self.qr = []
        self.qs = []
        self.waypoint = []
    def planning(self):
        i = 0
        if not self.vertex:
            self.vertex = [self.pos_start]
        start_ind = self.get_node_index_xy(self.pos_start.x,self.pos_start.y)
        if self.vertex[start_ind].parent:
            self.change_root(start_ind)
        while time.time() - self.start_time < 0.5:
            node_rand = self.generate_random_node(self.goal_sample_rate)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)

            if node_new and not self.utils.is_collision(node_near,node_new):
                neighbor_index = self.find_near_neighbor(node_new)
                self.vertex.append(node_new)
                self.edges.append(Edge(node_near,node_new))
                if neighbor_index and (len(neighbor_index) < self.k_max or self.utils.get_dist(node_new,node_near) < self.rs):
                    self.addNodeToTree(node_new,node_near,neighbor_index)
                    self.qr.insert(0,node_new)
                else:
                    self.qr.insert(0,node_near)
                self.rewireRandomNode()
            self.rewireRootNode()
            index = self.search_goal_parent()
            self.waypoint = self.extract_waypoint(self.vertex[index])
            self.generate_graph()
            self.fig.canvas.mpl_connect('button_press_event', self.on_press)
        index = self.search_goal_parent()
        self.waypoint = self.extract_path(self.vertex[index])
        next_ind = self.get_next_root_ind(self.waypoint)
        return (self.pos_goal.x,self.pos_goal.y),self.vertex,self.edges,(self.vertex[next_ind].x,self.vertex[next_ind].y)
    def change_root(self, next_root_ind):
        current_root_ind = self.get_node_index_xy(self.vertex[next_root_ind].parent.x, self.vertex[next_root_ind].parent.y)
        self.vertex[next_root_ind].parent =  None
        self.vertex[current_root_ind].parent = self.vertex[next_root_ind]
    def get_next_root_ind(self,path):
        next_ind = 0
        if len(path) == 2:
            next_root_x = path[1][0]
            next_root_y = path[1][1]
            next_ind = self.get_node_index_xy(next_root_x, next_root_y)
        elif len(path) > 2:
            next_root_x = path[-2][0]
            next_root_y = path[-2][1]
            next_ind = self.get_node_index_xy(next_root_x, next_root_y)
        else:
            next_root_x = path[0][0]
            next_root_y = path[0][1]
            next_ind = self.get_node_index_xy(next_root_x, next_root_y)
        return next_ind
    def get_node_index_xy(self, x, y):
        for i in range(0,len(self.vertex)):
            node = self.vertex[i]
            if node.x == x and node.y == y:
                return i
        return None
    def generate_graph(self):
        self.ax.clear()
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        for node in self.vertex:
            if node.parent:
                plt.plot(node.path_horizontal(), node.path_vertical(), "-b")
                # plt.plot(node.path_horizontal(), node.path_vertical(), "-w")
        for node in self.waypoint:
            if node.parent:
                plt.plot(node.path_horizontal(), node.path_vertical(), "-r")
        for (ox, oy, w, h) in self.env.obs_boundary:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )

        for (ox, oy, w, h) in self.env.obs_rectangle:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        for (ox, oy, r) in self.env.obs_circle:
            self.ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='gray',
                    fill=True
                )
            )

        plt.plot(self.pos_start.x, self.pos_start.y, "gs", linewidth=3)
        plt.plot(self.pos_goal.x, self.pos_goal.y, "rs", linewidth=3)


        plt.title(label="RT-RRT")


        plt.axis("equal")
        plt.pause(0.001)
    def extract_waypoint(self,node_end):
        path = []
        node = node_end

        while node.parent is not None:
            path.append(node)
            node = node.parent
        path.append(node)

        return path
    def extract_path(self, node_end):
        path = []
        node = node_end

        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([node.x, node.y])

        return path
    def search_goal_parent(self):
        dist_list = [math.hypot(n.x - self.pos_goal.x, n.y - self.pos_goal.y) for n in self.vertex]
        node_index = [i for i in range(len(dist_list)) if dist_list[i] <= self.step_len]

        if len(node_index) > 0:
            cost_list = [dist_list[i] + self.cost(self.vertex[i]) for i in node_index
                         if not self.utils.is_collision(self.vertex[i], self.pos_goal)]
            return node_index[int(np.argmin(cost_list))]

        return int(np.argmin(dist_list))

    @staticmethod
    def cost(node_p):
        node = node_p
        cost = 0.0

        while node.parent:
            cost += math.hypot(node.x - node.parent.x, node.y - node.parent.y)
            node = node.parent
        return cost
    def get_new_cost(self, node_start, node_end):
        dist, _ = self.get_distance_and_angle(node_start, node_end)

        return self.cost(node_start) + dist
    def addNodeToTree(self,node_rand,node_near,neighbor_index):
        cost = [self.get_new_cost(self.vertex[i], node_rand) for i in neighbor_index]

        cost_min_index = neighbor_index[int(np.argmin(cost))]
        node_rand.parent = self.vertex[cost_min_index]
        # self.vertex[cost_min_index].child.append(node_rand)
    def generate_random_node(self, goal_sample_rate):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:
            return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
                         np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.pos_goal

    def find_near_neighbor(self, node_new):
        n = len(self.vertex) + 1
        r = min(self.search_radius * math.sqrt((math.log(n) / n)), self.step_len)
        dist_table = [math.hypot(nd.x - node_new.x, nd.y - node_new.y) for nd in self.vertex]
        dist_table_index = [ind for ind in range(len(dist_table)) if dist_table[ind] <= r and
                            not self.utils.is_collision(node_new, self.vertex[ind])]

        return dist_table_index
    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]
    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)
    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start
        # node_start.child.append(node_new)
        return node_new

    def rewireRandomNode(self):
        while len(self.qr) != 0:
            x_r = self.qr.pop(0)
            neighbor_index = self.find_near_neighbor(x_r)
            for i in neighbor_index:
                node_neighbor = self.vertex[i]
                if self.cost(node_neighbor) > self.get_new_cost(x_r, node_neighbor):
                    node_neighbor.parent = x_r
                    # x_r.child.append(node_neighbor)
                    self.qr.append(node_neighbor)
    def rewireRootNode(self):
        if len(self.qs) == 0:
            self.qs.append(self.pos_start)
        qs_popped = []

        while len(self.qs) != 0:
            x_s = self.qs.pop(0)
            qs_popped.append(x_s)
            neighbor_index = self.find_near_neighbor(x_s)
            for i in neighbor_index:
                node_neighbor = self.vertex[i]
                if self.cost(node_neighbor) > self.get_new_cost(x_s, node_neighbor):
                    node_neighbor.parent = x_s
                    # x_s.child.append(node_neighbor)
                if node_neighbor not in qs_popped:
                    self.qs.append(node_neighbor)

    def is_path_invalid(self):
        for node in self.waypoint:
            if node.flag == "INVALID":
                return True

    def TrimRRT(self):

        for i in range(0, len(self.vertex)):
            path_index = []
            node = self.vertex[i]
            while node is not None:
                if node.flag == "INVALID" and len(path_index)!= 0:
                    for j in path_index:
                        self.vertex[j].flag = "INVALID"
                    break
                node = node.parent
                path_index.append(i)

        self.vertex = [node for node in self.vertex if node.flag == "VALID"]
        self.edges = [Edge(node.parent, node) for node in self.vertex if node.parent]
    def InvalidateNodes(self):
        for edge in self.edges:
            if self.is_collision_obs_add(edge.parent, edge.child):
                edge.child.flag = "INVALID"

    def is_collision_obs_add(self, start, end):
        delta = self.utils.delta
        obs_add = self.obs_add
        if math.hypot(start.x - obs_add[0], start.y - obs_add[1]) <= obs_add[2] + delta:
            return True

        if math.hypot(end.x - obs_add[0], end.y - obs_add[1]) <= obs_add[2] + delta:
            return True

        o, d = self.utils.get_ray(start, end)
        if self.utils.is_intersect_circle(o, d, [obs_add[0], obs_add[1]], obs_add[2]):
            return True

        return False
    def on_press(self,event):
        x, y = event.xdata, event.ydata
        if x < 0 or x > 60 or y < 0 or y > 30:
            print("Please choose right area!")
        elif event.button == 1:
            x = event.xdata
            y = event.ydata
            self.pos_goal = Node((x,y))
        elif event.button == 3:
            x = event.xdata
            y = event.ydata
            print("Add circle obstacle at: s =", x, ",", "y =", y)
            self.obs_add = [x,y,2]
            self.env.obs_circle.append(self.obs_add)
            self.InvalidateNodes()
            self.TrimRRT()
            # if self.is_path_invalid():
            #     print("Path is Replanning ...")
            # else:








def main():
    x_start = (2,2)
    x_goal = (49,24)
    fig, ax = plt.subplots()
    env = Env()
    start_time = time.time()
    rt_rrt = RT_RRT_Star(x_start,x_goal,start_time,3,3,0.1,10,10,5000,fig,ax,env)
    goal_node,vertex,edges,next_node = rt_rrt.planning()
    while time.time() - start_time < 60:
        epoch_time = time.time()
        rt_rrt = RT_RRT_Star(next_node, goal_node, epoch_time, 2, 3, 0.1, 10, 10, 5000,fig,ax,env,edges,vertex)
        goal_node, vertex, edges, next_node = rt_rrt.planning()

if __name__ == '__main__':
    main()