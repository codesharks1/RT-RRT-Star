import matplotlib.pyplot as plt
import math
import numpy as np
from Node import Node
class Draw:
    def __init__(self,environment):
        self.environment = environment
    def generate_graph(self):
        plt.clf()
        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        plt.gcf().canvas.mpl_connect('button_press_event', self.interaction_event)
        plt.plot(self.environment.pos_start.x, self.environment.pos_start.y, "Dg" , markersize=10)
        plt.plot(self.environment.pos_goal.x, self.environment.pos_goal.y, "or" , markersize = 10)
        for(ox, oy, size) in self.environment.circular_obstacles:
            self.generate_circle(ox, oy, size)
        plt.title(label="RT-RRT-Star")
        plt.axis([self.environment.min_randx, self.environment.max_randx, self.environment.min_randy, self.environment.max_randy])
        plt.xticks(color='w')
        plt.yticks(color='w')
        plt.grid(False)
        plt.pause(0.001)
    @staticmethod
    def generate_circle(x, y, size):
        # 创建实心圆对象
        circle = plt.Circle((x, y), size, color='k', fill=True)
        # 将圆对象添加到当前图形中
        plt.gca().add_patch(circle)
    # 左键是改变目标的位置  右键是生成障碍物
    def interaction_event(self,event):
        if event.button == 3:  # 仅处理鼠标右键点击
            x = event.xdata
            y = event.ydata
            self.environment.add_obstacle(x,y)
        if event.button == 1:
            x = event.xdata
            y = event.ydata
            self.environment.pos_goal = Node(x, y, None)