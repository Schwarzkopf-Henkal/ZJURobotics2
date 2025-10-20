import socket
import sys
import time

from zss_debug_pb2 import Debug_Msgs, Debug_Msg, Debug_Arc

class Debugger(object):
    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.debug_address = ('localhost', 20001)

    def draw_circle(self, package, x, y, radius=300, color=None):
        msg = package.msgs.add()
        msg.type = Debug_Msg.ARC
        msg.color = color if color is not None else Debug_Msg.WHITE
        arc = msg.arc
        arc.rectangle.point1.x = x - radius
        arc.rectangle.point1.y = y - radius
        arc.rectangle.point2.x = x + radius
        arc.rectangle.point2.y = y + radius
        arc.start = 0
        arc.end = 360
        arc.FILL = True
    
    def draw_rrt_tree(self, package, rrt_planner):
        """绘制RRT*树结构(灰色)"""
        for node in rrt_planner.node_list:
            if node.parent is not None:
                self.draw_line(package, 
                              node.parent.x, node.parent.y,
                              node.x, node.y, 
                              color=Debug_Msg.GRAY)
    
    def draw_path(self, package, path, color=None):
        """绘制路径"""
        c = color if color is not None else Debug_Msg.WHITE
        for i in range(len(path) - 1):
            self.draw_line(package,
                          path[i][0], path[i][1],
                          path[i+1][0], path[i+1][1],
                          color=c)
    
    def draw_trajectory(self, package, tck, num_points=100, color=None):
        """绘制B样条轨迹"""
        from scipy.interpolate import splev
        import numpy as np
        c = color if color is not None else Debug_Msg.BLUE
        u = np.linspace(0, 1, num_points)
        x, y = splev(u, tck)
        for i in range(len(x) - 1):
            self.draw_line(package, x[i], y[i], x[i+1], y[i+1], color=c)
    
    def draw_prediction(self, package, predicted_states, color=None):
        """绘制MPC预测轨迹"""
        c = color if color is not None else Debug_Msg.YELLOW
        for i in range(len(predicted_states) - 1):
            self.draw_line(package,
                          predicted_states[i][0], predicted_states[i][1],
                          predicted_states[i+1][0], predicted_states[i+1][1],
                          color=c)

    def draw_line(self, package, x1, y1, x2, y2, color=None):
        msg = package.msgs.add()
        msg.type = Debug_Msg.LINE
        msg.color = color if color is not None else Debug_Msg.WHITE
        line = msg.line
        line.start.x = x1
        line.start.y = y1
        line.end.x = x2
        line.end.y = y2
        line.FORWARD = True
        line.BACK = True
    
    def draw_lines(self, package, x1, y1, x2, y2, color=None):
        for i in range(len(x1)):
            msg = package.msgs.add()
            msg.type = Debug_Msg.LINE
            msg.color = color if color is not None else Debug_Msg.WHITE
            line = msg.line
            line.start.x = x1[i]
            line.start.y = y1[i]
            line.end.x = x2[i]
            line.end.y = y2[i]
            line.FORWARD = True
            line.BACK = True

    def draw_point(self, package, x, y, color=None):
        c = color if color is not None else Debug_Msg.WHITE
        msg = package.msgs.add()
        # line 1
        msg.type = Debug_Msg.LINE
        msg.color = c
        line = msg.line
        line.start.x = x + 50
        line.start.y = y + 50
        line.end.x = x - 50
        line.end.y = y - 50
        line.FORWARD = True
        line.BACK = True
        # line 2
        msg = package.msgs.add()
        msg.type = Debug_Msg.LINE
        msg.color = c
        line = msg.line
        line.start.x = x - 50
        line.start.y = y + 50
        line.end.x = x + 50
        line.end.y = y - 50
        line.FORWARD = True
        line.BACK = True

    def draw_points(self, package, x, y, color=None):
        c = color if color is not None else Debug_Msg.WHITE
        for i in range(len(x)):
            # line 1
            msg = package.msgs.add()
            msg.type = Debug_Msg.LINE
            msg.color = c
            line = msg.line
            line.start.x = x[i] + 50
            line.start.y = y[i] + 50
            line.end.x = x[i] - 50
            line.end.y = y[i] - 50
            line.FORWARD = True
            line.BACK = True
            # line 2
            msg = package.msgs.add()
            msg.type = Debug_Msg.LINE
            msg.color = c
            line = msg.line
            line.start.x = x[i] - 50
            line.start.y = y[i] + 50
            line.end.x = x[i] + 50
            line.end.y = y[i] - 50
            line.FORWARD = True
            line.BACK = True
    
    def send(self, package):
        self.sock.sendto(package.SerializeToString(), self.debug_address)


if __name__ == '__main__':
    debugger = Debugger()
    package = Debug_Msgs()
    debugger.draw_circle(package, x=0, y=500)
    debugger.draw_line(package, x1=0, y1=2500, x2=600, y2=2500)
    debugger.draw_lines(package, x1=[0,0], y1=[0,2000], x2=[2000,2000], y2=[0,2000])
    debugger.draw_point(package, x=500, y=500)
    debugger.draw_points(package, x=[1000, 2000], y=[3000, 3000])
    debugger.send(package)
