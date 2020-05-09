import numpy as np
import matplotlib.pyplot as plt
from configurationspace import ConfigurationSpace
from robot import TurtleBot
import constants as CONST
import obstructioncheck as oc


class CSpacePlotter:
    def __init__(self, c_space):
        self.c_space = c_space

    def plotMap(self, fig, ax):
        height = self.c_space.height
        width = self.c_space.width
        padding =  self.c_space.padding

        sq1_pts = self.c_space.coord_sq1
        sq2_pts = self.c_space.coord_sq2
        sq3_pts = self.c_space.coord_sq3
        circle1 = self.c_space.circle1
        circle2 = self.c_space.circle2
        circle3 = self.c_space.circle3
        circle4 = self.c_space.circle4

        border_left_pts = self.c_space.boundary_left
        border_right_pts = self.c_space.boundary_right
        border_up_pts = self.c_space.boundary_up
        border_down_pts = self.c_space.boundary_down

        


        #poly_pts = self.c_space.coord_poly
        #rect_pts = self.c_space.coord_rect
        #rhom_pts = self.c_space.coord_rhom
        #circle = self.c_space.circle
        #ellipse = self.c_space.ellipse
        
        #fig = plt.figure()
        fig.set_dpi(100)
        fig.set_size_inches(8.5,6)
        #ax = plt.axes(xlim=(0,width),ylim=(0,height))
        cir1 = plt.Circle((circle1[1]), circle1[0], fc=None)
        cir2 = plt.Circle((circle2[1]) ,circle2[0], fc=None)
        cir3 = plt.Circle((circle3[1]), circle3[0], fc=None)
        cir4 = plt.Circle((circle4[1]), circle4[0], fc=None)
        sq1 = plt.Polygon(sq1_pts)
        sq2 = plt.Polygon(sq2_pts)
        sq3 = plt.Polygon(sq3_pts)
        border_left = plt.Polygon(border_left_pts)
        border_right = plt.Polygon(border_right_pts)
        border_up = plt.Polygon(border_up_pts)
        border_down = plt.Polygon(border_down_pts)
        borders = plt.Rectangle((-5,-5), width, height, alpha=1, fill=None, ec='b', linewidth=padding)
        

        shapes = [cir1, cir2, cir3, cir4, sq1, sq2, sq3, border_left, border_right, border_up, border_down]
        #for shape in shapes:
            #plt.gca().add_patch(shape)
        #    ax.add_patch(shape)

        self.plot_prm_graph(ax)

        self.plot_cspace_obstacles(ax)
        return fig

    def plot_prm_graph(self, ax, color=CONST.GRAPH_CLR):
        c_space_graph = self.c_space.graph
        num_edges = 0
        for parent, children in c_space_graph.items():
            #children = c_space_graph.get(tuple(node))
            for child in children:
                if not self.is_edge_intersecting(parent, child, self.c_space.obstacle_list):
                    ax.plot([parent[0], child[0]], [parent[1], child[1]], color=color)
                ax.scatter(child[0], child[1], alpha=0.8, c=color, edgecolors='none', s=30)
                num_edges += 1
        print('num_edges', num_edges)

    def plot_cspace_obstacles(self, ax):
        for obstacle in self.c_space.obstacle_list:
            cir = plt.Circle((obstacle[1]), obstacle[0], fc=None)
            ax.add_patch(cir)

    
    def is_edge_intersecting(self, p1, p2, obstacle_list):
        for obstacle in obstacle_list:
            if oc.is_line_circle_intersecting(p1, p2, obstacle[1], obstacle[0]):
                return True
        return False
    
        




if __name__ == "__main__":
    t_bot = TurtleBot(radius=(0.105), clearance=0.4, wheel_rad=(0.033), dist_bet_wheels=0.16)
    c_space = ConfigurationSpace(x_limit=(-5, 5), y_limit=(-5,5), radius_of_bot=t_bot.radius, clearance=0.45)
    plotter = CSpacePlotter(c_space)

    fig, ax = plt.subplots()
    plt.xlim(c_space.x_limit[0]-0.1, c_space.x_limit[1]+0.1)
    plt.ylim(c_space.y_limit[0]-0.1, c_space.y_limit[1]+0.1)
    plt.grid()
    ax.set_aspect('equal')

    plotter.plotMap(fig, ax)
    plt.savefig("c_space.png")

    fig.show()
    plt.draw()
    plt.show()
