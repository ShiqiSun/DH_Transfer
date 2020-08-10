import numpy as np
from RRT.src.rrt.rrt import RRT
from RRT.src.search_space.search_space import SearchSpace
from RRT.src.utilities.plotting import Plot
from Constant import *


def obplot(obstacles, goal1):
    X_dimensions = np.array([(0, EnviromBoundx), (0, EnviromBoundy)])  # dimensions of Search Space
    # obstacles
    Obstacles = np.array([(ob[0][0], ob[0][1], ob[2][0], ob[2][1]) for ob in obstacles])
    X = SearchSpace(X_dimensions, Obstacles)


    path1 = [[0, 0], goal1]
    plot = Plot("obst_env")
    plot.plot_path(X, path1)
    plot.plot_obstacles(X, Obstacles)
    # plot.plot_start(X, x_init)
    # plot.plot_goal(X, x_goal)
    plot.draw(auto_open=True)