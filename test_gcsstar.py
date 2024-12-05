from os import environ
import mpld3
import numpy as np

from matplotlib import pyplot as plt
from pydrake.all import (
    AddDefaultVisualization,
    ConnectPlanarSceneGraphVisualizer,
    IrisFromCliqueCoverOptions,
    IrisInConfigurationSpaceFromCliqueCover,
    RandomGenerator,
    RigidTransform,
    RobotDiagramBuilder,
    SceneGraphCollisionChecker,
    StartMeshcat,
    VPolytope,
)

from pydrake.geometry.optimization import GraphOfConvexSetsOptions, HPolyhedron, Point
from scipy.spatial import ConvexHull

from manipulation import running_as_notebook
from manipulation.utils import ConfigureParser

from src.gcs.gcs_star import GCSStar

sr2 = np.sqrt(2)

# obstacles
obstacles = [
    np.array(
        [
            [3.4, 2.6],
            [3.4, 4.6],
            [2.4, 4.6],
            [2.4, 2.6],
            [1.4, 2.2],
            [3.8, 0.2],
            [4.8, 1.2],
        ]
    ),
    np.array([[1.4, 2.8], [2.2, 2.8], [2.2, 4.6], [1.4, 4.6]]),
    np.array([[1.0, 2.6], [1.0, 5.0], [0.4, 5.0], [0.4, 2.6]]),
    np.array([[1.0, 2.4], [1.0, 0.0], [0.4, 0.0], [0.4, 2.4]]),
    np.array([[3.8, 3.0], [3.8, 5.0], [4.4, 5.0], [4.4, 3.0]]),
    np.array([[3.8, 2.8], [3.8, 2.6], [5.0, 2.6], [5.0, 2.8]]),
]

# vertices of the safe regions
vertices = [
    np.array([[0.4, 0.0], [0.4, 5.0], [0.0, 5.0], [0.0, 0.0]]),
    np.array([[0.4, 2.4], [1.0, 2.4], [1.0, 2.6], [0.4, 2.6]]),
    np.array([[1.4, 2.2], [1.4, 4.6], [1.0, 4.6], [1.0, 2.2]]),
    np.array([[1.4, 2.2], [2.4, 2.6], [2.4, 2.8], [1.4, 2.8]]),
    np.array([[2.2, 2.8], [2.4, 2.8], [2.4, 4.6], [2.2, 4.6]]),
    np.array([[1.4, 2.2], [1.0, 2.2], [1.0, 0.0], [3.8, 0.0], [3.8, 0.2]]),
    np.array([[3.8, 4.6], [3.8, 5.0], [1.0, 5.0], [1.0, 4.6]]),
    np.array([[5.0, 0.0], [5.0, 1.2], [4.8, 1.2], [3.8, 0.2], [3.8, 0.0]]),
    np.array([[3.4, 2.6], [4.8, 1.2], [5.0, 1.2], [5.0, 2.6]]),
    np.array([[3.4, 2.6], [3.8, 2.6], [3.8, 4.6], [3.4, 4.6]]),
    np.array([[3.8, 2.8], [4.4, 2.8], [4.4, 3.0], [3.8, 3.0]]),
    np.array([[5.0, 2.8], [5.0, 5.0], [4.4, 5.0], [4.4, 2.8]]),
]

x_min = np.min(np.vstack(vertices), axis=0)
x_max = np.max(np.vstack(vertices), axis=0)

x_start = np.array([0.2, 0.2])
x_goal = np.array([4.8, 4.8])


def make_hpolytope(V):
    ch = ConvexHull(V)
    return HPolyhedron(ch.equations[:, :-1], -ch.equations[:, -1])


regions = [make_hpolytope(V) for V in vertices]


def environment_setup():
    plt.figure(figsize=(6, 6))
    plt.axis("square")

    plt.xlim([x_min[0], x_max[0]])
    plt.ylim([x_min[1], x_max[1]])

    tick_gap = 0.2

    def n_ticks(x_min, x_max):
        return round((x_max - x_min) / tick_gap) + 1

    x_ticks = np.linspace(x_min[0], x_max[0], n_ticks(x_min[0], x_max[0]))
    y_ticks = np.linspace(x_min[1], x_max[1], n_ticks(x_min[1], x_max[1]))
    plt.xticks(x_ticks)
    plt.yticks(y_ticks)

    label_gap = 0.5

    def keep_label(t):
        return np.isclose(t % label_gap, 0) or np.isclose(t % label_gap, label_gap)

    x_labels = [int(t) if keep_label(t) else "" for t in x_ticks]
    y_labels = [int(t) if keep_label(t) else "" for t in y_ticks]
    plt.gca().set_xticklabels(x_labels)
    plt.gca().set_yticklabels(y_labels)

    plt.grid()


def plot_trajectory(traj):
    plt.figure(figsize=(6, 6))

    for O in obstacles:
        plt.fill(*O.T, fc="lightcoral", ec="k", zorder=4)

    plt.plot(*traj.value(traj.start_time()), "kx")
    plt.plot(*traj.value(traj.end_time()), "kx")
    times = np.linspace(traj.start_time(), traj.end_time(), 1000)
    waypoints = traj.vector_values(times)
    plt.plot(*waypoints, "b", zorder=5)

    plt.axis("square")
    plt.xlim([x_min[0], x_max[0]])
    plt.ylim([x_min[1], x_max[1]])
    plt.xticks(range(6))
    plt.yticks(range(6))
    plt.grid(1)


def plot_velocity(traj):
    vel = traj.MakeDerivative()

    plt.figure(figsize=(6, 4))

    for i in range(vel.get_number_of_segments()):
        v = vel.segment(i)
        times = np.linspace(v.start_time(), v.end_time(), 500)
        values = v.vector_values(times)
        plt.plot(times, values[0], color="tab:blue")
        plt.plot(times, values[1], color="tab:orange")

    plt.xlim([traj.start_time(), traj.end_time()])
    plt.xticks(np.arange(int(np.ceil(traj.end_time() / 2))) * 2)
    plt.yticks(np.linspace(qdot_min, qdot_max, 5))
    plt.xlabel("Time $t$")
    plt.ylabel("Velocity $\dot{q}$")
    plt.grid()


def solve_min_time_STAR(order, continuity_order):
    trajopt = GCSStar(2)
    # breakpoint()
    gcs_regions = trajopt.AddRegions(regions, order=order)
    source = trajopt.AddRegions([Point(x_start)], order=0)
    target = trajopt.AddRegions([Point(x_goal)], order=0)
    trajopt.AddEdges(source, gcs_regions)
    trajopt.AddEdges(gcs_regions, target)
    trajopt.AddTimeCost()
    trajopt.AddVelocityBounds([qdot_min] * 2, [qdot_max] * 2)
    for o in range(1, continuity_order + 1):
        print(f"adding C{o} constraints")
        trajopt.AddContinuityConstraints(o)
    options = GraphOfConvexSetsOptions()
    [traj, result] = trajopt.SolvePath(source, target, options)

    return traj, result


environment_setup()

order = 1
continuity = 0

qdot_min = -1
qdot_max = 1

traj, results = solve_min_time_STAR(order, continuity)




