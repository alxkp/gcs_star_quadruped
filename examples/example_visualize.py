import numpy as np

from absl import app, flags, logging

from matplotlib import pyplot as plt
from pydrake.geometry.optimization import GraphOfConvexSetsOptions, Point

from gcs_star_drake.environment.basic import BasicEnvironment
from gcs_star_drake.visualization.basic import BasicEnvVisualizer
from gcs_star_drake.gcs.gcs_star_trajopt import GCSStarTrajectoryOptimization


FLAGS = flags.FLAGS
flags.DEFINE_boolean("visualize", False, "Whether to visualize the environment.")
flags.DEFINE_boolean("debug", False, "Whether to print debug messages.")


def solve_min_distance(regions, x_start, x_goal, order=1, qdot_min=-1, qdot_max=1):
    trajopt = GCSStarTrajectoryOptimization(2)

    gcs_regions = trajopt.AddRegions(regions, order=order)

    # BUG: fix this (cringe)
    def expand_point_to_triangle(point: np.ndarray, epsilon=1e-3):
        x = point[0]
        y = point[1]

        return np.array([[x, y], [x + epsilon, y], [x, y+epsilon]])

    x_start = expand_point_to_triangle(x_start)
    x_goal = expand_point_to_triangle(x_goal)
        
    start = BasicEnvironment()._make_hpolytope(x_start)
    goal = BasicEnvironment()._make_hpolytope(x_goal)
    source = trajopt.AddRegions([start], order=0)
    target = trajopt.AddRegions([goal], order=0)

    trajopt.AddEdges(source, gcs_regions)
    trajopt.AddEdges(gcs_regions, target)

    trajopt.AddPathLengthCost()
    trajopt.AddVelocityBounds([qdot_min] * 2, [qdot_max] * 2)

    options = GraphOfConvexSetsOptions()
    [traj, result] = trajopt.SolvePath(source, target, options)
    print(f"result.is_success() = {result.is_success()}")
    if result.is_success():
        print(traj.get_segment_times())
    return traj

def check_intersections(env: BasicEnvironment):
        for curr_polytope in env.regions:
            intersections = []
            for other_polytope in env.regions:
                if not curr_polytope is other_polytope:
                    intersection = curr_polytope.Intersection(other_polytope)
                    if not intersection.IsEmpty():
                        intersections.append(intersection)
                        logging.debug(f"Intersection between {curr_polytope} and {other_polytope} = {intersection}")

            # if FLAGS.debug:
                # print(f"n_intersections = {len(intersections)}, tot:{len(env.regions)}\n")

        return

 

def main(argv):
    del argv # unused

    env: BasicEnvironment = BasicEnvironment()

    logging.info("Created Environment")

    if FLAGS.visualize:
        viz: BasicEnvVisualizer = BasicEnvVisualizer(env)
    
        viz.plot_environment()
        plt.show()
    
    if FLAGS.debug:
        logging.set_verbosity(logging.DEBUG)
        check_intersections(env)
        logging.info("Checked intersections")
        logging.debug("Done")

    breakpoint()
    traj = solve_min_distance(env.regions, env.x_start, env.x_goal)

    if FLAGS.visualize:
        viz:BasicEnvVisualizer = BasicEnvVisualizer(env)
        viz.plot_trajectory(traj)
        plt.show()

if __name__ == "__main__":
    app.run(main)
