from absl import app, flags, logging

from matplotlib import pyplot as plt

from gcs_star_drake.environment.basic import BasicEnvironment
from gcs_star_drake.visualization.basic import BasicEnvVisualizer

FLAGS = flags.FLAGS
flags.DEFINE_boolean("visualize", False, "Whether to visualize the environment.")

def check_intersections(env: BasicEnvironment):
        for curr_polytope in env.regions:
            intersections = []
            for other_polytope in env.regions:
                if not curr_polytope is other_polytope:
                    intersection = curr_polytope.Intersection(other_polytope)
                    if not intersection.IsEmpty():
                        intersections.append(intersection)
                        logging.debug(f"Intersection between {curr_polytope} and {other_polytope} = {intersection}")

            print(f"n_intersections = {len(intersections)}, tot:{len(env.regions)}\n")

        return

 

def main(argv):
    del argv # unused

    env: BasicEnvironment = BasicEnvironment()

    logging.info("Created Environment")

    if FLAGS.visualize:
        viz: BasicEnvVisualizer = BasicEnvVisualizer(env)

        viz.plot_environment()
        plt.show()

    check_intersections(env)
    logging.info("Checked intersections")
    logging.debug("Done")

if __name__ == "__main__":
    app.run(main)
