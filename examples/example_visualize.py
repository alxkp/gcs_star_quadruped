from matplotlib import pyplot as plt

from gcs_star_drake.environment.basic import BasicEnvironment
from gcs_star_drake.visualization.basic import BasicEnvVisualizer


def main():
    env: BasicEnvironment = BasicEnvironment()

    viz: BasicEnvVisualizer = BasicEnvVisualizer(env)

    viz.plot_environment()
    plt.show()


if __name__ == "__main__":
    main()
