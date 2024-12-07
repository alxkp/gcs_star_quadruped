from matplotlib import pyplot as plt

from src.environment.basic import BasicEnvironment
from src.visualization.basic import BasicEnvVisualizer


def main():
    env: BasicEnvironment = BasicEnvironment()

    viz: BasicEnvVisualizer = BasicEnvVisualizer(env)

    viz.plot_environment()
    plt.show()


if __name__ == "__main__":
    main()
