import matplotlib.pyplot as plt
import numpy as np


class BasicEnvVisualizer:
    def __init__(self, env):
        self.env = env

    def setup_plot(self, figsize=(6, 6)):
        """Set up the plotting environment."""
        plt.figure(figsize=figsize)
        plt.axis("square")
        plt.xlim([self.env.x_min[0], self.env.x_max[0]])
        plt.ylim([self.env.x_min[1], self.env.x_max[1]])

        self._setup_ticks()
        plt.grid()

    def _setup_ticks(self, tick_gap=0.2, label_gap=0.5):
        """Set up plot ticks and labels."""

        def n_ticks(x_min, x_max):
            return round((x_max - x_min) / tick_gap) + 1

        x_ticks = np.linspace(
            self.env.x_min[0],
            self.env.x_max[0],
            n_ticks(self.env.x_min[0], self.env.x_max[0]),
        )
        y_ticks = np.linspace(
            self.env.x_min[1],
            self.env.x_max[1],
            n_ticks(self.env.x_min[1], self.env.x_max[1]),
        )

        plt.xticks(x_ticks)
        plt.yticks(y_ticks)

        def keep_label(t):
            return np.isclose(t % label_gap, 0) or np.isclose(t % label_gap, label_gap)

        x_labels = [int(t) if keep_label(t) else "" for t in x_ticks]
        y_labels = [int(t) if keep_label(t) else "" for t in y_ticks]

        plt.gca().set_xticklabels(x_labels)
        plt.gca().set_yticklabels(y_labels)

    def plot_environment(self):
        """Visualize the complete environment."""
        self.setup_plot()

        # Plot obstacles
        for O in self.env.obstacles:
            plt.fill(*O.T, fc="lightcoral", ec="k", zorder=4)

        # Plot start and goal
        plt.plot(*self.env.x_start, "kx")
        plt.plot(*self.env.x_goal, "kx")

        # Add labels
        plt.text(0.2, 0.35, "$q_0$", ha="center", va="bottom")
        plt.text(4.8, 4.65, "$q_T$", ha="center", va="top")

        # Plot safe regions
        for V in self.env.vertices:
            plt.fill(*V.T, fc="lightcyan", alpha=0.7, ec="k", zorder=4)

    def plot_trajectory(self, traj):
        """Plot a trajectory."""
        self.setup_plot()

        # Plot obstacles
        for O in self.env.obstacles:
            plt.fill(*O.T, fc="lightcoral", ec="k", zorder=4)

        # Plot trajectory
        plt.plot(*traj.value(traj.start_time()), "kx")
        plt.plot(*traj.value(traj.end_time()), "kx")
        times = np.linspace(traj.start_time(), traj.end_time(), 1000)
        waypoints = traj.vector_values(times)
        plt.plot(*waypoints, "b", zorder=5)

    def plot_velocity(self, traj, qdot_min, qdot_max):
        """Plot velocity profile of trajectory."""
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
