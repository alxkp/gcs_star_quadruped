import numpy as np
from pydrake.geometry.optimization import HPolyhedron
from scipy.spatial import ConvexHull


class BasicEnvironment:
    def __init__(self):
        (
            self.obstacles,
            self.vertices,
            self.x_min,
            self.x_max,
            self.x_start,
            self.x_goal,
        ) = self._generate_environment()
        self.regions = self._create_regions()

    def _generate_environment(self):
        """Generate the environment with obstacles and safe regions."""
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
            np.array([[0.2, 2.4], [1.2, 2.4], [1.2, 2.6], [0.2, 2.6]]),
            np.array([[1.4, 2.0], [1.4, 4.8], [1.0, 4.8], [1.0, 2.0]]),
            np.array([[1.3, 2.2], [2.4, 2.6], [2.4, 2.8], [1.3, 2.8]]),
            np.array([[2.2, 2.7], [2.4, 2.7], [2.4, 4.8], [2.2, 4.8]]),
            np.array([[1.4, 2.2], [1.0, 2.2], [1.0, 0.0], [3.8, 0.0], [3.8, 0.2]]),
            np.array([[3.8, 4.6], [3.8, 5.0], [1.0, 5.0], [1.0, 4.6]]),
            np.array([[5.0, 0.0], [5.0, 1.2], [4.8, 1.2], [3.8, 0.2], [3.6, 0.0]]),
            np.array([[3.4, 2.6], [4.9, 1.1], [5.0, 1.1], [5.0, 2.6]]),
            np.array([[3.4, 2.6], [3.8, 2.2], [3.8, 4.8], [3.4, 4.8]]),
            np.array([[3.6, 2.8], [4.6, 2.8], [4.6, 3.0], [3.6, 3.0]]),
            np.array([[5.0, 2.8], [5.0, 5.0], [4.4, 5.0], [4.4, 2.8]]),
        ]

        x_min = np.min(np.vstack(vertices), axis=0)
        x_max = np.max(np.vstack(vertices), axis=0)
        x_start = np.array([0.2, 0.2])
        x_goal = np.array([4.8, 4.8])

        return obstacles, vertices, x_min, x_max, x_start, x_goal

    def _create_regions(self):
        """Create HPolyhedron regions from vertices."""
        return [self._make_hpolytope(V) for V in self.vertices]

    @staticmethod
    def _make_hpolytope(V):
        """Create an HPolyhedron from vertices."""
        ch = ConvexHull(V)
        return HPolyhedron(ch.equations[:, :-1], -ch.equations[:, -1])
