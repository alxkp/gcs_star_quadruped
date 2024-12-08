from matplotlib import pyplot as plt
from pydrake.geometry.optimization import GraphOfConvexSetsOptions, Point # type: ignore

from src.gcs_star_drake.environment.basic import BasicEnvironment
from src.gcs_star_drake.visualization.basic import BasicEnvVisualizer
from src.gcs_star_drake.gcs.gcs_star import GCSStar

env: BasicEnvironment = BasicEnvironment()

# TODO: Get regions from env
regions = env.regions
source = env.x_start
target = env.x_goal

gcs_star = GCSStar(regions)

# Solve gcs star shorest path
path = gcs_star.SolveShortestPath(source, target, ))

