from matplotlib import pyplot as plt
from pydrake.geometry.optimization import ConvexSet, GraphOfConvexSetsOptions, Point # type: ignore

from gcs_star_drake.environment.basic import BasicEnvironment
from gcs_star_drake.visualization.basic import BasicEnvVisualizer
from gcs_star_drake.gcs.gcs_star import GCSStar
from gcs_star_drake.gcs.gcs_star_trajopt import f_estimator

env: BasicEnvironment = BasicEnvironment()

# TODO: Get regions from env
regions = env.regions


gcs_star = GCSStar(regions)

source = Point(env.x_start)
target = Point(env.x_goal)
vertices = env.vertices
dummy_source = gcs_star.mutable_gcs().AddVertex(source)
source_v = dummy_source

dummy_target = gcs_star.mutable_gcs().AddVertex(target)
target_v = dummy_target

f_est = f_estimator(vertices, target_v)

# Solve gcs star shorest path
path = gcs_star.SolveShortestPath(source_v, target_v, f_est)

print(path)

point_b = point + np.ndarray([0, 0.1])