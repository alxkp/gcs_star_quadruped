from typing import Optional, List, Set, Dict, Tuple, cast
import heapq
import numpy as np
# import openGJK_cython as opengjk
# from gcs_star import GCSStar

from gcs_star_drake.gcs.gcs_star import GCSStar

from pydrake.all import GraphOfConvexSets, GraphOfConvexSetsOptions, ConvexSet, GcsTrajectoryOptimization
from pydrake.trajectories import BezierCurve, CompositeTrajectory, PiecewiseTrajectory
from pydrake.solvers import MathematicalProgramResult
from pydrake.geometry.optimization import CartesianProduct, HPolyhedron, Point, ConvexSet


class Path: 
    """Represents a path through the graph with its associated costs and trajectory."""
    def __init__(self, vertices: List[GraphOfConvexSets.Vertex], 
                 cost_to_come: float,
                 trajectory: Optional[CompositeTrajectory] = None):
        self.vertices = vertices
        self.cost_to_come = cost_to_come
        self.trajectory = trajectory
        self.terminal_vertex = vertices[-1]

    def __lt__(self, other):
        # For priority queue ordering
        return self.cost_to_come < other.cost_to_come
    
class GCSStarTrajectoryOptimization(GcsTrajectoryOptimization):

        def __init__(self, num_positions:int, continuous_revolute_joints:List[int]=None) -> None:
            super().__init__(num_positions, continuous_revolute_joints or [])
            self._gcs_star = None # gets initialized in AddRegions
            self._subgraphs = {}

        def AddRegions(self, # type: ignore
                       regions: List[HPolyhedron],
                       order: int,
                       h_min: float=1e-6,
                       h_max: float=20.0,
                       name:str=""
                       ) -> GcsTrajectoryOptimization.Subgraph:
            # need regions for gcsstar
            if self._gcs_star is None:
                self._gcs_star = GCSStar(regions)
            else:
                self._gcs_star.regions.extend(regions)



            # create subgraph from parent
            subgraph = super().AddRegions(regions, 
                                          [],  # assume no edges init; discover later
                                          order, 
                                          h_min, 
                                          h_max, 
                                          name
                                      ) # HACK: this might error because there's no edges between regions

            self._subgraphs[subgraph] = subgraph
            return subgraph

        def SolvePath(self,
                      source: GcsTrajectoryOptimization.Subgraph,
                      target: GcsTrajectoryOptimization.Subgraph,
                      options: GraphOfConvexSetsOptions = None # type: ignore
                      ) -> tuple[CompositeTrajectory,
            MathematicalProgramResult]:

            if self._gcs_star is None:
                raise RuntimeError("GCSStarTrajectoryOptimization must have regions added before calling SolvePath.  \n Call AddRegions first befor calling SolvePath")

            def h_estimator(verticies: Tuple[GraphOfConvexSets.Vertex, ...]) -> float:
                """Returns chebyshev center distance cost of path"""
                current_v= verticies[-1]
                # TODO: Need to get correct target and make sure its actually correct
                target_set: CartesianProduct = target.Vertices()[0].set() # type: ignore


                def compute_centers(target_set: CartesianProduct | HPolyhedron) -> List:
                    if isinstance(target_set, CartesianProduct):
                        centers = []
                        for i in range(target_set.num_factors()):
                            factor = target_set.factor(i)
                            if isinstance(factor, HPolyhedron):
                                cheb = factor.ChebyshevCenter()
                                if len(cheb) > 1:
                                    centers.append(cheb)
                                else:
                                    continue
                            else:
                                raise ValueError("Unsupported type")
                    elif isinstance(target_set, HPolyhedron):
                        centers = [target_set.ChebyshevCenter()]

                    else:
                        raise ValueError("Unsupported type (outer loop)")
                    
                    return centers
                
                target_centers : List = compute_centers(target_set)

                target_centroid = np.mean(target_centers, axis=0)

                current_set: CartesianProduct = current_v.set()

                current_centers = compute_centers(current_set)

                current_centroid = np.mean(current_centers, axis=0)

                # current_centroid : List =  compute_centers(current_set)

                return float(np.linalg.norm(target_centroid - current_centroid))

            # source and target
            source_vertex = source.Vertices()[0]
            target_vertex = target.Vertices()[0]

            #breakpoint()

            # solve program
            vertex_path = self._gcs_star.SolveShortestPath(source_vertex, target_vertex, h_estimator)
            breakpoint()

            # convert path to trajectory with super class
            # path is a list of vertices, we need a list of edges
            if vertex_path: # none checking here
                edge_path = self._gcs_star.get_path_edges(vertex_path)


            ###########################################################################
                                    # this works #
            ###########################################################################

                gcs_result =  self._gcs_star.mutable_gcs().SolveConvexRestriction(active_edges=edge_path, options=options)
                # sus_gcs_result =  self._gcs_star.gcs().SolveConvexRestriction(active_edges=edge_path, options=options)
                breakpoint()

                if gcs_result.is_success():
                    return (self.ReconstructTrajectory(edge_path, gcs_result), gcs_result)
                else:
                    raise RuntimeError("Failed to solve convex restriction")

            return None, None # type: ignore something failed

        def AddEdges(self,
                     from_subgraph: GcsTrajectoryOptimization.Subgraph,
                     to_subgraph: GcsTrajectoryOptimization.Subgraph,
                    subspace: ConvexSet = None, # type: ignore
                     edges_between_regions: list[tuple[int,
                     int]] | None = None,
                     edge_offsets = None,
                     ) -> GcsTrajectoryOptimization.EdgesBetweenSubgraphs:
                if self._gcs_star is None:
                    raise RuntimeError("GCSStarTrajectoryOptimization must have regions added before calling AddEdges.  \n Call AddRegions first befor calling AddEdges")


                # call parent implementation, no edges assumed here since we discover them during our search
                return super().AddEdges(from_subgraph, to_subgraph, subspace, edges_between_regions=[], edge_offsets=edge_offsets)
        
        def _vertex_to_subgraph(self, vertex: GraphOfConvexSets.Vertex):
            subgraph = GcsTrajectoryOptimization.Subgraph()
            subgraph.AddRegions(vertex.set())

        def ReconstructTrajectory(self, path_edges: List[GraphOfConvexSets.Edge], result: MathematicalProgramResult):
            """Reconstructs a Trajectory from the Solved Shortest Path"""
            # Extract the path from the edges.
            bezier_curves: List[BezierCurve] = []
            current_time = 0.0

            for i, e in enumerate(path_edges):
                left_center = e.u().set().ChebyshevCenter()
                right_center = e.v().set().ChebyshevCenter()

                u_ctlpt = np.array(result.GetSolution(e.xu())).reshape(2, 1)
                v_ctlpt = np.array(result.GetSolution(e.xv())).reshape(2, 1)

                h = result.GetSolution(e.xv())[-1] if i > 0 else result.GetSolution(e.xu())[-1]
                control_points = np.hstack([ u_ctlpt, v_ctlpt ])
                
                breakpoint()

                # construct a bezier curve from the edge
                
                curve = BezierCurve(current_time, current_time + h, control_points)
                bezier_curves.append(curve)

                # update time for next segment
                current_time += h

            return CompositeTrajectory(bezier_curves)
