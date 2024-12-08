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




                gcs_result =  self._gcs_star.gcs().SolveConvexRestriction(active_edges=edge_path, options=options)

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

            for e in path_edges:
                # Extract phi from the soln. to rescale control points and duration
                # in case we get the relaxed soln
                phi_inv : float = 1 / result.GetSolution(e.phi())
                
                # Extract control points from the soln.
                num_control_points : int = np.size(result.GetSolution(e.xu())) // self.num_positions() #vertex_to_subgraph[e.u()].order() + 1
                edge_path_points : np.ndarray = phi_inv * np.array(result.GetSolution(e.xu())).reshape((self.num_positions(), num_control_points))

                # Extract the duration from the solution
                h : float = phi_inv * result.GetSolution(e.xu())[-1]
                start_time : float = 0 if not bezier_curves else bezier_curves[-1].end_time()

                # Skip edges with a single control point that spend near zero time in the
                # region, since zero order continuity constraint is sufficient. These edges
                # would result in a discontinuous trajectory for velocities and higher
                # derivatives.
                if (num_control_points > 1 and h < 1e-3):
                    raise RuntimeError("GcsTrajectoryOptimization returned a trajectory segment", 
                                    "with near-zero duration. Make sure you set h_min to be ",
                                f"at least {1e-3}", 
                                "for regions whose subgraph order is at ",
                                "least 1 or impose velocity limits.")
                # HACK: Just create the BezierCurve - skip the h_min check for now since it's not critical
                bezier_curves.append(BezierCurve(start_time, start_time + h, edge_path_points))
                # elif not (num_control_points == 1 and vertex_to_subgraph[e.u()].h_min() == 0):
                #     bezier_curves.append(BezierCurve(start_time, start_time + h, edge_path_points))

            # Get the final control points from the solution.
            last_edge : GraphOfConvexSets.Edge = path_edges[-1]

            phi_inv : float = 1. / result.GetSolution(last_edge.phi())

            #num_control_points : int = self._subgraphs[last_edge.v()].order() + 1 # NOTE: not sure if this works for our set of subgraphs
            num_control_points : int = result.GetSolution(last_edge.xv()).size // self.num_positions()  # Use same approach as above
            edge_path_points : np.ndarray = phi_inv * np.array(result.GetSolution(last_edge.xv())).reshape(self.num_positions(), num_control_points) # note: also might not work

            h : float = phi_inv * result.GetSolution(last_edge.xv())[-1]

            start_time : float = 0 if not bezier_curves else bezier_curves[-1].end_time()

            if num_control_points > 1 and h < PiecewiseTrajectory.kEpsilonTime: # not sure if this is exposed in pydrake
                raise RuntimeError(f"GcsTrajectoryOptimization returned a trajectory segment with near-zero duration. Make sure you set h_min to be at least {PiecewiseTrajectory.kEpsilonTime} for " "regions whose subgraph order is at least 1 or impose velocity limits.")

            

            # if not (num_control_points == 1 and vertex_to_subgraph[last_edge.v()].h_min() == 0): # dont need elif here
            #     bezier_curves.append(BezierCurve(start_time, start_time + h, edge_path_points))
            # HACK
            bezier_curves.append(BezierCurve(start_time, start_time + h, edge_path_points))

            return CompositeTrajectory(bezier_curves)
        ###################################################################


    ########################################################################
    ### NOTE: Very Rough Draft. Definitely needs second look and testing ###
    ########################################################################

class GcsStarTrajOpt(GcsTrajectoryOptimization):
        """GCS* Algorithm building on Drake's mature GCS implementation"""
        def __init__(self, num_positions: int):
            super().__init__(num_positions)
            self.gcs_star = GCSStar()
            # Priority queue Q ordered by f̃ values
            self.priority_queue: List[Path] = []
            # From paper - S maps vertices to sets of paths reaching them
            self.vertex_paths: Dict[GraphOfConvexSets.Vertex, Set[Path]] = {}

        def reaches_cheaper(self, candidate_path: Path, existing_paths: Set[Path], num_samples: int = 1) -> bool:
            """
            Check if candidate path reaches any point cheaper than existing paths.
            Uses sampling-based implementation for speed.
            Equation (2) From paper using sampling.

            Args:
                candidate_path: Candidate path to be checked
                existing_paths: Set of existing paths
                num_samples: Number of samples to sample with
            Returns:
                True if the candidate path reaches any point with lowe cost than any other existing path
            """
            # Sample points in terminal vertex's convex set
            # Check if candidate path reaches any point with lower cost
            # than all existing paths

            terminal_set = candidate_path.terminal_vertex.ambient_set()

            # Sample points in the terminal set
            samples = self.sample_from_set(terminal_set, num_samples)

            for x in samples:
                # Get cost-to-come for candidate path for point x
                candidate_cost = self.eval_cost_to_come(candidate_path, x)
                if candidate_cost == float('inf'):
                    continue

                # Check if this point is reached cheaper by any existing path
                cheaper = True
                for existing_path in existing_paths:
                    existing_cost = self.eval_cost_to_come(existing_path, x)
                    if existing_cost <= candidate_cost:
                        cheaper = False
                        break

                if cheaper:
                    return True

            return False # return false if cost is not lower

        def reaches_new(self, candidate_path: Path, existing_paths: Set[Path], num_samples: int = 1) -> bool:
            """
            Check if candidate path reaches any previously unreached points.
            Uses sampling-based implementation for speed.
            Equation (3) from paper.

            Args: 
                candidate_path: Candidate path to be checked
                existing_paths: Set of existing paths
                num_samples: Number of samples to sample with
            Returns:
                True if the candidate path reaches new, False otherwise
            """
            # Sample points in terminal vertex's convex set
            # Check if candidate path reaches any point unreachable
            # by existing paths
            terminal_set = candidate_path.terminal_vertex.ambient_set()
            samples = self.sample_from_set(terminal_set, num_samples)

            for x in samples:
                # Check if candidate path can reach this point
                if self.eval_cost_to_come(candidate_path, x) == float('inf'):
                    continue

                # Check if this point is unreachable by all existing paths
                unreached = True
                for existing_path in existing_paths:
                    if self.eval_cost_to_come(existing_path, x) < float('inf'):
                        unreached = False
                        break

                if unreached:
                    return True

            return False

        def get_successors(self, vertex: GraphOfConvexSets.Vertex) -> Set[GraphOfConvexSets.Vertex]:
            """
            Get successor vertices in the graph.

            Args:
                vertex: Vertex to find successors from
            Returns:
                succesors: List of successor vertices
            """
            gcs = self.graph_of_convex_sets()
            return {edge.v() for edge in gcs.Edges()
                if edge.u() == vertex}


        # def solve_convex_restrictions(self, path: Path) -> Tuple[float, Optional[CompositeTrajectory], MathematicalProgramResult]:
        #     """
        #     Solve convex optimization problem for a given path.

        #     Args:
        #         path: Path to solve optimizaiton problem with
        #     Returns:
        #         cost and trajectory if feasible
        #     """

        #     try:
        #         # Setup options
        #         options = GraphOfConvexSetsOptions()
        #         options.convex_relaxation = False

        #         # Add regions for path
        #         regions = self. #[self.regio() for v in path.vertices]
        #         subgraph = self.AddRegions(regions, 
        #                                  [(i,i+1) for i in range(len(regions)-1)],
        #                                  order=1)


        #         for vtex in path.vertices:
        #             for region in regions:
        #                 cost = 1./opengjk.gjk(vtex.x(), region)
        #                 sys.exit(1)
        #                 vtex.AddCost(cost)

        #         # for vtex in path.vertices:
        #         #     for region in regions:
        #         #         cost = 1./opengjk.gjk( vtex.x(), region)
        #         #         print(cost, file=sys.stderr)
        #         #         vtex.AddCost(cost)

        #         # Solve
        #         traj, result = self.SolveConvexRestriction(subgraph.Vertices(), options)

        #         if result.is_success():
        #             return result.get_optimal_cost(), traj, result
        #         return float('inf'), None

        #     except RuntimeError:
        #         return float('inf'), None


        #     GcsTrajectoryOptimization::SolveConvexRestriction(
        #     const std::vector<const Vertex*>& active_vertices,
        #     const GraphOfConvexSetsOptions& options) {
        #   DRAKE_DEMAND(active_vertices.size() > 1);

        #   std::vector<const Edge*> active_edges;
        #   for (size_t i = 0; i < active_vertices.size() - 1; ++i) {
        #     bool vertices_connected = false;
        #     for (const Edge* e : active_vertices[i]->outgoing_edges()) {
        #       if (e->v().id() == active_vertices[i + 1]->id()) {
        #         if (vertices_connected) {
        #           throw std::runtime_error(fmt::format(
        #               "Vertex: {} is connected to vertex: {} through multiple edges.",
        #               active_vertices[i]->name(), active_vertices[i + 1]->name()));
        #         }
        #         active_edges.push_back(e);
        #         vertices_connected = true;
        #       }
        #     }

        #     if (!vertices_connected) {
        #       throw std::runtime_error(fmt::format(
        #           "Vertex: {} is not connected to vertex: {}.",
        #           active_vertices[i]->name(), active_vertices[i + 1]->name()));
        #     }
        #   }

        #   solvers::MathematicalProgramResult result =
        #       gcs_.SolveConvexRestriction(active_edges, options);
        #   if (!result.is_success()) {
        #     return {CompositeTrajectory<double>({}), result};
        #   }

        #   return {ReconstructTrajectoryFromSolutionPath(active_edges, result), result};
    # }
        def compute_heuristic(self, vertex: GraphOfConvexSets.Vertex,
                              target: GraphOfConvexSets.Vertex) -> float:
            """
            Compute admissible heuristic cost-to-go as Euclidean distance between centers..

            Args:
                vertex: vertex to be used
                target: target vertex for heurisitic calculation
            Returns:
                float value of the cost from whatever heuristic we choose 
            """
            # Implement simple admissible heuristic
            # Could be Euclidean distance or other problem-specific metric
            # Get centers of vertex and target sets
            vertex_center = vertex.set().ChebyshevCenter()
            target_center = target.set().ChebyshevCenter()

            # Euclidean distance is admissible since it's always <= true cost
            return np.linalg.norm(target_center - vertex_center)


        def sample_from_set(self, convex_set: ConvexSet, num_samples: int) -> List[np.ndarray]:
            """Sample points from a convex set by solving with random objectives."""
            samples = []
            dim = convex_set.dimension()

            # Create temporary GCS for sampling
            temp_gcs = GcsTrajectoryOptimization(self.num_positions)
            vertex = temp_gcs.AddVertex(convex_set)

            for _ in range(num_samples):
                # Add random objective to get different points
                random_objective = np.random.randn(dim)
                vertex.AddCost(random_objective.dot(vertex.x()))

                # Solve the optimization
                try:
                    _, result = temp_gcs.SolveShortestPath(vertex, vertex)
                    if result.is_success():
                        point = result.GetSolution(vertex.x())
                        samples.append(point)
                except RuntimeError:
                    continue

            return samples

        def eval_cost_to_come(self, path: Path, point: np.ndarray) -> float:
            """Evaluate cost-to-come to reach specific point via path."""
            # Create temporary GCS for evaluation
            temp_gcs = GcsTrajectoryOptimization(self.num_positions)

            try:
                # Add path regions
                regions = [v.ambient_set() for v in path.vertices]
                subgraph = temp_gcs.AddRegions(regions, 
                                               [(i,i+1) for i in range(len(regions)-1)],
                                               order=1)

                # Create singleton point set as target
                target_region = Point(point)  # Use Drake's Point class
                target = temp_gcs.AddRegions([target_region], order=0)[0]

                # Connect subgraph to target point
                temp_gcs.AddEdges(subgraph, target)

                # Add costs
                temp_gcs.AddPathLengthCost()
                temp_gcs.AddTimeCost()

                # Solve
                options = GraphOfConvexSetsOptions()
                options.convex_relaxation = False
                traj, result = temp_gcs.SolvePath(subgraph.Vertices()[0], target, options)

                return result.get_optimal_cost() if result.is_success() else float('inf')

            except RuntimeError:
                return float('inf')


        def solve(self, source_vertex: GraphOfConvexSets.Vertex, 
                  target_vertex: GraphOfConvexSets.Vertex, 
                  use_optimal: bool = True) -> Optional[Tuple[Path, CompositeTrajectory, MathematicalProgramResult]]:
            """
            Solves GCS* path finding problem.
            If use_optimal=True, uses ReachesCheaper for optimality.
            Otherwise uses ReachesNew for faster but suboptimal solutions.

            Args:
                source_vertex: Starting vertex
                target_vertex: target vertex
                use_optimal: boolean variable to determine if we want reaches cheaper or new

            Returns:
                Tuple of path found and trajectory, or None if none found
            """
            # Initialize with source vertex
            start_path = Path([source_vertex], 0)
            heapq.heappush(self.priority_queue, start_path)
            self.vertex_paths[source_vertex] = {start_path}

            result = None

            while self.priority_queue:
                current_path = heapq.heappop(self.priority_queue)

                # Check if we've reached target
                if current_path.terminal_vertex == target_vertex:
                    return current_path, current_path.trajectory, result

                # Get successors of terminal vertex
                for successor in self.get_successors(current_path.terminal_vertex):
                    new_path = Path(current_path.vertices + [successor], current_path.cost_to_come)

                    # Solve convex restriction to get true cost
                    cost, traj, result = self.solve_convex_restrictions(new_path)
                    if cost == float("inf"):
                        continue
                    new_path.cost_to_come = cost
                    new_path.trajectory = traj

                    # Domination checks
                    if successor not in self.vertex_paths:
                        self.vertex_paths[successor] = set()

                    domination_check = (self.reaches_cheaper if use_optimal else self.reaches_new)

                    if domination_check(new_path, self.vertex_paths[successor]):
                        self.vertex_paths[successor].add(new_path)
                        heapq.heappush(self.priority_queue, new_path)

            return None # No path found


        def SolvePath(self, source: GcsTrajectoryOptimization.Subgraph, target: GcsTrajectoryOptimization.Subgraph, options=None):
            print("Entering custom SolvePath")  # This might show up
            # Call your custom solve method

            # if no options specified use default
            if not options.convex_relaxation:
                options.convex_relaxation = True

            if not options.preprocessing:
                options.preprocessing = True

            if not options.max_rounded_paths:
                options.max_rounded_paths = True

            # Get vertices from subgraph    
            source_vertex = source.vertices[0]
            target_vertex = target.vertices[0]

            if source.size() != 1:
                dummy_source = self.gcs_star.mutable_gcs().AddVertex()
                source_vertex = dummy_source
                for v in source.vertices:
                    self.gcs_star.mutable_gcs().AddEdge(dummy_source, v)

            if target.size() != 1:
                dummy_target = self.gcs_star.mutable_gcs().AddVertex()
                target_vertex = dummy_target
                for v in target.vertices:
                    self.gcs_star.mutable_gcs().AddEdge(dummy_target, v)

            result = self.gcs_star.SolveShortestPath(source_vertex, target_vertex, options, None) # TODO: Pass in f estimator

            if not result.is_success():
                return CompositeTrajectory, result

            k_tol = 1.0
            path_edges = self.gcs_star.mutable_gcs().GetSolutionPath(source_vertex, target_vertex, result, k_tol)

            # Remove dummy edges from path
            if dummy_source != None:
                assert(not path_edges.empty())
                path_edges.erase(path_edges.begin())

            if dummy_target != None:
                assert(not path_edges.empty())
                path_edges.erase(path_edges.end() - 1)

            return self.ReconstructTrajectory(path_edges, result), result
