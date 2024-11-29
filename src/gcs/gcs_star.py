from typing import Optional, List, Set, Dict, Tuple
import heapq
import numpy as np
from pydrake.all import GraphOfConvexSets, GraphOfConvexSetsOptions, ConvexSet, GcsTrajectoryOptimization
from pydrake.trajectories import CompositeTrajectory
from pydrake.solvers import MathematicalProgramResult
from pydrake.geometry.optimization import Point 


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

class GCSStar(GcsTrajectoryOptimization):
    """GCS* Algorithm building on Drake's mature GCS implementation"""
    def __init__(self, num_positions: int):
        super().__init__(num_positions)
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

        # # path has self.verticies
        # idxs = np.random.choice(len(candidate_path.vertices), size=num_samples, replace=False)

        # # check all existing paths if point is not present
        # # TODO: how do you get the cost of each vertex's solution?
        # min_cost = float('inf')
        # for path in existing_paths:
        #     for vtex in path.vertices:
        #         min_cost = min(vtex.get_cost(), min_cost)

        # for idx in idxs:
        #     test_vtex = candidate_path.vertices[idx]
        #     if test_vtex.get_cost() < min_cost:
        #         return True
        
        # return False # return false if cost is not lower

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
        # # NOTE: we should probably use an ordered set not a List[Verticies] to make this faster
        # # NOTE: needs review and testing

        # # path has self.verticies
        # idxs = np.random.choice(len(candidate_path.vertices), size=num_samples, replace=False)

        # # check all existing paths if point is not present
        # for idx in idxs:
        #     test_vertex = candidate_path.vertices[idx]
        #     for path in existing_paths:
        #         if test_vertex in path.vertices:
        #             return False
        # return True

    def get_successors(self, vertex: GraphOfConvexSets.Vertex) -> Set[GraphOfConvexSets.Vertex]:
        """
        Get successor vertices in the graph.

        Args:
            vertex: Vertex to find successors from
        Returns:
            succesors: List of successor vertices
        """
        gcs = self.graph_of_convex_sets()
        return {edge.target() for edge in gcs.Edges()
                if edge.source() == vertex}

        # # Implementation using GCS graph structure
        # # NOTE: Needs review and testing

        # successors : Set[GraphOfConvexSets.Vertex] = set()

        # for path in self.vertex_paths[vertex]:
        #     for idx, vtex in enumerate(path.vertices):
        #         if vtex == vertex:
        #             # add all to successors
        #             successors.update(path.vertices[idx+1:])
                    
        # return successors


    def solve_convex_restrictions(self, path: Path) -> Tuple[float, Optional[CompositeTrajectory]]:
        """
        Solve convex optimization problem for a given path.
        
        Args:
            path: Path to solve optimizaiton problem with
        Returns:
            cost and trajectory if feasible
        """
        try:
            # Setup options
            options = GraphOfConvexSetsOptions()
            options.convex_relaxation = False
            
            # Add regions for path
            regions = [v.ambient_set() for v in path.vertices]
            subgraph = self.AddRegions(regions, 
                                     [(i,i+1) for i in range(len(regions)-1)],
                                     order=1)
            
            # Solve
            traj, result = self.SolveConvexRestriction(subgraph.Vertices(), options)
            
            if result.is_success():
                return result.get_optimal_cost(), traj
            return float('inf'), None
            
        except RuntimeError:
            return float('inf'), None

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
              use_optimal: bool = True) -> Optional[Tuple[Path, CompositeTrajectory]]:
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

        while self.priority_queue:
            current_path = heapq.heappop(self.priority_queue)

            # Check if we've reached target
            if current_path.terminal_vertex == target_vertex:
                return current_path, current_path.trajectory
            
            # Get successors of terminal vertex
            for successor in self.get_successors(current_path.terminal_vertex):
                new_path = Path(current_path.vertices + [successor], current_path.cost_to_come)

                # Solve convex restriction to get true cost
                cost, traj = self.solve_convex_restrictions(new_path)
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
