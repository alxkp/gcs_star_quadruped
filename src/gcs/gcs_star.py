from typing import Optional, List, Set, Dict, Tuple
import numpy as np
from pydrake.all import GraphOfConvexSets, GraphOfConvexSetsOptions, ConvexSet, GcsTrajectoryOptimization
from pydrake.trajectories import CompositeTrajectory
from pydrake.solvers import MathematicalProgramResult
import heapq

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

class GCSStar:
    def __init__(self, num_positions: int):
        self.gcs = GraphOfConvexSets()
        self.num_positions = num_positions
        self.priority_queue: List[Path] = []
        # Maps vertices to sets of paths that reach them (From Paper Eq. )
        self.vertex_paths: Dict[GraphOfConvexSets.Vertex, Set[Path]] = {}

    def add_regions(self, regions: List[ConvexSet], edges: List[Tuple[int, int]]) -> None:
        """
        Adds regions edges to graph.
            
        Args:
            regions: list of Convex Sets to be added to graph
            edges: edge [u, v] to be added to the graph
        Returns:
            None
        """
        # Implementation using GcsTrajectoryOptimization.AddRegions
        pass

    def reaches_cheaper(self, candidate_path: Path, existing_paths: Set[Path], num_samples: int = 1) -> bool:
        """
        Check if candidate path reaches any point cheaper than existing paths.
        Uses sampling-based implementation for speed.

        Args:
            candidate_path: Candidate path to be checked
            existing_paths: Set of existing paths
            num_samples: Number of samples to sample with
        Returns:
            True if the candidate path reaches cheaper, False otherwise
        """
        # Sample points in terminal vertex's convex set
        # Check if candidate path reaches any point with lower cost
        # than all existing paths

        # path has self.verticies
        idxs = np.random.choice(len(candidate_path.vertices), size=num_samples, replace=False)

        # check all existing paths if point is not present
        # TODO: how do you get the cost of each vertex's solution?
        min_cost = float('inf')
        for path in existing_paths:
            for vtex in path.vertices:
                min_cost = min(vtex.get_cost(), min_cost)

        for idx in idxs:
            test_vtex = candidate_path.vertices[idx]
            if test_vtex.get_cost() < min_cost:
                return True
        
        return False # return false if cost is not lower


    def reaches_new(self, candidate_path: Path, existing_paths: Set[Path], num_samples: int = 1) -> bool:
        """
        Check if candidate path reaches any previously unreached points.
        Uses sampling-based implementation for speed.
        
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
        # NOTE: we should probably use an ordered set not a List[Verticies] to make this faster
        # NOTE: needs review and testing

        # path has self.verticies
        idxs = np.random.choice(len(candidate_path.vertices), size=num_samples, replace=False)

        # check all existing paths if point is not present
        for idx in idxs:
            test_vertex = candidate_path.vertices[idx]
            for path in existing_paths:
                if test_vertex in path.vertices:
                    return False
        return True

    def solve_convex_restrictions(self, path: Path) -> Tuple[float, Optional[CompositeTrajectory]]:
        """
        Solve convex optimization problem for a given path.
        
        Args:
            path: Path to solve optimizaiton problem with
        Returns:
            cost and trajectory if feasible
        """
        # Use GcsTrajectoryOptimization.SolveConvexRestriction
        # Return cost and trajectory if feasible, otherwise (inf, None)
        pass

    def compute_heuristic(self, vertex: GraphOfConvexSets.Vertex,
                         target: GraphOfConvexSets.Vertex) -> float:
        """
        Compute admissible heuristic cost-to-go.
            
        Args:
            vertex: vertex to be used
            target: target vertex for heurisitic calculation
        Returns:
            float value of the cost from whatever heuristic we choose 
        """
        # Implement simple admissible heuristic
        # Could be Euclidean distance or other problem-specific metric
        pass

    def get_successors(self, vertex: GraphOfConvexSets.Vertex) -> Set[GraphOfConvexSets.Vertex]:
        """
        Get successor vertices in the graph.

        Args:
            vertex: Vertex to find successors from
        Returns:
            succesors: List of successor vertices
        """
        # Implementation using GCS graph structure
        # NOTE: Needs review and testing

        successors : Set[GraphOfConvexSets.Vertex] = set()

        for path in self.vertex_paths[vertex]:
            for idx, vtex in enumerate(path.vertices):
                if vtex == vertex:
                    # add all to successors
                    successors.update(path.vertices[idx+1:])
                    
        return successors
                


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
