from Cython.Build.Dependencies import time
import graphviz
import numpy as np
import copy

from dataclasses import dataclass
from queue import PriorityQueue
from typing import Callable, Dict, List, Optional, Set, Tuple, cast

from pydrake.geometry.optimization import ( # type: ignore
    CartesianProduct,
    GraphOfConvexSets,
    GraphOfConvexSetsOptions,
    HPolyhedron,
    ImplicitGraphOfConvexSets,
    Point
)

from absl import logging


@dataclass
class SearchPath:
    vertices: Tuple[GraphOfConvexSets.Vertex, ...]
    f_value: float

    def __lt__(self, other):
        return self.f_value < other.f_value


class GCSStar(ImplicitGraphOfConvexSets):
    def __init__(self, regions: List[HPolyhedron]):
        super().__init__()

        self.regions = regions

        for region in regions:
            self.mutable_gcs().AddVertex(region) 

        self._S: Dict[
            GraphOfConvexSets.Vertex, Set[Tuple[GraphOfConvexSets.Vertex, ...]]
        ] = {}

        self._Q: PriorityQueue[SearchPath] = PriorityQueue()

    def SolveShortestPath(
        self,
        start: GraphOfConvexSets.Vertex,
        target: GraphOfConvexSets.Vertex,
        f_estimator: Callable[[Tuple[GraphOfConvexSets.Vertex, ...]], float],
    ) -> Optional[List[GraphOfConvexSets.Vertex]]:

        # init with start vertex
        initial_path = SearchPath((start,), f_estimator((start,)))

        self._Q.put(initial_path)
        self._S[start] = {(start,)}

        # while loop
        while not self._Q.empty():
            # v = q.pop
            current_path = self._Q.get()
            current = current_path.vertices[-1]

            # if v_end == target return v
            if current == target:
                return list(current_path.vertices)

            # forall v' in successors(v_end)
            for edge in self.Successors(current):
                # do
                # v' = [v,v']
                successor = edge.v()  # get the target vertex of each edge
                new_vertices = current_path.vertices + (successor,)
                new_path = SearchPath(new_vertices, f_estimator(new_vertices))

                # if not dominated (v, S[v])
                # dominance checking
                if self._not_dominated(new_path, successor):
                    if successor not in self._S:
                        # S[v'].add(v')
                        self._S[successor] = set()
                        # Q.add(v')
                        self._Q.put(new_path)

        # return failed to find path
        return None

    def _cost_to_come(
        self,
        path: Tuple[GraphOfConvexSets.Vertex, ...],
        sample_point: Optional[GraphOfConvexSets.Vertex],
    ) -> float:
        """Evaluate cost-to-come to reach specific point via path."""
        # Create temporary GCS for evaluation
        temp_gcs = GraphOfConvexSets()

        try:
            # Add path regions
            regions = self.regions
            v_0 = temp_gcs.AddVertex(path[0].set())
            v_prev = v_0 # NOTE: Could be issue
            for i in range(1, len(path) - 1):
                v_i = temp_gcs.AddVertex(path[i].set())
                temp_gcs.AddEdge(v_prev, v_i)
                v_prev = v_i

            # Create singleton point set as target
            v_sample = temp_gcs.AddVertex(sample_point)

            # Connect subgraph to target point
            temp_gcs.AddEdge(v_prev, v_sample)

            # Add costs
            for e in temp_gcs.Edges():
                #cost = np.linalg.norm(e.xv() - e.xu())
                e.AddCost(1)


            # Solve
            options = GraphOfConvexSetsOptions()
            options.convex_relaxation = False
            result = temp_gcs.SolveShortestPath(v_0, v_sample, options)

            breakpoint()
            return result.get_optimal_cost() if result.is_success() else float('inf')

        except RuntimeError:
            return float('inf')

    def _find_edge(self, u: GraphOfConvexSets.Vertex, v: GraphOfConvexSets.Vertex):
        edges = self.mutable_gcs().Edges()

        for edge in edges:
            if edge.u() == u and edge.v() == v:
                return edge

        return None

    def _is_point_reachable(
        self,
        path: Tuple[GraphOfConvexSets.Vertex, ...],
        sample_point: Optional[GraphOfConvexSets.Vertex],
    ) -> bool:

        if sample_point is None:
            raise ValueError("sample_point must not be none")

        # path to series of edges
        edges = [self._find_edge(path[i], path[i + 1]) for i in range(len(path) - 1)]
        if None in edges:
            return False
        edges = cast(List[GraphOfConvexSets.Edge], edges)  # for type checker

        # endpoint constraint
        last_vertex = path[-1]
        point = np.asarray(sample_point.x(), dtype=np.float64).reshape(-1, 1)

        if not last_vertex.set().PointInSet(point):
            return False  # NOTE: last vertex not in set of points

        return self.mutable_gcs().SolveConvexRestriction(edges).is_success()

    def _sample_point(
        self, vertex: GraphOfConvexSets.Vertex
    ) -> Optional[GraphOfConvexSets.Vertex]:
        """Sample a point from the vertex's convex set, uniformly"""
        cvx_set = vertex.set()
        if cvx_set.IsEmpty():
            return None

        # sample a point from the set
        feasible_point = cvx_set.MaybeGetFeasiblePoint()
        if feasible_point is None:
            raise ValueError("Could not sample a feasible point from the convex set")

        # HACK: naive sampling implementation, depends on convex set we create
        random_direction = np.random.randn(len(feasible_point))
        random_direction /= np.linalg.norm(random_direction)  # normalize step
        scale = 0.1

        while not cvx_set.PointInSet(feasible_point + random_direction * scale):
            random_direction = np.random.randn(len(feasible_point))
            random_direction /= np.linalg.norm(random_direction)
            scale *= 0.7  # NOTE: hparam here
        return Point(feasible_point + random_direction * scale)

    def _reaches_cheaper(self, path: SearchPath) -> bool:
        """Returns True if the path reaches any point cheaper than all other paths, with sampling"""
        vertex = path.vertices[-1]
        if vertex not in self._S:
            return True  # if we reach a new point its inherently cheaper?

        # sample a point randomly from the convex set of the vertex
        sampled_point = self._sample_point(vertex)

        # get cost to come for this vertex
        current_cost = self._cost_to_come(path.vertices, sampled_point)

        # compare against existing paths
        for existing_path in self._S[vertex]:
            existing_cost = self._cost_to_come(existing_path, sampled_point)
            if current_cost < existing_cost:
                return True
        return False

    def _reaches_new(self, path: SearchPath) -> bool:
        """Returns True if the path reaches any new point than all other paths, with sampling"""
        vertex = path.vertices[-1]
        if vertex not in self._S:
            return True

        # sample a point randomly from the convex set of the vertex
        sampled_point = self._sample_point(vertex)

        # check if current path can reach this
        current_reachable = self._is_point_reachable(path.vertices, sampled_point)
        if not current_reachable:
            return False

        # compare to existing paths if any can reach this point
        for existing_path in self._S[vertex]:
            existing_reachable = self._is_point_reachable(
                existing_path, sampled_point
            )  # can we reach this with an existing path

            if existing_reachable:
                return False

        return True

    def _not_dominated(
        self, path: SearchPath, vertex: GraphOfConvexSets.Vertex
    ) -> bool:
        return self._reaches_cheaper(path) or self._reaches_new(path)

    def ExpandRecursively(
        self, start: GraphOfConvexSets.Vertex, max_successor_calls: int = 10
    ) -> None:
        """Call successors and get"""

        edges = self.Successors(start)

        if max_successor_calls == 1:
            return

        for edge in edges:
            self.ExpandRecursively(edge.v(), max_successor_calls-1)

        return


    def Successors(self, v: GraphOfConvexSets.Vertex) -> List[GraphOfConvexSets.Edge]:
        """Get successors from each vertex"""
        # take steps at each corner
        curr_set = v.set()
        # curr_set = cast(HPolyhedron, curr_set)
        if isinstance(curr_set, CartesianProduct):
            for i in range(curr_set.num_factors()):
                cart_set = curr_set.factor(i)
                if len(cart_set.ChebyshevCenter()) == 2:
                    curr_set = cart_set
                    break
                else:
                    continue

        curr_set = cast(HPolyhedron, curr_set)
        # get intersections
        intersections = []
        intersections_vertices = []
        for other_vertex in self.gcs().Vertices():
            other_set = other_vertex.set()
            other_set = cast(HPolyhedron, other_set)

            if curr_set is other_set:
                continue # skip self
            intersection = curr_set.Intersection(other_set)
            if not intersection.IsEmpty():
                intersections.append(intersection)
                intersections_vertices.append(other_vertex)
                logging.debug(f"Intersection between {curr_set} and {other_set} = {intersection}")

        logging.debug(f"n_intersections = {len(intersections)}, tot:{len(self.regions)}\n")



        #breakpoint()

        # add edges
        edges = [
            self.mutable_gcs().AddEdge(v, other_vertex)
            for other_vertex in intersections_vertices
        ]

        graph_string =self.gcs().GetGraphvizString()
        
        graph = graphviz.Source(graph_string)

        # Save as PNG
        graph.render(str(int(time.time())), format="png", cleanup=True)

        return edges
