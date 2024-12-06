import numpy as np

from dataclasses import dataclass
from queue import PriorityQueue
from typing import Callable, Dict, List, Optional, Set, Tuple, cast

from pydrake.geometry.optimization import GraphOfConvexSets, ImplicitGraphOfConvexSets


@dataclass
class SearchPath:
    vertices: Tuple[GraphOfConvexSets.Vertex, ...]
    f_value: float

    def __lt__(self, other):
        return self.f_value < other.f_value


class GCSStar(ImplicitGraphOfConvexSets):
    def __init__(self):
        super().__init__()
        self._S: Dict[
            GraphOfConvexSets.Vertex, Set[Tuple[GraphOfConvexSets.Vertex, ...]]
        ] = {}
        self._Q: PriorityQueue[SearchPath] = PriorityQueue()

    def solve(
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
        raise NotImplementedError()

    def _find_edge(self, u: GraphOfConvexSets.Vertex, v: GraphOfConvexSets.Vertex):
        edges = self.gcs().Edges()

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

        return self.gcs().SolveConvexRestriction(edges).is_success()

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

        while not cvx_set.PointInSet(feasible_point + random_direction):
            random_direction = np.random.randn(len(feasible_point))
            random_direction /= np.linalg.norm(random_direction)
            scale *= 0.7  # NOTE: hparam here

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

    def Expand(self, v: GraphOfConvexSets.Vertex) -> List[GraphOfConvexSets.Vertex]:
        raise NotImplementedError("abc")

    def Successors(self, v: GraphOfConvexSets.Vertex) -> List[GraphOfConvexSets.Edge]:
        raise NotImplementedError("abc")