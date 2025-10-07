"""
graph.py: Graph data structures and algorithms for pytms
"""

from dataclasses import dataclass
from typing import Literal, List, Dict, Tuple, Optional

NodeID = str
EdgeID = str
PathID = str


@dataclass
class Coordinate:
    x: float
    y: float


@dataclass
class GraphEdge:
    edgeID: EdgeID
    u: NodeID
    v: NodeID
    length: float


@dataclass
class GraphNode:
    loc: Coordinate
    nodeID: NodeID
    type: Literal["main", "station", "side"]


@dataclass
class GraphData:
    nodes: List[GraphNode]
    edges: List[GraphEdge]


@dataclass
class GraphPosition:
    edge: GraphEdge
    distanceAlongEdge: float


@dataclass
class PathInfo:
    pathID: PathID
    route: List[NodeID]
    length: float


@dataclass
class SegmentSection:
    edge: GraphEdge
    start: float
    end: float

    def get_length(self) -> float:
        return self.end - self.start

    def __len__(self) -> int:
        return int(self.get_length())

    def clone(self) -> "SegmentSection":
        return SegmentSection(self.edge, self.start, self.end)


DistanceTable = Dict[NodeID, Dict[NodeID, float]]
NextNodeTable = Dict[NodeID, Dict[NodeID, Optional[NodeID]]]
FloydWarshallResult = Tuple[DistanceTable, NextNodeTable]


def floyd_warshall(
    nodes: List[GraphNode], edges: List[GraphEdge]
) -> FloydWarshallResult:
    """Compute shortest paths between all pairs of nodes using the Floyd-Warshall algorithm."""
    node_ids: List[NodeID] = [node.nodeID for node in nodes]
    distances: DistanceTable = {
        i: {j: float("inf") for j in node_ids} for i in node_ids
    }
    next_node: NextNodeTable = {i: {j: None for j in node_ids} for i in node_ids}
    # Distance from a node to itself is zero
    for i in node_ids:
        distances[i][i] = 0
    # Initialize distances and next_node for each edge
    for e in edges:
        distances[e.u][e.v] = e.length
        next_node[e.u][e.v] = e.v
    # Floyd-Warshall algorithm
    for k in node_ids:
        for i in node_ids:
            for j in node_ids:
                if distances[i][j] > distances[i][k] + distances[k][j]:
                    distances[i][j] = distances[i][k] + distances[k][j]
                    next_node[i][j] = next_node[i][k]
    return distances, next_node


def get_edge_key(u: NodeID, v: NodeID) -> EdgeID:
    """Return a unique EdgeID for an edge from node u to node v."""
    return f"{u}->{v}"


def get_path_key(start: NodeID, end: NodeID) -> str:
    """Return a unique key for a path from start node to end node."""
    return f"{start}->{end}"


class Graph:
    def __init__(self, graph_data: GraphData | None = None) -> None:
        self.edges: List[GraphEdge] = []
        self.nodes: List[GraphNode] = []
        self._node_map: Dict[NodeID, GraphNode] = {}
        self._edge_map: Dict[EdgeID, GraphEdge] = {}
        self._distances: DistanceTable = {}
        self._next_node: NextNodeTable = {}
        self._shortest_path_cache: Dict[PathID, PathInfo] = {}

        if graph_data and (graph_data.edges or graph_data.nodes):
            self.add_graph_data(graph_data)

    def _node_exists(self, nodeID: NodeID) -> bool:
        """Check if a node with the given id exists in the graph"""
        return any(node.nodeID == nodeID for node in self.nodes)

    def _edge_exists(self, edge_id: EdgeID) -> bool:
        """Check if an edge with the given id exists in the graph"""
        return edge_id in self._edge_map

    def _rebuild_maps(self) -> None:
        """Rebuild the node and edge maps from the current lists."""
        self._node_map = {node.nodeID: node for node in self.nodes}
        self._edge_map = {edge.edgeID: edge for edge in self.edges}

    def add_node(self, node: GraphNode) -> bool:
        """Add a node to the graph. Returns True if added, False if nodeID already exists."""
        if self._node_exists(node.nodeID):
            return False
        self.nodes.append(node)
        return True

    def add_edge(self, edge: GraphEdge) -> bool:
        """Add an edge to the graph. Returns True if added, False if edgeID already exists or nodes do not exist."""
        if (
            (not self._edge_exists(edge.edgeID))
            and self._node_exists(edge.u)
            and self._node_exists(edge.v)
        ):
            self.edges.append(edge)
            self._edge_map[edge.edgeID] = edge
            return True
        return False

    def add_node_list(self, node_list: List[GraphNode]) -> bool:
        """Add multiple nodes to the graph. Returns True if all added, False if any nodeID already exists."""
        completionChecker: bool = True
        for node in node_list:
            if self.add_node(node) == False:
                completionChecker = False
        return completionChecker

    def add_edge_list(self, edge_list: List[GraphEdge]) -> bool:
        """Add multiple edges to the graph. Returns True if all added, False if any edgeID already exists or nodes do not exist."""
        completionChecker: bool = True
        for edge in edge_list:
            if self.add_edge(edge) == False:
                completionChecker = False
        return completionChecker

    def add_graph_data(self, graph_data: GraphData) -> bool:
        """Add multiple nodes and edges from GraphData to the graph. Returns True if all added, False if any fail."""
        node_completion: bool = self.add_node_list(graph_data.nodes)
        edge_completion: bool = self.add_edge_list(graph_data.edges)
        return node_completion and edge_completion

    def _compute_shortest_paths(self) -> None:
        """Compute shortest paths between all pairs of nodes using the Floyd-Warshall algorithm."""
        self._distance, self._next_node = floyd_warshall(self.nodes, self.edges)
        self._shortest_path_cache.clear()

    def _reconstruct_path(self, u: NodeID, v: NodeID) -> List[NodeID]:
        """Reconstruct the shortest path from node u to node v using the next_node table."""
        if not self._next_node:
            raise ValueError(
                "Shortest paths not computed. Call compute_shortest_paths() first."
            )
        route: List[NodeID] = [u]
        while u != v:
            next_u = self._next_node[u][v]
            if next_u is None:
                # No path exists
                return []
            u = next_u
            route.append(u)
        return route

    def get_shortest_path(self, start: NodeID, end: NodeID) -> PathInfo:
        """Get the shortest path from start node to end node. Computes paths if not already done."""
        pathID: PathID = get_path_key(start, end)
        if pathID in self._shortest_path_cache:
            return self._shortest_path_cache[pathID]
        if not self._distances or not self._next_node:
            self._compute_shortest_paths()
        if self._distances[start][end] == float("inf"):
            raise ValueError(f"No path from {start} to {end}")
        else:
            route = self._reconstruct_path(start, end)
            entry = PathInfo(
                pathID=pathID, route=route, length=self._distances[start][end]
            )
        self._shortest_path_cache[pathID] = entry
        return entry

    def get_edge(self, u: NodeID, v: NodeID) -> GraphEdge:
        edge_id: EdgeID = get_edge_key(u, v)
        if edge_id not in self._edge_map:
            self._rebuild_maps()
            if edge_id not in self._edge_map:
                raise ValueError(f"No edge from {u} to {v}")
        return self._edge_map[edge_id]

    def get_next_edge(self, u: NodeID, v: NodeID) -> GraphEdge:
        """Get the next edge on the shortest path from node u to node v."""
        pathInfo: PathInfo = self.get_shortest_path(u, v)
        if not pathInfo.route or len(pathInfo.route) < 2:
            raise ValueError(f"No path from {u} to {v}")
        return self.get_edge(u, pathInfo.route[1])

    def get_distance_to_node(
        self, currentPosition: GraphPosition, targetNode: NodeID
    ) -> float:
        pathInfo = self.get_shortest_path(currentPosition.edge.v, targetNode)
        return pathInfo.length - currentPosition.distanceAlongEdge
