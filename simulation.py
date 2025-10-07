from dataclasses import dataclass
from typing import Literal, List
from graph import NodeID, GraphPosition, Graph, SegmentSection

ServiceID = str


@dataclass
class RouteStop:
    nodeID: NodeID
    t_dwell: float


@dataclass
class VehicleClass:
    name: str
    a_acc: float
    a_dcc: float
    v_max: float


@dataclass
class TransportService:
    serviceID: ServiceID
    startNodeID: NodeID
    stops: List[RouteStop]
    vehicle: VehicleClass


SimulationState = Literal[
    "stationary", "dwelling", "accelerating", "decelerating", "cruising"
]


@dataclass
class SimulationServiceLog:
    serviceID: ServiceID
    vehicleClassName: str
    currentPosition: GraphPosition
    nextStop: NodeID
    state: SimulationState
    velocity: float
    remainingDwell: float


def find_stop_index(nodeID: NodeID, stops: List[RouteStop]) -> int:
    for i, stop in enumerate(stops):
        if stop.nodeID == nodeID:
            return i
    return 0


def find_next_stop(current_node: NodeID, stops: List[RouteStop]) -> RouteStop:
    curStopIndex = find_stop_index(current_node, stops)
    nextStopIndex = (curStopIndex + 1) % len(stops)
    return stops[nextStopIndex]


def get_segments_along_path(
    graph: Graph,
    start_position: GraphPosition,
    stops: list[RouteStop],
    next_stop: str,
    s_total: float,
) -> list[SegmentSection]:
    """
    Returns a list of SegmentSection objects representing the path along the graph,
    starting from start_position, passing through stops, towards next_stop, for a total distance s_total.
    """
    current_position: GraphPosition = GraphPosition(
        edge=start_position.edge, distanceAlongEdge=start_position.distanceAlongEdge
    )
    segments: list[SegmentSection] = []
    s_remaining: float = s_total

    while s_remaining > 0:
        edge_distance_remaining = (
            current_position.edge.length - current_position.distanceAlongEdge
        )

        segment_length = min(s_remaining, edge_distance_remaining)
        segment_start = current_position.distanceAlongEdge
        segment_end = segment_start + segment_length
        segments.append(
            SegmentSection(current_position.edge, segment_start, segment_end)
        )

        if s_remaining >= edge_distance_remaining:
            # Find the next edge to move onto
            if current_position.edge.v == next_stop:
                next_stop = find_next_stop(current_position.edge.v, stops).nodeID
            next_edge = graph.get_next_edge(current_position.edge.v, next_stop)
            if not next_edge:
                print("Cannot find next edge")
                break
            else:
                current_position = GraphPosition(edge=next_edge, distanceAlongEdge=0)
                s_remaining -= edge_distance_remaining
                if s_remaining == 0:
                    segments.append(SegmentSection(current_position.edge, 0, 0))
        else:
            current_position.distanceAlongEdge += s_remaining
            s_remaining = 0

    return segments


def get_forward_position(
    graph: Graph,
    current_position: GraphPosition,
    stops: list[RouteStop],
    next_stop: str,
    s_total: float,
) -> GraphPosition:
    """
    Returns the GraphPosition reached after moving s_total along the path from current_position.
    """
    segments = get_segments_along_path(
        graph, current_position, stops, next_stop, s_total
    )
    if not segments:
        return current_position
    last_segment = segments[-1]
    return GraphPosition(edge=last_segment.edge, distanceAlongEdge=last_segment.end)


def project_distance_travelled(
    v_current: float, a_acc: float, a_dcc: float, timeStep: float
) -> float:
    pass


def get_segment_conflict(a: SegmentSection, b: SegmentSection) -> float:
    pass


class TransportMicroSimulator:
    def __init__(self):
        pass


class SimulationService:
    def __init__(self):
        pass
