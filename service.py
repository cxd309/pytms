from dataclasses import dataclass
from typing import Literal, List
from graph import NodeID, GraphPosition

ServiceID = str
RouteID = str

ServiceState = Literal[
    "stationary", "dwelling", "accelerating", "decelerating", "cruising"
]


@dataclass
class RouteStop:
    node_id: NodeID
    t_dwell: float


Route = List[RouteStop]


@dataclass
class Vehicle:
    name: str
    a_acc: float
    a_dcc: float
    v_max: float


@dataclass
class Service:
    service_id: ServiceID
    initial_position: NodeID
    route: Route
    vehicle: Vehicle
    state: ServiceState


class SimulationService:
    def __init__(self, currentPosition: GraphPosition):
        pass
