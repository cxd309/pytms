from dataclasses import dataclass
from typing import Literal, List
from graph import NodeID, GraphPosition

ServiceID = str
RouteID = str
Route = List["RouteStop"]
ServiceState = Literal[
    "stationary", "dwelling", "accelerating", "decelerating", "cruising"
]


@dataclass
class RouteStop:
    node_id: NodeID
    t_dwell: float


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


class SimulationService(Service):
    def __init__(self, service: Service, initial_position: GraphPosition):
        super().__init__(**service.__dict__)
        self.current_position: GraphPosition = initial_position
        self.state: ServiceState = "stationary"
        self.velocity: float = 0.0
        self.remaining_dwell: float = 0.0
        self.next_stop: NodeID = self.route[0].node_id
        self._next_stop_index: int = 0

        if self.initial_position == self.next_stop:
            self._advance_next_stop()

    def _advance_next_stop(self) -> None:
        new_index: int = (self._next_stop_index + 1) % len(self.route)
        self._next_stop_index = new_index
        self.next_stop = self.route[new_index].node_id

    def advance_dwell(self, t: float) -> None:
        if self.state != "dwelling":
            self._start_dwell()

        self.remaining_dwell -= t

        if self.remaining_dwell <= 0.0:
            self._end_dwell()

    def _start_dwell(self) -> None:
        self.state = "dwelling"
        self.velocity = 0.0
        self.remaining_dwell = self.route[self._next_stop_index].t_dwell
        self._advance_next_stop()

    def _end_dwell(self) -> None:
        self.state = "accelerating"
        self.velocity = 0.0
        self.remaining_dwell = 0.0

    def update_position(self, new_position: GraphPosition) -> None:
        self.current_position = new_position

    def get_braking_distance(self) -> float:
        return (self.velocity**2) / (2 * self.vehicle.a_dcc)

    def get_distance_travelled_in_time(self, t: float, state: ServiceState) -> float:
        a: float
        match state:
            case "accelerating":
                a = self.vehicle.a_acc
            case "decelerating":
                a = self.vehicle.a_dcc
            case "cruising":
                a = 0.0
            case "dwelling" | "stationary":
                return 0.0
            case _:
                a = 0.0
        return (self.velocity * t) + (0.5 * a * (t**2))
