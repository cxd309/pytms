from dataclasses import dataclass
from typing import List, Annotated
from graph import GraphData, Graph, GraphPosition
from service import Service, SimulationService, get_service_first_stop

SimulationID = str


@dataclass
class SimulationMeta:
    simulation_id: SimulationID
    run_time: Annotated[float, "seconds"]
    time_step: Annotated[float, "seconds"]


@dataclass
class SimulationInput:
    simulation_meta: SimulationMeta
    graph_data: GraphData
    service_list: List[Service]


class TMS(SimulationMeta):
    def __init__(self, simulation_input: SimulationInput):
        super().__init__(**simulation_input.simulation_meta.__dict__)
        self.graph: Graph = Graph(simulation_input.graph_data)
        self.service_list: List[SimulationService] = []
        for service in simulation_input.service_list:
            initial_position: GraphPosition = self.graph.get_path_start_position(
                service.initial_position, get_service_first_stop(service)[0]
            )
            sim_service = SimulationService(service, initial_position)
            self.service_list.append(sim_service)
