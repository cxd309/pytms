from dataclasses import dataclass
from typing import List, Annotated, Dict
from graph import GraphData, Graph, GraphPosition, Segment
from service import (
    Service,
    ServiceID,
    SimulationService,
    ServiceLog,
    get_service_first_stop,
)

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


@dataclass
class SimulationLogRow:
    timestamp: Annotated[float, "seconds"]
    service_log: ServiceLog


@dataclass
class SimulationLog:
    simulation_meta: SimulationMeta
    output: List[SimulationLogRow]


MovementAuthority = List[Segment]

MovementAuthorityRecord = Dict[ServiceID, MovementAuthority]


class TMS(SimulationMeta):
    def __init__(self, simulation_input: SimulationInput):
        super().__init__(**simulation_input.simulation_meta.__dict__)
        self.graph: Graph = Graph(simulation_input.graph_data)
        self.service_list: List[SimulationService] = []
        self.ma_record: MovementAuthorityRecord = {}
        for service in simulation_input.service_list:
            initial_position: GraphPosition = self.graph.get_path_start_position(
                service.initial_position, get_service_first_stop(service)[0]
            )
            sim_service = SimulationService(service, initial_position)
            self.service_list.append(sim_service)

        self.cur_time: Annotated[float, "seconds"] = 0

        self.log: SimulationLog = SimulationLog(simulation_input.simulation_meta, [])

    def run(self):
        print(f"running simulation {self.simulation_id}")
        self.cur_time = 0
        while self.cur_time <= self.run_time:
            self.step()
        print(f"completed simulation {self.simulation_id}")

    def step(self):
        # Future consider safe seperation but just movement right now
        for service_index in range(len(self.service_list)):
            proposed_motion = self.propose_service_motion(service_index)
            granted_motion 
            self.advance_service(service_index)
        self.cur_time += self.time_step

    def propose_service_motion(self, service_index)
