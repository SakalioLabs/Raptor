"""RaptorSystem: Central orchestrator integrating all 25 modules.

Provides a unified interface for flight control, vision, communication,
mission planning, swarm coordination, decision-making, and optimization.
Designed for RK3566-based UAV platforms with NPU acceleration.
"""
from dataclasses import dataclass
import numpy as np

from raptor.flight.pid_controller import PIDController, PIDGains
from raptor.flight.state_machine import FlightStateMachine, FlightMode
from raptor.flight.attitude import Quaternion
from raptor.flight.motor_mixer import HexacopterMixer
from raptor.control.mpc_controller import MPCController, MPCConfig, MPCState
from raptor.comm.mavlink_handler import MAVLinkHandler
from raptor.comm.link_manager import LinkManager, LinkType, LinkQuality
from raptor.comm.network_monitor import NetworkMonitor
from raptor.mission.waypoint import WaypointNavigator, Waypoint, Position
from raptor.mission.path_planner import AStarPlanner, GridMap, GridCell
from raptor.mission.geofence import Geofence, GeofenceZone, GeofencePoint
from raptor.vision.obstacle_detector import ObstacleDetector, BoundingBox
from raptor.vision.landing_detector import LandingDetector, MarkerDetection
from raptor.vision.depth_estimator import DepthEstimator
from raptor.ai.edge_detector import AnchorFreeHead, NMS, Detection
from raptor.slam.visual_inertial_odometry import EKFVIO, IMUMeasurement, VisualMeasurement
from raptor.planning.cooperative_planner import CooperativePlanner, Agent, Point
from raptor.platform.task_offloader import TaskOffloader, TaskProfile
from raptor.platform.swarm_coordinator import SwarmCoordinator, DroneState
from raptor.swarm.evolutionary_game import TaskAllocationEGT, ReplicatorDynamics, PayoffMatrix
from raptor.swarm.network_centric import NetworkCentricManager, UAVNode, MissionTask, Role
from raptor.decision.marl_qmix import QMixTrainer, QMixAgent, QMixNetwork
from raptor.decision.consensus import SwarmConsensus, NodeState
from raptor.decision.intent_predictor import IntentPredictor, BehaviorObservation
from raptor.optimization.energy_optimizer import EnergyOptimizer, BatteryState


@dataclass
class SystemState:
    armed: bool = False
    flight_mode: str = "DISARMED"
    battery_pct: float = 100.0
    altitude: float = 0.0
    position: tuple = (0.0, 0.0, 0.0)
    velocity: tuple = (0.0, 0.0, 0.0)
    cpu_load: float = 0.0
    npu_load: float = 0.0
    link_quality: float = 1.0


@dataclass
class SystemConfig:
    uav_id: str = "raptor-01"
    swarm_enabled: bool = True
    autonomous_decision: bool = True
    optimization_enabled: bool = True
    max_altitude: float = 120.0
    acceptance_radius: float = 2.0
    home_position: tuple = (0.0, 0.0, 0.0)


class RaptorSystem:
    def __init__(self, config: SystemConfig = None):
        self.config = config or SystemConfig()
        self.state = SystemState()

        self.pid_roll = PIDController(PIDGains(kp=1.0, ki=0.1, kd=0.05))
        self.pid_pitch = PIDController(PIDGains(kp=1.0, ki=0.1, kd=0.05))
        self.pid_yaw = PIDController(PIDGains(kp=0.8, ki=0.05, kd=0.02))
        self.pid_alt = PIDController(PIDGains(kp=0.5, ki=0.05, kd=0.05))

        self.mixer = HexacopterMixer()
        self.fsm = FlightStateMachine()
        self.mpc = MPCController()
        self.mavlink = MAVLinkHandler()
        self.link_mgr = LinkManager()
        self.network_mon = NetworkMonitor()
        self.navigator = WaypointNavigator(acceptance_radius=self.config.acceptance_radius)
        self.path_planner = AStarPlanner()
        self.grid_map = GridMap(50, 50)
        self.geofence = Geofence()
        self.obstacle_detector = ObstacleDetector()
        self.landing_detector = LandingDetector()
        self.depth_estimator = DepthEstimator()
        self.edge_detector = AnchorFreeHead()
        self.vio = EKFVIO()
        self.task_offloader = TaskOffloader()
        self.swarm_coord = SwarmCoordinator()
        self.energy_optimizer = EnergyOptimizer()

        self.ncm = NetworkCentricManager()
        self.node = UAVNode(uav_id=config.uav_id if config else "raptor-01")
        self.egt = TaskAllocationEGT(n_tasks=4, n_agents=1)
        self.qmix = QMixTrainer(n_agents=1, n_actions=4, state_dim=8)
        self.consensus = SwarmConsensus()
        self.intent_predictor = IntentPredictor(["patrol", "attack", "retreat", "support"])

    def initialize(self):
        self.consensus.add_node(self.config.uav_id)
        self.ncm.register_uav(self.node)
        self.state.armed = True
        self.fsm.transition(FlightMode.STANDBY)
        return True

    def arm(self):
        if self.state.battery_pct > 0.15:
            self.state.armed = True
            self.fsm.transition(FlightMode.STABILIZE)
            return True
        return False

    def disarm(self):
        self.state.armed = False
        self.fsm.transition(FlightMode.DISARMED)
        return True

    def set_mission(self, waypoints):
        self.navigator.set_waypoints(waypoints)

    def navigate(self):
        pos = Position(*self.state.position)
        return self.navigator.navigate(pos)

    def compute_avoidance(self, detections, altitude):
        return self.obstacle_detector.compute_avoidance(detections, altitude)

    def get_status(self):
        return (
            f"Raptor {self.config.uav_id}: mode={self.fsm.mode.name} "
            f"pos=({self.state.position[0]:.1f},{self.state.position[1]:.1f}) "
            f"alt={self.state.altitude:.1f}m bat={self.state.battery_pct:.0f}% "
            f"link={self.state.link_quality:.2f}"
        )

    def get_swarm_state(self):
        return {
            "mode": self.fsm.mode.name,
            "position": self.state.position,
            "battery": self.state.battery_pct,
            "armed": self.state.armed,
            "link_quality": self.state.link_quality,
        }

    def run_pid(self, setpoint, measurement, dt):
        return self.pid_roll.update(setpoint - measurement, dt)

    def update_state(self, pos=None, alt=None, bat=None):
        if pos: self.state.position = pos
        if alt is not None: self.state.altitude = alt
        if bat is not None: self.state.battery_pct = bat
