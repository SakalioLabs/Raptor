import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.swarm.network_centric import NetworkCentricManager, UAVNode, MissionTask, Role, TaskStatus

def test_register_uav():
    mgr = NetworkCentricManager()
    mgr.register_uav(UAVNode("u1", Role.SCOUT, (0, 0, 0), 1.0))
    assert "u1" in mgr.uavs

def test_connection():
    mgr = NetworkCentricManager()
    mgr.register_uav(UAVNode("u1", position=(0,0,0)))
    mgr.register_uav(UAVNode("u2", position=(5,0,0)))
    mgr.add_connection("u1", "u2")
    assert "u2" in mgr.uavs["u1"].connected_peers

def test_situational_awareness():
    mgr = NetworkCentricManager()
    mgr.register_uav(UAVNode("u1", position=(0,0,0)))
    mgr.register_uav(UAVNode("u2", position=(10,0,0)))
    mgr.add_connection("u1", "u2")
    sa = mgr.get_situational_awareness("u1")
    assert "u2" in sa["peers"]

def test_task_assignment():
    mgr = NetworkCentricManager()
    u1 = UAVNode("u1", Role.STRIKER, (0,0,0), 1.0)
    mgr.register_uav(u1)
    task = MissionTask("t1", (5,0,0), priority=1.0, required_role=Role.STRIKER)
    mgr.submit_task(task)
    assignments = mgr.assign_tasks()
    assert "t1" in assignments
    assert assignments["t1"] == "u1"
    assert task.status == TaskStatus.ASSIGNED

def test_role_update():
    mgr = NetworkCentricManager()
    u1 = UAVNode("u1", Role.STRIKER, (0,0,0), 0.1)
    mgr.register_uav(u1)
    mgr.update_roles()
    assert u1.role == Role.RELAY
