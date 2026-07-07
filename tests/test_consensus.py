import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))
from raptor.decision.consensus import SwarmConsensus, NodeState

def test_add_nodes():
    sc = SwarmConsensus()
    sc.add_node("n1")
    sc.add_node("n2")
    assert len(sc.nodes) == 2

def test_leader_election():
    sc = SwarmConsensus()
    sc.add_node("n1")
    sc.add_node("n2")
    sc.add_node("n3")
    result = sc.request_vote("n1")
    assert result["elected"] == True
    assert result["leader"] == "n1"
    assert sc.leader == "n1"

def test_heartbeat():
    sc = SwarmConsensus()
    sc.add_node("n1")
    sc.add_node("n2")
    sc.add_node("n3")
    sc.request_vote("n1")
    count = sc.heartbeat("n1")
    assert count == 2

def test_append_entry():
    sc = SwarmConsensus()
    sc.add_node("n1")
    sc.add_node("n2")
    sc.add_node("n3")
    sc.request_vote("n1")
    ok, idx = sc.append_entry("command_A")
    assert ok == True
    assert idx == 0
    committed = sc.get_committed_log("n1")
    assert len(committed) == 1
    assert committed[0].command == "command_A"

def test_no_leader_no_append():
    sc = SwarmConsensus()
    sc.add_node("n1")
    ok, idx = sc.append_entry("cmd")
    assert ok == False
