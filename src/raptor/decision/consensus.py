"""Distributed Consensus for UAV Swarm Decision-Making.

Implements a Raft-inspired consensus protocol adapted for UAV swarms
with network partitions and dynamic membership. Supports leader election,
log replication, and view-change for fault tolerance.

Based on: Ongaro & Ousterhout, "In Search of an Understandable
Consensus Algorithm" (2014), adapted for UAV swarm contexts.
"""
from dataclasses import dataclass, field
from enum import Enum, auto
import random
import time


class NodeState(Enum):
    FOLLOWER = auto()
    CANDIDATE = auto()
    LEADER = auto()


@dataclass
class LogEntry:
    term: int
    index: int
    command: str
    committed: bool = False


@dataclass
class ConsensusNode:
    node_id: str
    state: NodeState = NodeState.FOLLOWER
    current_term: int = 0
    voted_for: str = None
    log: list = field(default_factory=list)
    commit_index: int = -1
    leader_id: str = None
    election_timeout: float = 0.0
    last_heartbeat: float = 0.0


class SwarmConsensus:
    def __init__(self, election_timeout_range: tuple = (1.5, 3.0)):
        self.nodes: dict[str, ConsensusNode] = {}
        self.election_timeout_range = election_timeout_range
        self._votes: dict[str, set] = {}

    def add_node(self, node_id: str):
        self.nodes[node_id] = ConsensusNode(
            node_id=node_id,
            election_timeout=random.uniform(*self.election_timeout_range),
            last_heartbeat=time.time(),
        )

    def remove_node(self, node_id: str):
        if node_id in self.nodes:
            del self.nodes[node_id]

    @property
    def leader(self) -> str:
        for node in self.nodes.values():
            if node.state == NodeState.LEADER:
                return node.node_id
        return None

    def append_entry(self, command: str) -> tuple:
        leader_id = self.leader
        if leader_id is None:
            return (False, None)
        leader = self.nodes[leader_id]
        entry = LogEntry(
            term=leader.current_term,
            index=len(leader.log),
            command=command,
        )
        leader.log.append(entry)
        replicas = 0
        for node_id, node in self.nodes.items():
            if node_id != leader_id and node.state == NodeState.FOLLOWER:
                node.log.append(LogEntry(
                    term=entry.term, index=entry.index,
                    command=entry.command,
                ))
                replicas += 1
        if replicas >= len(self.nodes) // 2:
            entry.committed = True
            leader.commit_index = entry.index
            for node in self.nodes.values():
                if node.state == NodeState.FOLLOWER:
                    node.commit_index = max(node.commit_index, entry.index)
                    for e in node.log:
                        if e.index <= node.commit_index:
                            e.committed = True
            return (True, entry.index)
        return (False, None)

    def request_vote(self, candidate_id: str) -> dict:
        if candidate_id not in self.nodes:
            return {}
        candidate = self.nodes[candidate_id]
        candidate.current_term += 1
        candidate.state = NodeState.CANDIDATE
        candidate.voted_for = candidate_id
        votes = {candidate_id}
        for node_id, node in self.nodes.items():
            if node_id == candidate_id:
                continue
            if (node.voted_for is None or node.voted_for == candidate_id):
                last_log_term = node.log[-1].term if node.log else 0
                last_log_idx = len(node.log) - 1
                cand_term = candidate.log[-1].term if candidate.log else 0
                cand_idx = len(candidate.log) - 1
                if (cand_term > last_log_term or
                    (cand_term == last_log_term and cand_idx >= last_log_idx)):
                    node.voted_for = candidate_id
                    node.current_term = candidate.current_term
                    votes.add(node_id)
        majority = len(self.nodes) // 2 + 1
        if len(votes) >= majority:
            candidate.state = NodeState.LEADER
            candidate.leader_id = candidate_id
            for node in self.nodes.values():
                if node.node_id != candidate_id:
                    node.leader_id = candidate_id
                    node.state = NodeState.FOLLOWER
                    node.last_heartbeat = time.time()
            return {"elected": True, "leader": candidate_id, "votes": len(votes)}
        candidate.state = NodeState.FOLLOWER
        return {"elected": False, "votes": len(votes)}

    def heartbeat(self, leader_id: str) -> int:
        if leader_id not in self.nodes:
            return 0
        leader = self.nodes[leader_id]
        if leader.state != NodeState.LEADER:
            return 0
        count = 0
        for node in self.nodes.values():
            if node.node_id != leader_id:
                node.last_heartbeat = time.time()
                node.leader_id = leader_id
                count += 1
        return count

    def check_leader_timeout(self) -> str:
        now = time.time()
        for node in self.nodes.values():
            if node.state == NodeState.FOLLOWER:
                if now - node.last_heartbeat > node.election_timeout:
                    result = self.request_vote(node.node_id)
                    if result.get("elected"):
                        return node.node_id
        return None

    def get_committed_log(self, node_id: str) -> list:
        if node_id not in self.nodes:
            return []
        node = self.nodes[node_id]
        return [e for e in node.log if e.committed]
