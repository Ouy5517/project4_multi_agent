"""Common BT leaf-node package, decoupled from specific playbooks.

Split into three responsibility-based modules:

data layer that writes blackboard state
condition leaves that only read the blackboard
action leaves that write commands

PLAY-stage role conditions and kicking action leaves live in :mod:`src.play.nodes`.
"""
