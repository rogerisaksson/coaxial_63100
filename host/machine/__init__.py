"""Machines over IO nodes: loops of float channels, programs a model writes, any board family.

    from machine import Machine
    humanoid = Machine.discover('humanoid', port='COM4')   # every board found, the type over them
    print(humanoid.prompt())                               # what a model is told
    humanoid.run(program)                                  # checked, armed, run, disarmed

A board family (the Coaxial63100's: `coaxial.node`) is loaded when discovering, and only
if its package is here; nothing in this package imports one.
"""
from machine.errors import MachineError
from machine.machine import Machine
from machine.nodes import Nodes

__all__ = ['Machine', 'MachineError', 'Nodes']
