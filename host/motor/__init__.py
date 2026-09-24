"""A PMSM: its constants and model, the motors and loads known here, their identification.

    from motor import Motor, Parameters
    from motor.catalog import BENCH_MOTOR, PLATINUM_5230SL
    from motor.loads import APC20x10E

Nothing here imports a board family or `machine`.
"""
from motor.pmsm import Motor, Parameters

__all__ = ['Motor', 'Parameters']
