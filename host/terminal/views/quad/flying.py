"""A craft's flight run apart as her body is: stepped in its own process, played back by its page.

Its boards, its sky, its law and its card stepped STEP_S at a time (`machine.process.apart`), paced to the
wall and no further than its lead past what the page has shown; each step said as the page draws
it: the frame, its rotors, the flight's instruments. The page plays them back steady
(`terminal.views.playback`, `blend`): in real time or slower, never run ahead.

    flying = Flying(machine.aircraft.helicopter, terminal.views.helicopter.flight, 2)
    said = flying.first()                  # the states said before the first frame
    said = flying.said()                   # since the last; the page shows them and says so:
    flying.shown(t)
    flying.send(reset=True) | flying.send(wind=True)
    flying.close()
"""
import copy
import importlib
import math
import os
import sys
import time
import traceback
from contextlib import suppress

from coaxial import Coaxial63100
from coaxial.errors import RigError
from machine.process import apart, placed
from machine.aircraft import aerobatics, aerokin, flightpack, gimbal, grounds, weather
from machine.rig.modes import EMULATED, SIMULATED
from terminal.views.quad import flight as qflight

#: The flight's loop's least turn while it waits, s.
REST_S = 0.004

#: What a state carries of a rotor, of the flight and of its air: what the page draws.
ROTOR = ('w', 'angle', 'amps', 'fault', 'budget', 'board_c', 'cooling', 'ident')
FLIGHT = ('stage', 'apex', 'share', 'over', 'lap', 'row', 'cells', 'peak', 'controls', 'wreck',
          'crashes')
AIR = ('kind', 'kept', 'for', 'come', 'blows', 'wind', 'steady')

#: The most the page waits for the flight's first state, s: a spawned interpreter, MuJoCo and
#: the boards' libraries or Renode limbs loaded.
FIRST_S = 120.0


def arm_emulated(rig, clock=1, warm_s=100.0, room='bench', rotor_m=None):
    """Arm an emulated STM32H753 board (`emulator://`) for flight and pair it with a local C
    stand-in slot for 1 kHz inner substeps."""
    from coaxial.comm.hostclock import clock_of
    from coaxial.control.commission import Commissioning
    from coaxial.control.motion import started
    from coaxial.simulated.sto import PILOT_VOLTS
    from machine.aircraft import pull, quad
    from machine.rig.controller import Feedback
    from machine.rig.parts import Gain, Slew, SpeedPI
    rotor_m = rotor_m or pull.ROTOR
    board, drive = rig.board, rig.board.drive
    board.thermal.configure(application=qflight.APPLICATION, clock=clock, margin_floor=1.0)
    board.thermal.situation(room or 'bench')
    board.afe.on()
    rig.pilot(PILOT_VOLTS)
    clock_of(rig).sleep(qflight.STO_SETTLE_S)
    board.gate_drivers.clear()
    rig.gates.on()
    drive.configure(profile=qflight.PROFILE)
    drive.configure(drv_i_max=pull.I_WEP, drv_i_trip=qflight.I_TRIP)
    drive.model.configure(j=rotor_m.j, b=rotor_m.b, load=0.0, vdc=flightpack.open_volts(1.0))
    drive.configure(source='model')
    Commissioning(rig).gains()
    params = drive.params()
    pairs = max(1.0, params.get('motor_pole_pairs') or 1.0)
    kt = 1.5 * pairs * (params.get('motor_lambda') or 0.002)
    board.ctrl.load(Feedback(SpeedPI(qflight.SPEED_HZ, pull.I_WEP, kt, rotor_m.j, rotor_m.b, rotor_m.drag),
                             prefilter=Slew(pull.slew(pull.I_WEP, math.inf, rotor_m)), measure=Gain(1.0 / pairs)))
    board.ctrl.wire('omega_hat', 'iq_ref', qflight.LOOP_HZ)
    board.ctrl.off()
    board.ctrl.drop()
    started(drive, clock_of(rig).sleep, qflight.START_A, quad.IDLE_RAD_S * pairs, qflight.START_S)
    board.ctrl.rows([(0.0, quad.IDLE_RAD_S)])
    board.ctrl.on()
    rotor = qflight.arm(Coaxial63100(execution_mode=SIMULATED).open(), clock=clock, warm_s=warm_s,
                        room=room, rotor_m=rotor_m)
    rotor.update(emu=rig, origin=rig.origin)
    return rotor


class Flight:

    """A craft's flight as `apart.paced` steps it: its time `t`, its `epoch` - begun again each
    reset -, a command applied, a step of STEP_S, its state `said`."""

    def __init__(self, craft, flown, count, port=None):
        self.flown, self.rotors = flown, []
        emu = str(port or '').startswith('emulator://')
        bus = 'emulator://?nodes=%d' % count if port == 'emulator://' else str(port)
        try:
            for u in range(1, count + 1):
                rig = (Coaxial63100(port=bus, unit=u, execution_mode=EMULATED).open()
                       if emu else Coaxial63100(execution_mode=SIMULATED).open())
                self.rotors.append(flown.arm(rig))
            self.sky = craft.Sky(grounds.world(getattr(flown, 'FIELD', None)))
        except Exception:
            self.close()
            raise
        self.route, self.law, self.flight = aerobatics.routine(flown.CARD), aerokin.fresh(), flown.fresh()
        self.t, self.epoch, self.aim, self.frame = 0.0, 0, gimbal.REST, self.sky.state()
        self.origin = self.rotors[0].get('origin') or self.rotors[0]['rig'].origin

    def apply(self, command):
        """{'reset': True}: the flight begun again on its spot, a new epoch; {'wind': True}: the
        air it flies in kept."""
        flight = self.flight
        if command.get('wind'):
            weather.kept(flight['air'])
        if command.get('reset'):
            self.sky.reset()
            self.route.pop('lap', None)
            self.route.update(aerobatics.routine(self.flown.CARD), at=self.t)
            self.flown.begun(self.law, flight)
            self.flown.reset(self.rotors)
            flight['cells'].update(flightpack.pack())
            flight.update(apex=0.0, share=1.0, over=0.0, gone=0.0, stood=0.0, wreck=None)
            self.frame, self.epoch = self.sky.state(), self.epoch + 1

    def step(self):
        """A step of STEP_S, its frame and the carried camera's aim after it; nothing left out
        of the pace."""
        for self.t, self.frame in self.flown.passed(self.rotors, self.sky, self.route, self.law,
                                                    self.flight, self.t, qflight.STEP_S):
            self.aim = gimbal.step(self.aim, self.frame['turn'], self.frame['vel'], qflight.STEP_S)
        return 0.0

    def said(self, _ratio, _headroom=1.0):
        """The flight as the page draws it, copied whole: its process goes on stepping."""
        air = self.flight.get('air') or {}
        return {'t': self.t, 'epoch': self.epoch, 'frame': self.frame, 'aim': self.aim,
                'heading': tuple(self.law['heading']), 'lined': qflight.lined(self.route),
                'origin': self.origin,
                'rotors': [copy.deepcopy({k: r.get(k) for k in ROTOR}) for r in self.rotors],
                'flight': copy.deepcopy(dict({k: self.flight.get(k) for k in FLIGHT},
                                             air={k: air.get(k) for k in AIR}))}

    def close(self):
        """Its drives off, gates off, its stand-ins and emulated boards closed."""
        for rotor in self.rotors:
            for rig in (rotor.get('emu'), rotor['rig']):
                if rig is not None:
                    with suppress(RigError):
                        rig.board.drive.off()
                        rig.gates.off()
                    rig.close()


def run(commands, states, shown, craft, flown, count, setup=None, port=None):
    """The flight apart (`apart.paced`), no further than its lead past what the page has `shown`;
    `setup` (a function, its arguments) run on its flight's module first; its own prints kept
    off the page's terminal, a fault said as {'error': its trace}."""
    null = os.open(os.devnull, os.O_WRONLY)
    os.dup2(null, 1)
    os.dup2(null, 2)
    sys.stdout = sys.stderr = open(os.devnull, 'w', encoding='utf-8')
    placed.placed()
    flight = None
    try:
        flown = importlib.import_module(flown)
        if setup is not None:
            setup[0](flown, *setup[1])
        flight = Flight(importlib.import_module(craft), flown, count, port=port)
        apart.paced(flight, commands, states, shown=shown, rest_s=REST_S)
    except Exception:
        apart.put(states, {'error': traceback.format_exc()})
    finally:
        if flight is not None:
            flight.close()


class Flying(apart.Apart):

    """A craft's flight apart (`run`) for its page: the states it has said since asked, the first
    awaited, what the page has shown told it, keys sent it; `setup` (a function, its arguments)
    run on its flight's module in its process before it flies - a card, a pack of its own."""

    def __init__(self, craft, flown, count, setup=None, port=None):
        super().__init__(run, craft.__name__, flown.__name__, count, setup, port, states=64, shown=0.0)

    def said(self):
        """Every state said since the last asked; a fault in the flight raised as RuntimeError."""
        out = []
        self.latest(into=out)
        for state in out:
            if 'error' in state:
                raise RuntimeError('the flight: ' + state['error'].strip().splitlines()[-1])
        return out

    def first(self, wait_s=FIRST_S):
        """The states said up to the first, waited for `wait_s` s at the most."""
        until = time.monotonic() + wait_s
        while time.monotonic() < until:
            out = self.said()
            if out:
                return out
            if not self.alive():
                raise RuntimeError('the flight ended before it said a state')
            time.sleep(0.02)
        raise RuntimeError('the flight said nothing in %.0f s' % wait_s)


def blend(a, b, k):
    """The frame and the rotors `k` of the way from state `a` to `b`: places, speeds and the turn
    - made a rotation again - between them, a rotor's angle on at its speed; across a reset, `b`."""
    if a['epoch'] != b['epoch']:
        return {}
    fa, fb = a['frame'], b['frame']
    frame = dict(fb)
    for key in ('h', 'v', 'a', 'at', 'vel', 'spin', 'acc'):
        frame[key] = fa[key] + (fb[key] - fa[key]) * k
    frame['turn'] = rotation(fa['turn'] + (fb['turn'] - fa['turn']) * k)
    span = (b['t'] - a['t']) * k
    return {'frame': frame, 'rotors': [dict(rb, w=ra['w'] + (rb['w'] - ra['w']) * k,
                                            angle=(ra['angle'] + ra['w'] * span) % math.tau)
                                       for ra, rb in zip(a['rotors'], b['rotors'])]}


def rotation(m):
    """The rotation nearest the 3x3 `m`."""
    import numpy as np
    u, _s, vt = np.linalg.svd(m)
    return u @ vt
