"""The stand-in's motor: the model the law runs on, and what the board's other parts read of it."""
import math
from typing import Any

from coaxial.devices.drive import MODE_NAMES, MODEL_IDS
from coaxial.simulated.drive.locked import _rotor_locked
from motor.pmsm import RAD_S_PER_RPM

#: 1 / sqrt 3: beta off two phases.
INV_SQRT3 = 1.0 / math.sqrt(3.0)


class DrivePlant:

    """The PMSM under the stand-in's drive (drive_model.c): its parameters, its truth, and the
    currents, duties and speed the stand-in's heat, converters and shaft sensor take off it."""

    # What the class this mixes into brings.
    DEFAULTS: Any
    _law: Any
    _lib: Any
    _lock: Any
    _model: Any
    _now: Any
    _params: Any
    _slot: Any
    _source: Any
    _stage: Any
    _switching: Any

    @_rotor_locked
    def _set_model(self, **values):
        for name in values:
            if name not in MODEL_IDS:
                raise ValueError('%r is not a model parameter; they are %s'
                                 % (name, ', '.join(MODEL_IDS)))
        self._now()                              # the old parameters' time, first
        self._model.update({k: float(v) for k, v in values.items()})
        # Live, as Board_DriveModelParam writes them under a turning rotor.
        self._law.model_params(**self._model)
        # Pole pairs divide the rotor's own angle: a different motor, at rest.
        if 'pole_pairs' in values:
            self._lib.world_drive_rest(self._slot)
        return dict(values)

    @_rotor_locked
    def _read_model(self):
        """The rotor the law runs on, and the estimate beside it."""
        law, _row = self._now()
        motor = self._law.model_state()
        return {'source': self._source, 'theta': motor['theta'], 'omega': motor['omega'],
                'id': motor['id'], 'iq': motor['iq'], 'vdc': self._model['vdc'],
                'theta_hat': law['theta_hat'], 'omega_hat': law['omega_hat'],
                'error': (law['theta_hat'] - motor['theta'] + math.pi) % math.tau - math.pi}

    @_rotor_locked
    def _reset_model(self):
        """The rotor back to theta0, at rest - the contract `drive.py` states."""
        self._now()
        self._lib.world_drive_rest(self._slot)
        return True

    # -- what the board's other parts read -------------------------------

    @_rotor_locked
    def sample(self):
        """What a sampler in the control interrupt sees now: the phases' amps - none on a
        bridge that does not switch, the model source's being its own -, whether it does, the
        link."""
        law, row = self._now()
        on = self._stage()[1] if self._switching else law['mode'] != 0.0
        amps = (row['amps_u'], row['amps_v'], row['amps_w']) if on else (0.0, 0.0, 0.0)
        return {'amps': amps, 'switching': bool(on), 'link': self._model['vdc']}

    @_rotor_locked
    def _carrying(self):
        """(amps, electrical angle) of the stator's current vector now."""
        _law, row = self._now()
        alpha = row['amps_u']
        beta = (row['amps_v'] - row['amps_w']) * INV_SQRT3
        return math.hypot(alpha, beta), math.atan2(beta, alpha) % math.tau

    @_rotor_locked
    def _duty(self):
        """What the modulator put on the compares last period; none on an open bridge."""
        _law, row = self._now()
        if not row['driven']:
            return (0.0, 0.0, 0.0)
        return (row['duty_u'], row['duty_v'], row['duty_w'])

    @_rotor_locked
    def _dq(self):
        """(id, iq, vd, vq) in the loop's frame, this period."""
        law, _row = self._now()
        return law['id'], law['iq'], law['vd'], law['vq']

    @_rotor_locked
    def _omega(self):
        """The loop's frame, rad/s electrical: the command's in a command frame, the estimate's
        sensorless, none off."""
        law, _row = self._now()
        mode = MODE_NAMES[int(law['mode'])]
        if mode in ('hold', 'volt'):
            return law['omega_cmd']
        return law['omega_hat'] if mode == 'sensorless' else 0.0

    @property
    def _mode(self):
        """The law's mode, by name."""
        with self._lock:
            return MODE_NAMES[int(self._now()[0]['mode'])]

    @property
    def _mech(self):
        """The shaft, rad mechanical, unwrapped: what a shaft sensor reads."""
        with self._lock:
            return self._now()[1]['shaft']

    @_rotor_locked
    def _rpm(self):
        """The rotor, rpm mechanical, as board_thermal.c's speed_now takes it: off the estimate
        the loop runs on, by the record's pole pairs; none while off. By the bench motor's 7 the
        QUAD page's 63100, 14, cooled its boards on 2 941 rpm at 1 471."""
        law = self._now()[0]
        if law['mode'] == 0.0:
            return 0.0
        pairs = self._params.get('motor_pole_pairs') or self.DEFAULTS['motor_pole_pairs']
        return abs(law['omega_hat']) / pairs / RAD_S_PER_RPM
