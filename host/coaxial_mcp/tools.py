"""Tool schemas and handlers."""
import os
import re
import subprocess
import sys
import time

from coaxial import DividerParams, NtcParams
from coaxial.comm import protocol
from coaxial.comm.wire import pack
from coaxial.devices import scaling
from coaxial.draw import orientation as orient
from coaxial.errors import LINK_FAULTS, DeviceStateError

ROTATION_VECTOR = 0x05

from. import render
from .docs import docs as _docs
from coaxial_mcp.bus import devices, link
from coaxial_mcp.names import _resolve, _split_pin
from coaxial_mcp.schema import BOARD_INFO_KINDS

#: What asking an older board for something newer raises: a link fault, no
#: subsystem or op behind the attribute, no such field in the reply.
OLDER_FIRMWARE = LINK_FAULTS + (AttributeError, KeyError)


# ---- handlers --------------------------------------------------------------


def angle(session, op='read', **_):
    """The A1335's reading, off the board's poll loop."""
    part = session.board.angle

    if op == 'registers':
        with part.configuring():
            rows = [(name, part.peek(reg)['value'])
                    for reg, name in ((0x20, 'ANG'), (0x22, 'STA'),
                                      (0x24, 'ERR'), (0x28, 'TSEN'),
                                      (0x2A, 'FIELD'))]
        return render.angle_registers(rows)

    return render.angle(part.state())


def thermal(session, op='state', **_):
    """The thermal observer behind device 8, as three questions."""
    part = session.board.thermal
    if op == 'budget':
        return render.thermal_budget(part.budget())
    if op == 'ident':
        return render.thermal_ident(part.identification())
    return render.thermal_state(part.state())


def board_info(session, refresh=False, kind='all', **_):
    if kind not in BOARD_INFO_KINDS:
        return 'ERR unknown kind %r; try %s' % (kind,
                                                ', '.join(BOARD_INFO_KINDS))
    version, clock, channels = session.info(refresh=refresh)
    # The pins come from the board too (command 0x6D).
    if kind == 'parts':
        # What is fitted, which only the firmware knows: one entry per part,
        # reported by channels kind 4, with what powers each one.
        got = session.board.system.channel_map(refresh=refresh)
        return render.parts(got.get('parts') or [])

    if kind == 'subsystems':
        # What the board is made of, which only the firmware knows: one entry
        # per command table, reported by channels kind 3.
        got = session.board.system.channel_map(refresh=refresh)
        return render.subsystems(got.get('subsystems') or [])

    section = 'reserved' if kind == 'reserved' else 'digital'
    try:
        pins = session.board.system.channel_map(refresh=refresh)[section]
    except OLDER_FIRMWARE:
        pins = None
    return render.board_info(version, clock, channels, pins, kind)


def analog_read(session, ch=None, samples=64, rate_hz=2000.0,
                ntc_beta=None, ntc_r25=None, vref=3.3, **_):
    board = session.board

    # Resolve the arguments BEFORE checking the board's state.
    _, _, channels = session.info()
    notes = []
    indices = (_resolve(session, ch, notes) if ch
               else [c['index'] for c in channels])

    # Read either way, and say which it was.
    afe_on = bool(board.afe.state().get('on'))

    mask = 0
    for index in indices:
        mask |= 1 << index

    burst = board.analog.burst(mask, samples, rate_hz)

    # Derive only the two quantities whose conversion is known.
    own = board.analog.scaling()
    ntc = NtcParams(r25=ntc_r25 or own['ntc'].r25,
                    beta=ntc_beta or own['ntc'].beta,
                    r_fixed=own['ntc'].r_fixed,
                    t25_kelvin=own['ntc'].t25_kelvin)
    divider = DividerParams(r_top=own['dcbus'].r_top,
                            r_bottom=own['dcbus'].r_bottom, vref=vref)

    rows = []
    derived = {}
    for index in sorted(burst['channels']):
        meta = channels[index]
        stats = burst['channels'][index]
        convert = (scaling.differential_volts if meta['differential']
                   else scaling.single_ended_volts)
        row = dict(meta)
        row.update(stats)
        row['volts_at_pin'] = convert(stats['mean_raw'], vref)
        rows.append(row)

        if meta['signal'] == 'NTC':
            try:
                derived[index] = '%.2fC' % ntc.celsius(stats['mean_raw'])
            except ValueError as exc:
                # A rail reading (a genuinely open or shorted thermistor, not
                # the AFE-off case - that lands at mid-scale, not a rail) makes
                # the conversion undefined.
                derived[index] = 'no conversion: %s' % exc
        elif meta['signal'] == 'DC bus':
            derived[index] = '%.3fV bus' % divider.volts(stats['mean_raw'])

    banner = ('' if afe_on else
              'AFE OFF - the ADC reference is unpowered. These are the codes '
              'the converter returned, not measurements: every channel sits '
              'near mid-scale, and the degC and volts below are arithmetic '
              'on that - not a temperature, not a bus voltage. Call '
              'afe_power on to measure.' + '\n')
    # What had to be corrected to answer at all, above the answer.
    if notes:
        banner = ('read as asked, with corrections: %s%s'
                  % ('; '.join(notes), '\n')) + banner
    return banner + render.analog({'samples': burst['samples'],
                          'rate_hz': burst['rate_hz'],
                          'channels': rows}, derived)


def self_test(session, failures_only=False, **_):
    checks = session.board.system.self_test()
    failed = [c for c in checks if c['status'] == 'fail']
    if failures_only and not failed:
        return 'all pass'
    return render.checks(failed if failures_only else checks)


def imu(session, op='read', report_id=None, interval_us=None, **_):
    """The IMU. Reads by default: it is the question that gets asked."""
    part = session.board.imu

    if op == 'id':
        with part.configuring():
            return render.imu('id', part.product_id())

    if op == 'feature':
        return _imu_feature(part, report_id, interval_us)

    return render.imu('state', part.state())


def _imu_feature(part, report_id, interval_us):
    """Enable one report at an interval, or disable it with none."""
    if report_id is None:
        raise ValueError("op='feature' needs report_id - 1 accelerometer, "
                         "2 gyroscope, 3 magnetic field, 5 rotation vector")
    part.configure({int(report_id): int(interval_us or 0)}, reset=True)
    return 'imu: report 0x%02X %s' % (
        int(report_id),
        'every %d us' % int(interval_us) if interval_us else 'disabled')


#: How often the rotation vector is asked for when `orientation` has to
#: enable it, and how many looks at the record it gets to arrive in.
ORIENTATION_INTERVAL_US = 20000
ORIENTATION_LOOKS = 20


def _rotation_vector(part):
    """The loop's newest record, with a rotation vector in it when the part is
    reporting one.
    """
    got = part.state()
    if got['quaternion'] is not None:
        return got
    part.configure({ROTATION_VECTOR: ORIENTATION_INTERVAL_US}, reset=True)
    for _ in range(ORIENTATION_LOOKS):
        got = part.state()
        if got['quaternion'] is not None:
            return got
    return got


def orientation(session, op='once', **_):
    """One picture, or a window that keeps drawing them."""
    if op == 'show':
        return _open_orientation_window(session)

    got = _rotation_vector(session.board.imu)
    if got['quaternion'] is None:
        raise DeviceStateError(
            'the IMU sent no rotation vector. It is enabled now, so a second '
            'call may find one; if not, the part is not reporting. AFE_ON '
            'powers it - with that off it answers reads and acts on no write.')

    q = got['quaternion']
    return orient.picture((q['i'], q['j'], q['k'], q['real']))


def _open_orientation_window(session):
    """Start the live view in a console of its own, and say so."""
    tools_dir = os.path.dirname(os.path.abspath(__file__))
    script = os.path.join(os.path.dirname(tools_dir), 'terminal', 'views',
                          'show_orientation.py')
    port = session.port or 'COM4'

    if os.name != 'nt':
        return ('a live view needs its own console and this is not Windows - '
                'run: python terminal/views/show_orientation.py --port %s' % port)

    subprocess.Popen(
        [sys.executable, script, '--port', str(port)],
        creationflags=subprocess.CREATE_NEW_CONSOLE,
        cwd=os.path.dirname(tools_dir))

    return ('orientation: a window is drawing the board live from %s. '
            'Ctrl+C in it to stop; it puts the IMU back as it found it.'
            % port)


def _multicast(session):
    return session.unit == protocol.BROADCAST


def afe_power(session, action='read', **_):
    if _multicast(session):
        return _afe_order(session, action)
    afe = session.board.afe
    if action != 'read':
        _afe_switch(session, afe, action)
    return render.kv(afe.state())


def _afe_order(session, action):
    """An order to every node, not a request."""
    if action not in ('on', 'off'):
        return ('ERR %s needs a reply and a broadcast has none; '
                'select one node, or use on/off' % action)
    session.board.broadcast(
        protocol.AFE, pack(('u8', protocol.AFE_ACTIONS[action])))
    return ('afe %s sent to every node - broadcast, so no read-back '
            'and no confirmation' % action)


def _afe_switch(session, afe, action):
    """Switch the rail, and wait out the reference when this turned it on."""
    was = afe.state()['on']
    {'on': afe.on, 'off': afe.off, 'toggle': afe.toggle}[action]()
    if afe.is_on() and not was:
        _settle(session)


def _settle(session):
    """Wait out the reference after AFE_ON went high."""

    try:
        wait = session.board.thermal.state()['sample_settle_s']
    except OLDER_FIRMWARE:
        wait = 0.5
    time.sleep(min(2.0, max(0.0, wait)))


def gpio_pin(session, op='read', pin='B2', level=False, mode='input',
             pull='none', **_):
    port, number = _split_pin(pin)
    gpio = session.board.gpio

    if op == 'read':
        return '%s=%d' % (pin.upper(), gpio.read(port, number))
    if op == 'write':
        return '%s=%d readback' % (pin.upper(),
                                   gpio.write(port, number, level))
    gpio.configure(port, number, mode, pull)
    return '%s mode=%s pull=%s' % (pin.upper(), mode, pull)


def digital_read(session, **_):
    """The level of every digital I/O channel, from the board's own map."""
    pins = session.board.system.channel_map()['digital']
    gpio = session.board.gpio
    rows = []
    for entry in pins:
        # The map spells them "PB2"; _split_pin takes "B2".
        name = entry['pin']
        port, number = _split_pin(name[1:] if name[:1] == 'P' else name)
        rows.append(dict(entry, level=int(bool(gpio.read(port, number)))))
    return render.digital_levels(rows)


def gpio_port(session, op='read', port='E', mask=0, value=0, **_):
    letter = str(port).strip().upper()[:1]
    gpio = session.board.gpio
    reserved = {p for (prt, p) in protocol.RESERVED_PINS if prt == letter}

    if op == 'read':
        return render.port_bits(letter, gpio.port_read(letter), reserved)
    return render.port_bits(letter, gpio.port_write(letter, mask, value),
                            reserved)


def program(session, op='card', text='', machine='humanoid', **_):
    """The machine the session's buses make - found once, each type built once: its card, a
    whole run, or live - start, send (a line plays once checked), wait (one line), now, stop."""
    from machine import Machine, MachineError, Nodes
    from machine.live import Live
    held = vars(session)
    machines, lives = held.setdefault('machines', {}), held.setdefault('lives', {})
    if machine not in machines:
        if 'nodes' not in held:
            held['nodes'] = Nodes.discover(port=session.port,
                                           simulated=bool(getattr(session, 'simulated', False)))
        machines[machine] = Machine(held['nodes'], type=machine)
    built, live = machines[machine], lives.get(machine)
    if op == 'card':
        return built.prompt()
    if op == 'now':
        return built.status()
    if op == 'start':
        if live is None or live.status in ('stopped', 'tripped', 'fault'):
            lives[machine] = live = Live(built).start()
        return 'started, failsafe %s | %s' % (built.failsafe, built.status(changed=True))
    if op in ('send', 'wait', 'stop'):
        if live is None:
            raise MachineError('%s is not live - op=start first' % machine)
        if op == 'send':
            refused = live.feed(text + '\n') + live.flush()
            return '\n'.join(refused) or 'queued, %.2f s to play' % live.buffered()
        if op == 'wait':
            return live.wait()
        state = lives.pop(machine).stop()
        return '%s%s | %s' % (state['status'], ' - ' + state['reason'] if state['reason'] else '',
                              built.status(changed=True))
    if live is not None:
        raise MachineError('%s is live - op=stop first, or op=send' % machine)
    named = [name for name in built.actuators if re.search(r'\b%s\b' % name, text)]
    return built.run(text).summary(*['%s.%s' % (name, built.actuators[name].BACK)
                                     for name in named])


def close_programs(session):
    """What `program` opened on this session: live runs stopped, the nodes closed."""
    held = vars(session)
    for live in held.pop('lives', {}).values():
        live.stop()
    held.pop('machines', None)
    nodes = held.pop('nodes', None)
    if nodes is not None:
        nodes.close()


def test_gate(session, enable=False, **_):
    gpio = session.board.gpio
    return 'gate=%d' % (gpio.on() if enable else gpio.off())


HANDLERS = {
    'board_info': board_info,
    'docs': _docs,
    'self_test': self_test,
    'analog_read': analog_read,
    'afe_power': afe_power,
    'devices': devices,
    'digital_read': digital_read,
    'imu': imu,
    'angle': angle,
    'orientation': orientation,
    'gpio_pin': gpio_pin,
    'gpio_port': gpio_port,
    'test_gate': test_gate,
    'program': program,
    'thermal': thermal,
    'link': link,
}
