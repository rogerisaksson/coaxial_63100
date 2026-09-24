"""The meter bridge's load: the sweep, the filter chain the link can carry, the plan."""
import time

from coaxial.acquire import bessel


def drain(rig, cap=32):
    """Every record the ring is holding, up to `cap`."""
    got = []
    while len(got) < cap:
        block = rig.acquire()
        if not block:
            break
        got.extend(block)
    return got


def rows_from(records, layout):
    """A drained block as the rows this renderer already understands."""
    out = []
    for field in layout['fields']:
        name = field['signal']
        total = sum(r[name] for r in records)
        count = sum(max(1, r['samples']) for r in records)
        means = [r[name] / max(1, r['samples']) for r in records]
        out.append({
            'index': field['channel'],
            'signal': name,
            'unit': field['unit'],
            'differential': field['differential'],
            'mean_raw': total / max(1, count),
            'min_raw': min(means),
            'max_raw': max(means),
            'samples': count,
        })
    return out


def duties_from(records):
    """Each pin's duty, meaned over the block."""
    seen = [r['digital'] for r in records if r.get('digital')]
    if not seen:
        return {}
    return {name: sum(d[name] for d in seen) / len(seen)
            for name in seen[0]}


#: Records the host holds between the reader and the frame that draws
#: them. Its own ring, and it can overflow the same way the board's
#: does - which is the point of showing both.
HOST_RING = 2048

#: How much of what the board says it can carry a task actually asks
#: for. A ring produced at exactly the drain rate overflows on the first
#: slow read; `bessel.for_link` uses the same 0.8 for the same reason.
#: What a task asks for against what the board says it carries. OVER
#: ONE on purpose: the board should stay a little ahead of the link
#: so every read finds a full reply and the ring absorbs the rest.
#: Measured at ten channels and stride 55, ring flat and nothing
#: dropped: 0.8 held 39% of the line, 1.2 held 56%, 1.45 held 65% and
#: 2.5 holds 73%. Higher asks buy throughput by taking DECIMATION out
#: - ratio 7, then 4, then 2 - and two is the floor: at one there is
#: nothing left for the chain to shape. The passband follows the rate
#: that actually comes out (see `bessel.design`), so this cannot buy
#: speed by putting the cutoff above Nyquist, which is what it did
#: before that was fixed.
LINK_SHARE = 2.5

#: Sweeps a record below which the anti-alias chain would cost more
#: link than it is worth - the loop spends N sweeps for one record
#: and those sweeps come off the link, because sampling and the
#: Modbus handler share main(). Not reached since the board started
#: sampling while the UART drains, which took the loop from 380 to
#: 1880 sweeps/s and left plenty to decimate.
MIN_OVERSAMPLE = 4.0

#: How far ahead of the link the board is asked to run when nothing
#: is gating it. Measured: 2x gave 53% of the line, 4x 63%, 6x 68%,
#: 10x nothing more - the transaction floor is what is left.
RUN_AHEAD = 6.0


def sweep_rate(rig, records=300, timeout=6.0):
    """What the acquisition loop manages, in sweeps a second."""
    rig.shape()
    rig.configure(accumulate=1, digital=True, records=records,
                  interval_us=0)
    began = time.time()
    rig.start()
    while time.time() - began < timeout and not rig.state()['done']:
        time.sleep(0.005)
    span = time.time() - began
    state = rig.state()
    rig.stop()
    return (state['produced'] + state['dropped']) / max(span, 1e-6)


def under_load(rig, settle=0.5, window=1.5):
    """(sweeps/s, records/s, ring drift) while the link is streaming."""
    rig.start()
    time.sleep(settle)                        # the reader reaches its pace
    was = rig.state() or {}
    first = rig.buffered
    began = time.time()
    time.sleep(window)
    now = rig.state() or {}
    last = rig.buffered
    span = time.time() - began
    rig.stop()
    if span <= 0 or was.get('triggers') is None:
        return 0.0, 0.0, 0, 0.0
    sweeps = max(0.0, (now['triggers'] - was['triggers']) / span)
    records = max(0.0, (last['records'] - first['records']) / span)
    drift = (now.get('available') or 0) - (was.get('available') or 0)
    reads = max(1e-9, (last['reads'] - first['reads']) / span)
    return sweeps, records, drift, records / reads


def load(rig, args, sweeps, rate):
    """Design for `sweeps` and put it on the board. (layout, chain)."""
    rig.shape()
    try:
        chain = bessel.design(fs=sweeps, out_rate=rate, order=args.order)
    except ValueError:
        # A CHAIN NEEDS A RATE TO BE DESIGNED AGAINST.
        return rig.configure(sample_rate=rate, digital=True), None
    layout = rig.configure(accumulate=chain['boxcar'], digital=True)
    rig.shape(chain['sections'], chain['decimate'])
    chain['sweeps'] = sweeps
    return layout, chain


def plan(rig, args):
    """Measure what the loop gives, design the low-pass for it, load it."""
    # TAKING THE BOARD OVER STARTS BY TAKING IT OVER.
    rig.stop()
    sweeps = sweep_rate(rig)
    # WHAT THE LINK CARRIES, NOT WHAT THE SCREEN DRAWS.
    carries = (rig.state() or {}).get('max_rate_hz') or 0
    rate = args.rate if args.rate > 0 else carries * LINK_SHARE
    if rate <= 0:
        rate = max(1.0, args.hz)

    # A CHAIN ONLY EARNS ITS KEEP ON OVERSAMPLING.
    layout, chain = load(rig, args, sweeps, rate)
    if chain is None:
        return layout, chain

    # ONE REDESIGN, AGAINST THE LOOP IT WILL RUN IN.
    live, made, drift, per_read = under_load(rig)
    off = live and abs(live - sweeps) > 0.2 * sweeps
    fresh = load(rig, args, live, rate) if off else (None, None)
    if fresh[1] is not None:
        layout, chain = fresh
        chain['idle_sweeps'] = sweeps
    return layout, chain


def _line_share(link):
    """How much of the line the stream is actually claiming, when the link
    knows its baud and its bits.
    """
    if not (link.get('baud') and link.get('bits')):
        return ''
    return '   %2.0f%% of line' % (100.0 * link['bits'] / float(link['baud']))


#: Bytes a read costs beyond its records: unit, function code and CRC
#: on the request and the reply, plus the count byte and the backlog the
#: reply appends.
PER_READ_BYTES = 4 + 4 + 1 + 4


def wire_rate(bits):
    """Bits a second as the largest unit that still leaves digits."""
    if bits >= 1e6:
        return '%.1f Mbit/s' % (bits / 1e6)
    if bits >= 1e3:
        return '%.1f kbit/s' % (bits / 1e3)
    return '%.0f bit/s' % bits


def take_link(link, seen, now):
    """Fold one sample of the library reader's queue into `link`."""
    if not link['at']:
        link['reads'], link['seen'] = seen['reads'], seen['records']
        link['at'] = now
    elif now - link['at'] > 0.2:
        since = now - link['at']
        link['rate'] = (seen['reads'] - link['reads']) / since
        # WHAT ACTUALLY GOES DOWN THE WIRE, not the records' own size: a
        # record's bytes plus the transaction around it - unit, function, the
        # count byte, the backlog and the CRC - and ten bits a byte, because
        # 8N1 sends a start and a stop with every one.
        payload = (seen['records'] - link['seen']) * link['stride']
        frames = (seen['reads'] - link['reads']) * PER_READ_BYTES
        link['bits'] = (payload + frames) * 10.0 / since
        link['reads'], link['seen'] = seen['reads'], seen['records']
        link['at'] = now
    link.update({k: seen[k] for k in ('host', 'peak', 'dropped', 'backlog')})


def _clocked(clock, rig, now):
    """The board's state, and the loop's own rate differentiated off the
    board's trigger count - live, and not the figure the chain was
    designed against: the two part company the moment the link is busy.
    """
    state = rig.state()
    seen, since = state.get('triggers'), now - clock['at']
    if seen is not None and clock['triggers'] is not None and since:
        clock['sweeps'] = (seen - clock['triggers']) / since
    clock['triggers'] = seen
    clock['state'], clock['at'] = state, now
