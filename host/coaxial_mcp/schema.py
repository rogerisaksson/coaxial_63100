"""The tools as MCP declares them, and each argument coerced to its schema."""


_PIN = {'type': 'string', 'description': 'Pin as PORT+NUMBER, e.g. B2 or E15'}

_PORT = {'type': 'string', 'description': 'Port letter A-K'}

# Which section of the map board_info answers with.
BOARD_INFO_KINDS = ('all', 'analog', 'digital', 'reserved', 'identity',
                    'subsystems', 'parts')

TOOLS = [
    {
        'name': 'board_info',
        'description': "Identity, clock, the channels the board reports with their directions, and what is fitted on it. kind narrows it to one section.",
        'description_terse': "Identity, clock, channels, fitted parts. kind: analog|digital|reserved|identity|subsystems|parts|all.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'refresh': {'type': 'boolean'},
                'kind': {'type': 'string',
                         'enum': list(BOARD_INFO_KINDS),
                         'description': 'which section; omit for all'},
            },
        },
    },
    {
        'name': 'analog_read',
        'description': "Sample the board's ADC channels: mean raw, pin volts, plus degC for the NTC and volts for the DC bus. Needs the AFE on. Not the IMU.",
        'description_terse': "The board's ADC channels: mean raw, pin volts, NTC degC, bus volts. Not the IMU.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'ch': {'type': 'array', 'items': {'type': 'string'},
                       'description': 'Channel indices or short names; omit for all'},
                'samples': {'type': 'integer'},
                'rate_hz': {'type': 'number'},
                'ntc_beta': {'type': 'number', 'description': 'Override thermistor B'},
                'ntc_r25': {'type': 'number'},
                'vref': {'type': 'number', 'description': 'Measured reference, default 3.3'},
            },
        },
    },
    {
        'name': 'docs',
        'description': "This board's own documents: background, not a live reading. find=TEXT to answer a question; doc= alone returns headings, not answers.",
        'description_terse': "This board's documents, not a reading. find=TEXT answers; doc= alone lists headings.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'doc': {'type': 'string',
                        'description': 'README|CLAUDE|ARCHITECTURE|PROTOCOL|HARDWARE|FINDINGS|MODELS'},
                'section': {'type': 'string', 'description': 'A heading from that document'},
                'find': {'type': 'string', 'description': 'Search all of them instead'},
            },
        },
    },
    {
        'name': 'self_test',
        'description': 'What the board can prove about itself: clocks, calibration, firmware checksum. Pass/fail only where no external reference is needed.',
        'description_terse': 'What the board proves about itself: clocks, calibration, checksum.',
        'inputSchema': {
            'type': 'object',
            'properties': {'failures_only': {'type': 'boolean'}},
        },
    },
    {
        'name': 'imu',
        'description': "The IMU (BNO08X): accelerometer, gyro, magnetometer readings, not the ADC channels. op='read' values, 'id' the part, 'feature' enable.",
        'description_terse': "The IMU's readings - accelerometer, gyro, magnetometer. Not the ADC channels. op='read' values, 'id' the part, 'feature' enable.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'type': 'string', 'enum': ['id', 'read', 'feature']},
                'report_id': {'type': 'integer'},
                'interval_us': {'type': 'integer'},
            },
        },
    },
    {
        'name': 'angle',
        'description': "The angle sensor (A1335) on SPI4: shaft angle in degrees, its die temperature, the field it sees. Not the IMU, not an ADC channel.",
        'description_terse': "The angle sensor (A1335): shaft angle, die temperature, field strength. Not the IMU.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'type': 'string', 'enum': ['read', 'registers']},
            },
        },
    },
    {
        'name': 'orientation',
        'description': "How the board is turned or oriented, drawn as a picture from the IMU's rotation vector. op='show' opens a terminal that redraws it live.",
        'description_terse': "How the board is turned or oriented, as a picture. op='show' opens a terminal that redraws it live.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'type': 'string', 'enum': ['once', 'show']},
            },
        },
    },
    {
        'name': 'afe_power',
        'description': 'Analog front end switch. It also powers the ADC reference, so readings are meaningless with it off.',
        'description_terse': 'Front end switch. It also powers the ADC reference.',
        'inputSchema': {
            'type': 'object',
            'properties': {'action': {'enum': ['on', 'off', 'toggle', 'read']}},
            'required': ['action'],
        },
    },
    {
        'name': 'devices',
        'description': "Nodes on every bus, and what each says it is. op=buses lists the segments; op=use name='right knee' picks one.",
        'description_terse': "Nodes on every bus. op=buses for segments; op=use + name='right knee' picks one.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'enum': ['list', 'buses', 'use']},
                'unit': {'type': 'integer'},
                'name': {'type': 'string',
                         'description': 'what a node calls itself'},
                'bus': {'type': 'string', 'description': 'segment, e.g. RL'},
            },
        },
    },
    {
        'name': 'digital_read',
        'description': 'Read the digital channels: the value, 0 or 1, of every one, now. Values, not the list - board_info lists them.',
        'description_terse': 'Values of every digital channel, 0 or 1, now. Not the list.',
        'inputSchema': {'type': 'object', 'properties': {}},
    },
    {
        'name': 'gpio_pin',
        'description': 'Read, drive or configure one pin. Writing and configuring need test_gate open.',
        'description_terse': 'Read, drive or configure one pin. Write and mode need test_gate.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'enum': ['read', 'write', 'mode']},
                'pin': _PIN,
                'level': {'type': 'boolean'},
                'mode': {'enum': ['input', 'output', 'output_od', 'analog']},
                'pull': {'enum': ['none', 'up', 'down']},
            },
            'required': ['op', 'pin'],
        },
    },
    {
        'name': 'gpio_port',
        'description': 'Read a whole 16-bit port, or drive a masked set of its pins atomically. Writing needs test_gate open.',
        'description_terse': 'Read a 16-bit port, or drive masked pins. Write needs test_gate.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'enum': ['read', 'write']},
                'port': _PORT,
                'mask': {'type': 'integer'},
                'value': {'type': 'integer'},
            },
            'required': ['op', 'port'],
        },
    },
    {
        'name': 'test_gate',
        'description': 'Open or close raw pin access. Reads work without it; writes do not.',
        'description_terse': 'Open or close raw pin access. Reads work without it.',
        'inputSchema': {
            'type': 'object',
            'properties': {'enable': {'type': 'boolean'}},
            'required': ['enable'],
        },
    },
    {
        'name': 'thermal',
        'description': "Thermal observer: op=state NTC and each node's estimated degC, op=budget SOA spend and clamp, op=ident identification and margin.",
        'description_terse': "Thermal observer: op=state NTC and every node's estimate, op=budget SOA spend and clamp, op=ident the identification and the margin.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'enum': ['state', 'budget', 'ident']},
            },
        },
    },
    {
        'name': 'program',
        'description': "Move the joints: op=card gives the grammar and the joints; op=run runs `text`, one step a line (seconds, joint=deg), and says how it ended.",
        'description_terse': "op=card: the grammar and the joints. op=run: run `text`, one step a line (seconds, joint=deg); answers how it ended.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'enum': ['card', 'run']},
                'text': {'type': 'string'},
                'machine': {'enum': ['humanoid', 'quad', 'fixed_wing', 'ebike']},
            },
            'required': ['op'],
        },
    },
    {
        'name': 'link',
        'description': 'echo tests the wire, state returns frame counters, release hands the UART back to the text console.',
        'description_terse': 'echo tests the wire, state returns counters, release hands back the console.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'op': {'enum': ['echo', 'state', 'release']},
                'text': {'type': 'string'},
            },
            'required': ['op'],
        },
    },
]

SCHEMAS = {spec['name']: spec['inputSchema'].get('properties', {})
           for spec in TOOLS}

TRUE = ('1', 'true', 'yes', 'on')


def coerce(name, arguments):
    """The arguments as the schema declares them, whatever the model sent."""
    properties = SCHEMAS.get(name) or {}
    coerced = {}
    for key, value in (arguments or {}).items():
        kind = (properties.get(key) or {}).get('type')
        if value is None or kind is None:
            coerced[key] = value
            continue
        try:
            coerced[key] = _as(kind, value)
        except (TypeError, ValueError):
            raise ValueError('%s: %s should be %s, got %r'
                             % (name, key, kind, value))
    return coerced


def _boolean(value):
    """A flag, from a bool or from the words a model spells one with."""
    if isinstance(value, str):
        return value.strip().lower() in TRUE
    return bool(value)


def _coercer(kind):
    """How a declared type takes a value; anything else passes as it came."""
    return {'array': _names, 'boolean': _boolean,
            'integer': lambda value: int(float(value)),
            'number': float}.get(kind, lambda same: same)


def _as(kind, value):
    """One value as one declared type. Raises for anything that will not go."""
    if kind not in ('array', 'boolean') and isinstance(value, bool):
        # A bool is an int in python, and a model that sent one where a number
        # belongs has made a mistake worth reporting.
        raise ValueError('not a number')
    return _coercer(kind)(value)


def _names(wanted):
    """Whatever a model spelled a list as, as a list of names."""
    if isinstance(wanted, str):
        wanted = wanted.strip().strip('[]').split(',')
    quotes = '"' + "'"
    return [str(item).strip().strip(quotes) for item in wanted if str(item).strip()]
