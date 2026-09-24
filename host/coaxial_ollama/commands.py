"""The prompt loop's slash commands, and the switches they make: model, node, board."""
from contextlib import suppress

from . import language
from .capability import choose, probe
from .client import FAULTS, Ollama, OllamaError
from .sandbox import clip
from coaxial.comm import ports, session as sessionmod
from coaxial.errors import RigError
from coaxial.simulated import bus_nodes
from coaxial_mcp import detail
from coaxial_ollama.words import BUILDS, HELP, ROLE
from tools.target import find_board


def _without_side(where):
    """The joint without its side, because the bus already carries it: "RL 2
    knee", not "RL node 2 right knee".
    """
    for side in ('left ', 'right '):
        if where.startswith(side):
            return where[len(side):]
    return where


class ChatCommands:

    """What a line starting with / does."""

    origin: 'tuple[str, bool] | None' = None   # once a board was opened

    def command(self, line):
        """A slash command."""
        if not line.startswith('/'):
            return None
        verb, _, rest = line[1:].partition(' ')
        handler = self.COMMANDS.get(verb)
        if handler is None:
            return 'no such command. /help'
        return handler(self, rest.strip())

    def _cmd_quit(self, rest):
        raise SystemExit(0)

    def _cmd_help(self, rest):
        """Live, not a fixed string: what it can do depends on the tool set
        this session started with, what it costs on the detail level."""
        lines = [ROLE]
        if {'build_firmware', 'run_command'} & set(self.tool_names):
            lines.append(BUILDS)
        lines.append('%s, %s, %d tok/turn: %s'
                     % (self._model_tag(), self.detail,
                        self.tool_cost(),
                        ', '.join(self.tool_names) or 'no tools'))
        return '\n'.join(lines + [HELP])

    def _cmd_py(self, rest):
        return self.toolbox.call('run_python', {'code': rest})

    def _cmd_sh(self, rest):
        return self.toolbox.call('run_command', {'cmd': rest})

    def _cmd_reconnect(self, rest):
        return self._reconnect()

    def _cmd_clear(self, rest):
        self.history = []
        return 'context cleared'

    def _cmd_tools(self, rest):
        if rest:
            self.set_tools(rest)
        return '%s (%d tok/turn)' % (', '.join(self.tool_names) or 'none',
                                     self.tool_cost())

    def _cmd_detail(self, rest):
        """Priced, not just named: the whole point of the level is what the
        tool list costs per turn, and that number is the argument for
        changing it."""
        if rest and rest.lower() not in detail.LEVELS:
            return 'detail: %s, or auto' % ', '.join((detail.TERSE, detail.FULL))
        if rest:
            self.set_detail(rest.lower())
        return 'detail: %s (%d tok/turn of tools)' % (self.detail,
                                                      self.tool_cost())

    def _cmd_confirm(self, rest):
        """/tools build alone hands the model run_command with nothing
        asking first, unless --confirm was already on the command line
        that started this session - this is the other half of that
        switch, reachable without a restart either.
        """
        from .cli import ask_operator     # cli imports Chat: not at top
        self.toolbox.confirm = (None if self.toolbox.confirm
                                else ask_operator)
        return 'confirm: %s' % ('on - asks before every write'
                                if self.toolbox.confirm else 'off')

    def _cmd_lang(self, rest):
        if not rest:
            return ('session language: %s' % self.language
                    if self.language else
                    'not locked yet - mirroring each question')
        if rest.lower() in ('auto', 'off'):
            self.language = None
            return 'language: unlocked - back to mirroring each question'
        named = (language._NAME_TO_LANGUAGE.get(rest.lower())
                 or (rest.title() if rest.title() in language.LANGUAGE_NAMES
                     else None))
        if named is None:
            return ("don't know %r - try an English language name, or "
                    "/lang auto to unlock" % rest)
        self.language = named
        return 'language: %s (locked)' % named

    def _cmd_ctx(self, rest):
        """The budget is the number that explains the other two once a
        conversation gets long: a turn that is not growing any more is a
        turn being trimmed to fit, not a turn that stopped costing."""
        budget = self.prompt_budget()
        return '%d messages, next turn about %d tok in%s, %d of it tools' \
            % (len(self.history), self.context_cost(),
               ' of %d' % budget if budget else '', self.tool_cost())

    def _cmd_cost(self, rest):
        return self.cost_line()

    def _cmd_history(self, rest):
        if not self.prompt_history:
            return 'nothing asked yet this session'
        return '\n'.join('%d. %s' % (i, clip(q, 100))
                         for i, q in enumerate(self.prompt_history, 1))

    def _cmd_clear_history(self, rest):
        n = len(self.prompt_history)
        self.prompt_history = ()
        return 'prompt history cleared (%d question%s)' \
            % (n, '' if n == 1 else 's')

    #: The slash commands, by the word after the slash. Each takes the rest
    #: of the line; a word not here is answered '/help'.
    COMMANDS = {
        'q': _cmd_quit, 'quit': _cmd_quit, 'exit': _cmd_quit,
        'help': _cmd_help, '?': _cmd_help,
        'py': _cmd_py, 'sh': _cmd_sh, 'reconnect': _cmd_reconnect,
        'model': lambda self, rest: self._switch_model(rest),
        'board': lambda self, rest: self._switch_board(rest),
        'node': lambda self, rest: self._switch_node(rest),
        'clear': _cmd_clear, 'tools': _cmd_tools, 'detail': _cmd_detail,
        'confirm': _cmd_confirm, 'lang': _cmd_lang, 'ctx': _cmd_ctx,
        'cost': _cmd_cost, 'history': _cmd_history,
        'clear_history': _cmd_clear_history,
    }

    def _switch_model(self, rest):
        """Run this session on another tag, without restarting it."""

        tag, extra = rest.strip(), {}
        if not tag:
            try:
                have = ', '.join(self.client.models())
            except OllamaError as exc:
                have = str(exc)
            return 'model: %s (%d tok window). Available: %s' % (
                self.client.model, self.client.options.get('num_ctx', 0), have)
        if tag == 'auto':
            picked = choose(probe())
            tag, extra = picked.tag, dict(picked.options or {})
        if tag == self.client.model and not extra:
            return 'model: %s already' % tag

        old = self.client
        fresh = Ollama(tag, host=old.host, remote_ok=old.remote_ok,
                       keep_alive=old.keep_alive, think=old.think,
                       fmt=old.fmt, timeout=old.timeout)
        fresh.options = dict(old.options)
        fresh.options.update(extra)
        try:
            # Its own error text names the tag and how to pull it.
            fresh.require_model()
        except OllamaError as exc:
            return str(exc)

        try:
            old.unload()
        except FAULTS as exc:
            # Not fatal, and not silent: the session still works, the card is
            # just holding weights nobody is using until keep_alive expires.
            self.io_log.write('  ! could not unload %s: %s%s'
                              % (old.model, exc, '\n'))
        self.client = fresh
        self.history = []
        self.set_detail(self.detail)
        return 'model: %s (was %s), context cleared' % (fresh.model, old.model)

    def _switch_node(self, rest):
        """Which node on the bus the tools talk to."""
        from coaxial_mcp import bus

        session = self.toolbox.session
        want = rest.strip()
        if not want:
            return bus.devices(session)
        if want.lower() in ('buses', 'bus'):
            return bus.devices(session, op='buses')
        # "LL 2" is a bus and a node, which is what a node id needs beside it
        # once there is more than one segment.
        parts = want.split()
        if len(parts) == 2 and parts[1].lstrip('-').isdigit():
            return bus.devices(session, op='use', bus=parts[0].upper(),
                               unit=int(parts[1]))
        if not want.lstrip('-').isdigit():
            return bus.devices(session, op='use', name=want)
        return bus.devices(session, op='use', unit=int(want))

    def prompt_tag(self):
        """(text, ok) for the prompt: the interface, then the node."""
        label, real = self.origin or (None, True)
        if label is None:
            return None, True
        session = self.toolbox.session
        unit = session.unit
        if unit is None:
            return label, real
        bus = session.bus
        where = None
        with suppress(Exception):
            if bus:
                where = (bus_nodes(bus).get(unit) or (None, None, None))[2]
        # The bus first, because with five segments a node number alone names
        # nothing: node 2 is a knee on two of them.
        where = _without_side(where or '')
        node = 'ALL NODES' if unit == 0 else (
            '%d %s' % (unit, where) if where else 'node %d' % unit)
        if bus:
            node = '%s %s' % (bus, node)
        return '%s, %s' % (label, node), ('all' if unit == 0 else real)

    def _switch_board(self, rest):
        """Point this session at another board, or at a simulated one."""

        want = rest.strip().lower()
        if not want:
            label = (self.origin or ('unknown',))[0]
            return ('board: %s. /board simulated | auto | rs485 | COM4'
                    % label)
        if want in ('sim', 'simulated', 'fake'):
            session, found = sessionmod.open_session(simulated=True)
        elif want == 'auto':
            session, found = sessionmod.open_session()
        elif want in ('rs485', 'serial'):
            # The field bus, not the bench cable: probes are excluded, or the
            # probe-first order hands back the one board that was just ruled
            # out.
            session, found = sessionmod.open_session(only=ports.SERIAL)
        elif want in ('probe', 'jtag', 'swd', 'debugger'):
            session, found = sessionmod.open_session(only=ports.PROBE)
        else:
            session, found = sessionmod.open_session(rest.strip())

        wanted_real = want not in ('sim', 'simulated', 'fake')
        if wanted_real and not found.real:
            # The search found nothing.
            with suppress(Exception):
                session.close()
            seen = ', '.join(find_board.list_ports())
            here = (self.origin or ('unknown',))[0]
            return ('board: nothing answered on %s - still on %s'
                    % (seen or 'no COM port at all', here))

        previous = self.toolbox.session
        if previous is not session:
            with suppress(Exception):
                previous.close()
        self.toolbox.session = session
        self.origin = (found.label, found.real)
        self.link_ok = True
        # Readings remembered from the board just left are not this one's.
        self.last_channels = None
        return 'board: %s' % found.label

    def _reconnect(self):
        """Drop the link and try to reopen it - for a cable that was plugged
        back in without restarting this whole prompt loop.
        """
        session = self.toolbox.session
        session.reset()
        try:
            session.board
        except RigError as exc:
            self.link_ok = False
            return 'board: %s' % exc
        self.link_ok = True
        return 'board: link is up'
