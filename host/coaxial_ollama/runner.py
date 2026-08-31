"""The loop: one plan step at a time, the model driving, Python judging.

Shape of a step:

    plan step -> fresh conversation -> tool calls against the board -> report
                                                                         |
                                     Limit.judge in Python <-------------+

Each step starts a new conversation. It costs the system prompt again and buys
more: a model confused in step 3 arrives clean at step 4, and a verdict's
reasoning is one contiguous block an auditor can read. The Python namespace does
persist - state the model built deliberately survives, state it drifted into
does not.

What this loop refuses:

  * a verdict from the model. It reports a number; `plan.Limit` decides.
  * an endless step. `max_turns` ends it, recorded unfinished, not dropped.
  * hiding anything. Every message, call and result reaches the JSONL
    transcript before the next turn, so a run that dies mid-plan leaves its
    evidence on disk.

The system prompt is the other half of this file: mostly the board's invariants
restated as things the model may not conclude.
"""
import json
import os
import time

from. import context
from . import tools as toolmod

SYSTEM = """You are driving a hardware test bench from a written test plan.

The board is a coaxial BLDC inverter, an STM32H753 measurement platform: three
differential phase-sense channels, a DC link sense, an NTC. There is no PWM and
no motor drive in this firmware - it measures, it does not spin anything.

How to work:
- Use the board tools. They are cheaper and more reliable than writing code, so
  reach for run_python only when no tool does the job: statistics over a burst,
  a sweep, a comparison across channels.
- Call board_info once if you need the channel map. It does not change.
- The docs tool holds this board's own documents. HARDWARE says what a channel
  is and what it is behind; FINDINGS records what has already been ruled out,
  so a surprise you are about to investigate may already have an answer there.
  Prefer what they say to what you remember about hardware in general.
- Analog reads need the front end on: call afe_power {"action":"on"} first. With
  it off every channel reads exact mid-scale and the NTC reports exactly
  25.00 C. That number is not a measurement, and reporting it as one is the
  single worst mistake you can make here.
- Finish every step with exactly one report call.

What you must not do:
- Do not judge. You are never told the limits, because you are not the judge:
  you report the number and its unit, and the plan file decides pass or fail.
  Never say a value is good, in spec, nominal or acceptable.
- Do not invent a number. If a measurement did not happen, report with no value
  and say in the note what stopped you. An honest gap is a useful result; a
  plausible guess is a wrong verdict that nobody catches. A tool call that
  errors is a measurement that did not happen too - do not report an earlier
  turn's reading in its place, even if it looks close enough.
- Do not report a sensed current or a phase voltage at the motor. The phase
  channels sit behind analog front end gain that neither you nor the host knows.
  Volts at the ADC pin is as far as the data goes.
- Do not retry a call the operator declined.
- Do not use a pipe table or markdown in the note. This is a plain terminal;
  a table wider than it or cut off mid-row reads as garbage, not data.
"""


class Transcript:
    """Append-only JSONL, flushed every line.

    Flushing per line rather than per step is the whole reason this is JSONL: a
    run that dies with the fixture half configured should still say what the last
    thing it did was.
    """

    def __init__(self, path=None):
        self.path = path
        self.handle = open(path, 'w', encoding='utf-8') if path else None
        self.events = []

    def write(self, kind, **fields):
        event = dict(kind=kind, t=round(time.time(), 3), **fields)
        self.events.append(event)
        if self.handle:
            self.handle.write(json.dumps(event, default=str) + '\n')
            self.handle.flush()
        return event

    def close(self):
        if self.handle:
            self.handle.close()
            self.handle = None


class Record:
    """The outcome of one step. This is what a report is built from."""

    def __init__(self, task):
        self.task = task
        self.verdict = 'error'
        self.value = None
        self.unit = ''
        self.note = ''
        self.turns = 0
        self.calls = []
        self.seconds = 0.0
        self.warnings = []

    def as_dict(self):
        return {
            'id': self.task.id, 'name': self.task.name,
            'verdict': self.verdict, 'value': self.value, 'unit': self.unit,
            'limit': self.task.limit.describe() if self.task.limit else None,
            'note': self.note, 'turns': self.turns, 'calls': self.calls,
            'seconds': round(self.seconds, 2), 'warnings': self.warnings,
        }

    def line(self):
        value = '-' if self.value is None else '%g' % self.value
        return '%-9s %10s %-10s %s' % (self.verdict.upper(), value, self.unit,
                                       self.task.name[:40])


def _preview(args):
    """One short line of what a call was asked to do."""
    if not args:
        return ''
    text = ' '.join('%s=%s' % (k, v) for k, v in args.items())
    text = ' '.join(text.split())
    return text if len(text) <= 76 else text[:73] + '...'


class Runner:
    """One conversation per plan step, a JSONL transcript, and a verdict
    that comes from plan.Limit - never from the model."""
    def __init__(self, plan, client, toolbox, transcript=None, echo=True):
        self.plan = plan
        self.client = client
        self.toolbox = toolbox
        self.transcript = transcript or Transcript()
        self.echo = echo
        self.records = []

    # ---- one step ----------------------------------------------------------

    def prompt_budget(self):
        """What a step's conversation may grow to, from the client's own
        num_ctx. Read fresh every turn rather than cached at construction:
        the client halves its window when the machine runs out of memory
        (see client._make_room), and a runner still trimming to the old
        number would hand the daemon exactly the prompt that just failed."""
        return context.budget_for(getattr(self.client, 'options', None))

    def system_prompt(self):
        text = SYSTEM
        if self.plan.context:
            text += '\nAbout this bench:\n' + self.plan.context.strip() + '\n'
        return text

    def _refuse_unmeasured(self, task, record, messages, name, args, st):
        """A report with no board tool behind it, refused like a prose
        stop: nudged twice, then unfinished. True when the step is over.

        The same failure debug.py hardened against, here with higher
        stakes: this report becomes a signed pass/fail, and SYSTEM already
        says not to invent a number - which did not stop the model in
        debug.py either.
        """
        st['nudges'] += 1
        text = ('refused: no board tool was called this step, '
                'so there is nothing behind that value. '
                'Measure something, or call report with no '
                'value and say what stopped you.')
        self._say('   %-12s %s' % (name, text))
        self.transcript.write('tool', id=task.id, turn=record.turns,
                              name=name, args=args, result=text)
        if st['nudges'] > 2:
            record.note = text
            record.verdict = 'unfinished'
            record.warnings.append(
                'called report with no board measurement this step')
            return True
        messages.append({'role': 'tool', 'tool_name': name, 'name': name,
                         'content': '%s: %s' % (name, text)})
        return False

    def _take_call(self, task, record, messages, call, st):
        """One tool call of a turn: the Reported that ends the step,
        'give_up', or None to go on."""
        name = (call.get('function') or {}).get('name', '?')
        args = toolmod.arguments(call)
        result = self.toolbox.call(name, args)
        record.calls.append(name)
        if name in toolmod.LINK_TOOLS:
            st['board_touched'] = True

        if isinstance(result, toolmod.Reported):
            if not st['board_touched']:
                over = self._refuse_unmeasured(task, record, messages, name,
                                               args, st)
                return 'give_up' if over else None
            self.transcript.write('report', id=task.id, value=result.value,
                                  unit=result.unit, note=result.note)
            return result

        self._say('   %-12s %s' % (name, _preview(args)))
        self.transcript.write('tool', id=task.id, turn=record.turns,
                              name=name, args=args, result=result)
        messages.append({'role': 'tool', 'tool_name': name, 'name': name,
                         'content': '%s: %s' % (name, result)})
        return None

    def run_task(self, task):
        record = Record(task)
        started = time.time()
        self.transcript.write('step_begin', id=task.id, name=task.name,
                              ask=task.ask)
        self._say('\n== %s  %s' % (task.id, task.name))

        messages = [{'role': 'system', 'content': self.system_prompt()},
                    {'role': 'user', 'content': task.brief()}]
        schemas = self.toolbox.schemas()
        # nudges: prose answers and unmeasured reports, two of either and
        # the step is unfinished; board_touched: a real board tool was
        # called this step.
        st = {'nudges': 0, 'board_touched': False}
        reported = None
        giving_up = False

        # The tool schemas ride with every turn and come out of the same
        # window the conversation does, so they are counted as part of it.
        schema_tokens = context.approx_tokens(json.dumps(schemas))

        while record.turns < task.max_turns and reported is None:
            record.turns += 1
            # A step is a fresh conversation but not a short one: max_turns
            # tool calls with a build log or a hundred sample rows in each is
            # a prompt past num_ctx by turn four, and past num_ctx is where
            # the daemon starts answering 500 instead of answering. The
            # transcript above already has every result whole - this only
            # bounds what is re-sent.
            messages = context.fit(messages, self.prompt_budget(),
                                   schema_tokens)
            message = self.client.chat(messages, schemas)

            # Thinking is logged but not fed back: it is the largest thing in a
            # reasoning model's reply and the least useful to the next turn.
            thinking = message.pop('thinking', None)
            self.transcript.write('assistant', id=task.id, turn=record.turns,
                                  content=message.get('content', ''),
                                  thinking=thinking,
                                  tool_calls=message.get('tool_calls'))
            messages.append(message)

            calls = message.get('tool_calls') or []
            if not calls:
                text = (message.get('content') or '').strip()
                self._say('   ...%s' % text[:100].replace('\n', ' '))
                st['nudges'] += 1
                if st['nudges'] > 2:
                    record.note = text
                    record.verdict = 'unfinished'
                    record.warnings.append('stopped in prose, never reported')
                    break
                messages.append({
                    'role': 'user',
                    'content': 'Do not answer in prose. Either call a tool, or '
                               'call report to finish this step.'})
                continue

            for call in calls:
                got = self._take_call(task, record, messages, call, st)
                if got == 'give_up':
                    giving_up = True
                    break
                if got is not None:
                    reported = got
                    break

            if giving_up:
                break

        record.seconds = time.time() - started
        if reported is not None:
            self._judge(record, reported)
        elif record.verdict != 'unfinished':
            record.verdict = 'unfinished'
            record.warnings.append('no report within %d turns' % task.max_turns)

        self.transcript.write('step_end', **record.as_dict())
        self._say('   -> %s' % record.line())
        self.records.append(record)
        return record

    def _judge(self, record, reported):
        """The only place a verdict is produced, and the model is not in it."""
        task = record.task
        record.value = reported.value
        record.unit = reported.unit or task.unit
        record.note = reported.note

        if task.unit and reported.unit and \
                reported.unit.split()[0].lower() != task.unit.split()[0].lower():
            record.warnings.append('reported in %r, plan expects %r'
                                   % (reported.unit, task.unit))

        if task.record_only:
            record.verdict = 'record'
        elif reported.value is None:
            record.verdict = 'fail'
            record.warnings.append('a limited step needs a number; none reported')
        else:
            record.verdict = 'pass' if task.limit.judge(reported.value) else 'fail'

    # ---- the whole plan ----------------------------------------------------

    def run(self, only=None):
        self.transcript.write('run_begin', plan=self.plan.header(),
                              model=self.client.model,
                              allow_writes=self.toolbox.allow_writes)
        for task in self.plan.tasks:
            if only and task.id not in only:
                continue
            if task.needs_writes and not self.toolbox.allow_writes:
                # Skipped loudly. A step that silently did not run is the one
                # failure mode a test report must never have.
                record = Record(task)
                record.verdict = 'skipped'
                record.warnings.append('step needs --allow-writes')
                self.records.append(record)
                self.transcript.write('step_end', **record.as_dict())
                self._say('\n== %s  %s\n   -> SKIPPED (needs --allow-writes)'
                          % (task.id, task.name))
                continue
            self.run_task(task)

        summary = self.summary()
        self.transcript.write('run_end', counts=summary['counts'],
                              usage=summary['usage'])
        return summary

    def summary(self):
        counts = {}
        for record in self.records:
            counts[record.verdict] = counts.get(record.verdict, 0) + 1
        return {
            'plan': self.plan.header(),
            'model': self.client.model,
            'usage': self.client.usage(),
            'counts': counts,
            'records': [r.as_dict() for r in self.records],
        }

    def _say(self, text):
        if self.echo:
            print(text, flush=True)


def report_text(summary):
    """The table an operator reads, and the one that goes beside the log."""
    head = summary['plan']
    lines = [
        '',
        '%s  %s  plan %s' % (head['product'], head['pcba_revision'],
                             head['plan_version']),
        'plan file  %s' % head['plan_file'],
        'study      %s' % head['measurement_system_study'],
        'model      %s' % summary['model'],
        '',
        '%-6s %-9s %10s %-10s %s' % ('step', 'verdict', 'value', 'unit', 'name'),
        '-' * 78,
    ]
    for record in summary['records']:
        value = '-' if record['value'] is None else '%g' % record['value']
        lines.append('%-6s %-9s %10s %-10s %s'
                     % (record['id'], record['verdict'].upper(), value,
                        record['unit'], record['name'][:40]))
        if record['limit']:
            lines.append('%17s limit %s' % ('', record['limit']))
        for warning in record['warnings']:
            lines.append('%17s ! %s' % ('', warning))

    usage = summary['usage']
    lines += ['-' * 78,
              '  '.join('%s %d' % (k, v)
                        for k, v in sorted(summary['counts'].items())),
              'ollama: %d calls, %d prompt + %d generated tokens'
              % (usage['calls'], usage['prompt_tokens'], usage['eval_tokens']),
              '']
    return '\n'.join(lines)


def default_transcript_path(directory='data'):
    """Named by the clock, so a run never overwrites the evidence of another."""
    os.makedirs(directory, exist_ok=True)
    return os.path.join(directory,
                        'ollama-run-%s.jsonl' % time.strftime('%Y%m%d-%H%M%S'))
