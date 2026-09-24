"""What a prompt costs: the tool list and its detail, the trim to the window, the meter."""
import json
from typing import Any

from . import context
from . import language
from . import tools as toolmod
from .context import approx_tokens
from .sandbox import clip
from coaxial_mcp import detail
from coaxial_ollama.words import (BUILD_FIRMWARE_HINT, BUILD_HINT, DOCS_HINT, LINK_DIAGNOSE_HINT,
                                  SETS, SYSTEM)


class ChatBudget:

    """The prompt held inside the model's window, and what each turn cost."""

    # What the class this mixes into brings.
    budget: Any
    client: Any
    history: Any
    intent: Any
    keep: Any
    prompt_history: Any
    toolbox: Any
    turn_cost: Any

    def set_tools(self, wanted):
        names = SETS.get(wanted)
        if names is None:
            names = tuple(n.strip() for n in str(wanted).split(',') if n.strip())
        known = {spec['name'] for spec in toolmod.TOOLS}
        unknown = [n for n in names if n not in known]
        if unknown:
            raise ValueError('no such tool: %s. Sets: %s'
                             % (', '.join(unknown), ', '.join(SETS)))
        self.tool_names = names
        self.schemas = toolmod.schemas(
            [spec for spec in toolmod.TOOLS if spec['name'] in names],
            self.detail) or None
        return names

    def set_detail(self, wanted=detail.AUTO):
        """How much of each tool's documentation this model gets."""
        self.detail = detail.resolve(wanted, model=self.client.model,
                                     default=detail.TERSE)
        self.toolbox.detail = self.detail
        if self.tool_names:
            self.set_tools(','.join(self.tool_names) or 'none')
        return self.detail

    def _model_tag(self):
        """The tag exactly as ollama runs it - `unknown` on a chat built
        without a client."""
        return self.client.model if self.client is not None else 'unknown'

    def tool_cost(self):
        """What the tool list alone costs, every turn, before any question."""
        return approx_tokens(json.dumps(self.schemas or []))

    def _lock_language(self, asked):
        """Move the session language, if this question moves it."""
        current = self.language
        requested = language.requested_language(asked)
        detected = language.detect(asked)
        if requested and requested != current:
            current = requested
        elif detected and detected != current:
            current = detected
        self.language = current

    def trim(self):
        """System prompt, stubbed history, recent turns whole."""
        head = self.history[:-self.keep] if self.keep else self.history
        tail = self.history[-self.keep:] if self.keep else []

        # The language is named here, not worked out by the model: told to do
        # it itself, qwen2.5:14b answered a European question in Chinese.
        prompt_history = self.prompt_history
        asked = (prompt_history[-1] if prompt_history else
                 next((m.get('content') or '' for m in reversed(self.history)
                       if m['role'] == 'user'), ''))
        self._lock_language(asked)
        names = self.tool_names
        hint = ''
        if 'build_firmware' in names:
            hint += '\n' + BUILD_FIRMWARE_HINT
        if 'run_command' in names:
            hint += '\n' + BUILD_HINT
        if 'link_diagnose' in names:
            hint += '\n' + LINK_DIAGNOSE_HINT
        if 'docs' in names:
            hint += '\n' + DOCS_HINT
        # The earlier questions, not the wiped history: "tabulate", then "why
        # can you not reach the board", then "tried that, still nothing" only
        # reads as a sequence with them in view.
        hint += self.intent
        prior = self.prompt_history[-6:-1]
        if prior:
            hint += ('\nEarlier this session, in order: %s. Treat these as '
                     'troubleshooting steps already tried in this '
                     'conversation, not separate unrelated questions.'
                     % '; '.join('"%s"' % clip(q, 60) for q in prior))
        # Which model this is, from the tag the daemon was actually asked for
        # rather than from whatever the weights remember being called.
        who = ('Your model tag is exactly "%s", run locally by ollama on this '
               'bench; give that tag verbatim if asked which model you are.'
               % self._model_tag())
        if 'build_firmware' in names:
            # Said as identity, not only as the instruction BUILD_FIRMWARE_HINT
            # carries: "what am I" and "what do I do when asked to flash" are
            # different questions, and the second hint never answered the
            # first.
            who += (' You are this board\'s build system too: you compile its'
                    ' firmware and program it over SWD yourself.')
        sent = [{'role': 'system',
                 'content': SYSTEM + '\n' + who + hint + '\n'
                           + language.instruction_for(self.language)}]
        for message in head:
            content = (message.get('content') or '').strip()
            if message['role'] == 'tool':
                first = content.splitlines()[0] if content else ''
                sent.append({'role': 'tool',
                             'content': clip(first, 80) + ' [...]'})
            else:
                # Assistant tool_calls are dropped along with the results they
                # go with: a call whose answer has been stubbed is noise.
                sent.append({'role': message['role'],
                             'content': clip(content, 200)})
        sent.extend(tail)
        return self._fit(sent)

    def prompt_budget(self):
        """Tokens the next prompt may take, of the window this model has."""
        return context.budget_for(None if self.client is None
                                  else self.client.options)

    def _fit(self, sent):
        """Whatever trim() decided to send, cut down to what the model can
        actually be handed.
        """
        return context.fit(sent, self.prompt_budget(), self.tool_cost())

    def context_cost(self):
        return context.cost(self.trim(), self.tool_cost())

    def over_budget(self):
        usage = self.client.usage()
        return bool(self.budget) and \
            usage['prompt_tokens'] + usage['eval_tokens'] >= self.budget

    def cost_line(self):
        usage = self.client.usage()
        total = usage['prompt_tokens'] + usage['eval_tokens']
        text = '%d calls, %d in + %d out = %d tok' % (
            usage['calls'], usage['prompt_tokens'], usage['eval_tokens'], total)
        return text + (' of %d' % self.budget if self.budget else '')

    def _meter(self, prompt_tokens, eval_tokens):
        """Record the cost of one turn. Not printed - see /cost and /ctx."""
        self.turn_cost.append((prompt_tokens, eval_tokens))
