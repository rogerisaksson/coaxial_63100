"""The humanoid page's playback: her newest state said, drawn as it comes.

    playback = Playback()
    playback.push(said)                       # the states her process said since the last frame
    now = playback.at(time.perf_counter())    # the newest state to draw, or None before the first
"""


class Playback:

    """Her newest state said, drawn as it comes without artificial lag: `push(states)`, `at(wall)`.
    Drawn as each state comes, a frame at 15 a second repeats a state when her process runs in
    slices slower than real time, trading interpolation lag for immediate response."""

    def __init__(self):
        self.states, self.shown, self.wall = [], None, None

    def push(self, states):
        if states:
            self.states = [states[-1]]

    def at(self, wall):
        """The state to draw at `wall` seconds, or None before the first."""
        if not self.states:
            return None
        self.wall = wall
        self.shown = self.states[-1]['t']
        return self.states[-1]
