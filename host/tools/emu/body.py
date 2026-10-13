#!/usr/bin/env python3
"""A multi-limb machine emulated across parallel Renode processes."""
import threading

from tools.emu.emulator import ELF, FAITHFUL_MIPS, Limb


class Body:
    """A machine's limbs, a Renode process each so they run on the host's cores side by side -
    one process's machines wait for each other every quantum, eight boards in one doing 5 a
    board's work. Each limb is its own RS485 segment, as on the machine:
    `urls[name]` is its bus. `limbs` {name: boards}, `worlds` {name: a world's name}."""

    def __init__(self, limbs, elf=ELF, worlds=None, mips: int | None = FAITHFUL_MIPS,
                 mpu=False):
        worlds = worlds or {}
        self.limbs = {name: Limb(n, elf, world=worlds.get(name), mips=mips, mpu=mpu)
                      for name, n in limbs.items()}
        self.urls = {name: limb.url for name, limb in self.limbs.items()}

    def start(self):
        failed = []

        def up(limb):
            try:
                limb.start()
            except RuntimeError as exc:
                failed.append(str(exc))

        threads = [threading.Thread(target=up, args=(limb,)) for limb in self.limbs.values()]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join()
        if failed:
            self.stop()
            raise RuntimeError('; '.join(failed))
        return self

    def stop(self):
        for limb in self.limbs.values():
            limb.stop()

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()
