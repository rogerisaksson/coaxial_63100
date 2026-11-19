"""Command-line scripts by function: dev, target, bench, thermal, render, sim, cores, notebooks."""
import os

#: host/ and the checkout it sits in.
HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(HOST)

from .dev import arithmetic as _arithmetic  # noqa: E402 - the kernels held before any numpy loads OpenBLAS

_arithmetic.hold()
