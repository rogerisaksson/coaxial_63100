"""Command-line scripts by function: dev, target, bench, thermal, render, sim, cores, notebooks."""
import os

#: host/ and the checkout it sits in.
HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(HOST)
