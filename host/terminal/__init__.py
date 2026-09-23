"""The terminal: the front page and the live views, as one process.

What lies under `pages/` is the terminal - one module per page, each
saying what it is and how it runs - and `loader` reads that folder,
lists the pages on the front page, preloads the model into memory and
runs the picked page here. `python -m terminal` starts it;
coaxial_tty.ps1 is the shortcut in front of that.
"""
