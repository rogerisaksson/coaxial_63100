"""The commit her recordings are made at: HEAD of the checkout their folder sits in (`head`)."""
import os
import subprocess

from machine.process import introspect

#: The digits of a commit a recording's name carries.
SHORT = 8


def head():
    """HEAD of the checkout `introspect.RECORDINGS` sits in, as `git rev-parse` gives it; None without git or
    outside a repository. Asked each time, never kept: a page's session outlives commits."""
    try:
        run = subprocess.run(['git', 'rev-parse', 'HEAD'], cwd=os.path.dirname(introspect.RECORDINGS),
                             capture_output=True, text=True)
    except OSError:
        return None
    return (run.stdout.strip() if run.returncode == 0 else '') or None
