"""wbc/'s library as this host builds it: its sources, headers and flags (`wbc.library`)."""
import os

from tools import REPO
from tools.cores import build

#: The robot's C law, its eyes' classes and wholes (wbc_classes.c, wbc_wholes.c), its harnesses, the bus it answers
#: on, and the SE(3) core's friction its world's contacts take (wbc_world.c: se3_rubs, se3_kept).
SOURCES = ([os.path.join(REPO, 'wbc', 'test', 'harness%s.c' % n) for n in ('', '_world', '_qp', '_classes', '_libm',
                                                                           '_borne', '_inekf')]
           + [os.path.join(REPO, 'wbc', 'src', 'wbc_%s.c' % n) for n in (
               'body stack solve wrench cone shrunk model pass pass_heat beat world bus imp latched poses stride poise swing floor surety touch twist wait anatomy bedexit scoot sensed brace blas qp qp_interior law step maps rounds patch sil rcu gmo gmo_factor soa chain chain_qp chain_rows chain_tasks borne hull upright inekf registry afford classes wholes libm').split()]
           + [os.path.join(REPO, 'modbus', 'src', 'modbus_%s.c' % n) for n in ('crc rtu slave').split()]
           + [os.path.join(REPO, 'se3', 'src', 'se3_law.c')])
INCLUDES = [os.path.join(REPO, *folder) for folder in (('wbc', 'inc'), ('modbus', 'inc'), ('se3', 'inc'), ('se3', 'src'),
                                                       ('comms', 'inc'))]
#: The firmware compiles the chain at -O2 -fno-math-errno (CMakeLists.txt); sin and cos called apart, as numpy calls
#: them: gcc's sincos for the pair is glibc's own path at 0.855 < |x| < 2.43, its sin's last bit not sin's. On this
#: host's own instruction sets (`build.native`), with no errno branch on sqrt and non-trapping comparisons (no fenv
#: anywhere): nothing contracted, a loop's elements four or eight at once are the same bits.
FLAGS = (('-O2', '-fno-math-errno', '-fno-trapping-math') + build.native()
         + ('-fno-builtin-sin', '-fno-builtin-cos') + build.mujoco_headers())
