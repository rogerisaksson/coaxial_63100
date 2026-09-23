"""`python -m terminal`: the loader, guarded so a crew's spawned worker
importing this module runs nothing."""
import sys

from .loader import main

if __name__ == '__main__':
    sys.exit(main())
