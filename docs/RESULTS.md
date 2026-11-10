# Results (written by host/tools/dev/cover.py, not by hand)

<!-- coverage -->
Generated at `647aea1` by `python host/tools/dev/cover.py --readme`:
one offline run (`host/tools/dev/run_tests.py --offline`), the Python under
coverage.py, the C under gcov. ST's generated code - CubeMX's core/, startup and
cmake/stm32cubemx/, the HAL - is not counted.

| Code | Lines | Covered |
| --- | ---: | ---: |
| **Host**: host/'s Python, the emulation's C | 36162 | **76.5 %** |
| **Target**: the firmware's C, the target-only files uncovered | 9618 | **72.5 %** |

3866 checks in 32 suites: 3866 passed, 0 failed; 323 skipped, a board's.
<!-- /coverage -->
