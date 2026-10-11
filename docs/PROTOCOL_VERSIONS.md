# Protocol versions

The MINOR history of the wire (docs/PROTOCOL.md, `comms/inc/cmd.h`): MINOR
appends; MAJOR breaks a codec.

| MINOR | Change |
| --- | --- |
| 1 | gate op 10 alternate |
| 2 | device 10 DRIVE; the DC link appended to gate op 0 |
| 3 | a DAQ record ends with `u16 count`; accumulate 0 closes on the clock; op 5 says the stride |
| 4 | DAQ op 0 appends the buffer level, capacity and high-water mark |
| 5 | DAQ op 4 appends the backlog |
| 6 | IMU op 8 appends the three vectors, each with its own `have` |
| 7 | DAQ op 1 appends a sensor mask; records append four i16 per sensor after the pins; op 5 appends the rows |
| 8 | gate op 2 takes an optional period count; op 0 appends `periods_left` |
| 9 | fixed-shape requests dispatch on their own CRC, not t3.5 |
| 10 | drive op 14, the back-EMF observer chain |
| 11 | thermal budget appends the derate, the soak joules and the effective duty |
| 12 | thermal budget appends the winding - estimate, spend, own factor; thermal op 6 sets its envelope |
| 13 | twenty thermal nodes, the count says so; op 0 appends the FET junction rises and the speed; ops 7, 8, 9 read the node table, the edge table, set an edge |
| 14 | thermal op 10 reads the online identification, op 11 resets it |
| 15 | thermal op 10 appends the room as identified |
| 16 | thermal op 10 appends the margin floor, op 12 sets it; nothing identified is kept |
| 17 | thermal op 10 appends the trip cap as it stands |
| 18 | device 11 BOOT as the application serves it: op 10 `state`, op 12 `stay`, the rest refused in words (BOOT.md) |
| 19 | device 11 `state` appends the image verified and run and assign's flags; `seal` takes `[u8 flags]` (BOOT.md) |
| 20 | device 12 CTRL, the board's loop: slots, a wire, rows streamed and held; the `dec` wire type |
| 21 | link op 0 on the port carrying the request refused in words, `u8 0, str`, where it answered ILLEGAL DATA VALUE |
| 22 | thermal op 13 sets the observer's clock; the state's `seconds` and `seen_ms_ago` on it |
| 23 | gate drivers op 0 appends `u8 nfault, i32 vgate_mv`: the STO chain's FAULTOUT on PE15 and the drivers' supply |
| 24 | drive op 0 appends `i32 turns`: the estimate unwrapped, theta_hat + 2 pi turns |
| 25 | thermal op 14, WEP: the derate held off |
| 26 | thermal op 15 the application, op 16 the airspeed |
| 27 | gate op 10 a period count |
| 28 | angle op 2 appends `u32 crc_errors` |
| 29 | 0x6B appends `u16 locked` |
| 30 | thermal op 15 takes 7, a joint's holder |
| 31 | device 13 WBC |
| 32 | thermal op 4 appends `i32 hold_milliamps`: the clamp held to what the envelope leaves |

MAJOR 2: thermal nodes went per leg, indices repurposed.
A host ignores fields past what it knows. `test_conformance.py` holds a
live board to this document.
