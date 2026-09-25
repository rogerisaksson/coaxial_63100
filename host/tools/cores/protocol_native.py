"""pyserial's name for the `native://` scheme: tools.cores.native's port, emulator://'s buses."""
from tools.cores import native
from tools.emu import protocol_emulator

Serial = native.Serial
buses = protocol_emulator.buses
