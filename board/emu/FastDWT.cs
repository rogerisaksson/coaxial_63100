// FastDWT.cs - The Cortex-M DWT's cycle counter off the instructions the CPU has executed, at the
// CPU's own rate, so it runs with virtual time without syncing it: Renode's DWT syncs virtual
// time on every CYCCNT read, and the firmware reads it three times a main-loop pass - the
// emulator's bottleneck (2026-09-25). It counts at `clock`'s frequency, the core's as the RCC
// sets it - the bootloader's 160 MHz, the app's 475 - taken up on the first read after a
// change: at a fixed 475 the bootloader's t1.5 was 253 us, and a frame split across a quantum
// was lost (2026-09-25). CYCCNTENA and CYCCNT only; the rest reads zero.

using System.Linq;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class FastDWT : IDoubleWordPeripheral, IKnownSize
    {
        public FastDWT(IMachine machine, IHasFrequency clock)
        {
            this.machine = machine;
            this.clock = clock;
        }

        public long Size => 0x1000;

        public void Reset()
        {
            control = 0;
            held = 0;
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case Control:
                    return control;
                case CycleCounter:
                    return Counting ? Raw() + shift : held;
                default:
                    return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
                case Control:
                    var now = ReadDoubleWord(CycleCounter);
                    control = value;
                    Set(now);
                    break;
                case CycleCounter:
                    Set(value);
                    break;
            }
        }

        private bool Counting => (control & CycleCountEnable) != 0;

        /// <summary>The counter reads `value` from here on, counting or held.</summary>
        private void Set(uint value)
        {
            held = value;
            shift = value - Raw();
        }

        /// <summary>Cycles since reset: instructions at the CPU's rate, `clock`'s a second.</summary>
        private uint Raw()
        {
            if(cpu == null)
            {
                cpu = machine.SystemBus.GetCPUs().OfType<BaseCPU>().First();
            }
            var executed = cpu.ExecutedInstructions;
            if(clock.Frequency != rate)
            {
                cycles += Cycles(executed);
                since = executed;
                rate = clock.Frequency;
            }
            return (uint)(cycles + Cycles(executed));
        }

        private ulong Cycles(ulong executed)
        {
            return (ulong)((double)(executed - since) * rate / (cpu.PerformanceInMips * 1e6));
        }

        private readonly IMachine machine;
        private readonly IHasFrequency clock;
        private BaseCPU cpu;
        private ulong rate;
        private ulong cycles;
        private ulong since;
        private uint control;
        private uint held;
        private uint shift;

        private const long Control = 0x0;
        private const long CycleCounter = 0x4;
        private const uint CycleCountEnable = 1U;
    }
}
