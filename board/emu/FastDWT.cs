// FastDWT.cs - The Cortex-M DWT's cycle counter off the instructions the CPU has executed, at the
// CPU's own rate, so it runs with virtual time without syncing it: Renode's DWT syncs virtual
// time on every CYCCNT read, and the firmware reads it three times a main-loop pass - the
// emulator's bottleneck (2026-09-25). CYCCNTENA and CYCCNT only; the rest reads zero.

using System.Linq;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class FastDWT : IDoubleWordPeripheral, IKnownSize
    {
        public FastDWT(IMachine machine, ulong frequency)
        {
            this.machine = machine;
            this.frequency = frequency;
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

        /// <summary>Cycles since reset: instructions at the CPU's rate, `frequency` a second.</summary>
        private uint Raw()
        {
            if(cpu == null)
            {
                cpu = machine.SystemBus.GetCPUs().OfType<BaseCPU>().First();
            }
            return (uint)(cpu.ExecutedInstructions * frequency / (cpu.PerformanceInMips * 1000000UL));
        }

        private readonly IMachine machine;
        private readonly ulong frequency;
        private BaseCPU cpu;
        private uint control;
        private uint held;
        private uint shift;

        private const long Control = 0x0;
        private const long CycleCounter = 0x4;
        private const uint CycleCountEnable = 1U;
    }
}
