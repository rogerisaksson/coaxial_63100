// FastDWT.cs - The Cortex-M DWT's cycle counter off the instructions the CPU has executed, at the
// CPU's own rate, so it runs with virtual time without syncing it: Renode's DWT syncs virtual
// time on every CYCCNT read, and the firmware reads it three times a main-loop pass - the
// emulator's bottleneck (2026-09-25). It counts at `clock`'s frequency, the core's as the RCC
// sets it - the bootloader's 160 MHz, the app's 475 - taken up on the first read after a
// change: at a fixed 475 the bootloader's t1.5 was 253 us, and a frame split across a quantum
// was lost (2026-09-25). A change of the CPU's MIPS rebases it too, and the first read after
// a WFI: the sleep executed nothing and took time, which the CPU's TimeHandle has - read in
// the hook, the waking handler's stamp came first and RTU frames split (2026-09-25). CYCCNTENA
// and CYCCNT only; the rest reads zero.

using System;
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
            anchored = false;
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

        /// <summary>Cycles since the first read, `clock`'s a second: the last anchor's count and
        /// the instructions since at the CPU's rate.</summary>
        private uint Raw()
        {
            if(cpu == null)
            {
                cpu = machine.SystemBus.GetCPUs().OfType<TranslationCPU>().First();
                cpu.AddHookAtWfiStateChange(_ => slept = true);
            }
            // A reset starts the CPU's count again: anchored afresh, nothing carried.
            if(!anchored || cpu.ExecutedInstructions < since)
            {
                anchored = false;
                Anchor();
            }
            if(slept || clock.Frequency != rate || cpu.PerformanceInMips != mips)
            {
                Anchor();
            }
            return (uint)(ulong)(cycles + Ran(cpu.ExecutedInstructions - since));
        }

        private double Ran(ulong instructions)
        {
            return (double)instructions * rate / (Math.Max(1U, mips) * 1e6);
        }

        /// <summary>The count carried to now at the old rate: the instructions since the last
        /// anchor and the virtual time they leave over, a sleep.</summary>
        private void Anchor()
        {
            cpu.SyncTime();
            var executed = cpu.ExecutedInstructions;
            var now = cpu.TimeHandle.TotalElapsedTime.Ticks;
            if(anchored)
            {
                // Ticks are nanoseconds. Rounding leaves virtual time a few instructions either
                // side of the count; a shortfall is carried, never counted back.
                var asleep = (double)(now - at) - (executed - since) * 1e3 / Math.Max(1U, mips) - owed;
                owed = Math.Max(0.0, -asleep);
                cycles += Ran(executed - since) + Math.Max(0.0, asleep) * rate / 1e9;
            }
            anchored = true;
            slept = false;
            at = now;
            since = executed;
            rate = clock.Frequency;
            mips = cpu.PerformanceInMips;
        }

        private readonly IMachine machine;
        private readonly IHasFrequency clock;
        private TranslationCPU cpu;
        private bool anchored;
        private volatile bool slept;
        private ulong at;
        private double owed;
        private ulong rate;
        private uint mips;
        private double cycles;
        private ulong since;
        private uint control;
        private uint held;
        private uint shift;

        private const long Control = 0x0;
        private const long CycleCounter = 0x4;
        private const uint CycleCountEnable = 1U;
    }
}
