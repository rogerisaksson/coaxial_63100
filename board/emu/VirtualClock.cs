// VirtualClock.cs - A machine's virtual time, s, read without syncing it: the instructions since an
// anchor at the CPU's rate, exact within a translated block. Anchored afresh on the CPU's
// TimeHandle - exact once synced - wherever time passed that executed nothing, or passed at
// another rate: a WFI, a skip, a change of MIPS, a reset. FastDWT's CYCCNT and TIM1's count run on
// it: counted off instructions alone, both stopped through a sleep or a skip (2026-09-25).

using System.Collections.Generic;
using System.Linq;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.CPU;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public sealed class VirtualClock
    {
        /// <summary>The machine's clock, made on first asking.</summary>
        public static VirtualClock Of(IMachine machine)
        {
            lock(clocks)
            {
                if(!clocks.TryGetValue(machine, out var clock))
                {
                    clock = new VirtualClock(machine);
                    clocks[machine] = clock;
                }
                return clock;
            }
        }

        /// <summary>Virtual seconds since the machine started.</summary>
        public double Seconds
        {
            get
            {
                if(cpu == null)
                {
                    cpu = machine.SystemBus.GetCPUs().OfType<TranslationCPU>().First();
                    cpu.AddHookAtWfiStateChange(_ => stale = true);
                    stale = true;
                }
                var executed = cpu.ExecutedInstructions;
                if(stale || executed < since || cpu.PerformanceInMips != mips
                   || cpu.SkippedInstructions != skipped)
                {
                    Anchor();
                    executed = since;
                }
                // An anchor lands up to a rounding's few instructions behind the count: held,
                // never run back.
                last = System.Math.Max(last, at + (executed - since) / (mips * 1e6));
                return last;
            }
        }

        private VirtualClock(IMachine machine)
        {
            this.machine = machine;
        }

        private void Anchor()
        {
            cpu.SyncTime();
            at = cpu.TimeHandle.TotalElapsedTime.Ticks / 1e9;
            since = cpu.ExecutedInstructions;
            mips = System.Math.Max(1U, cpu.PerformanceInMips);
            skipped = cpu.SkippedInstructions;
            stale = false;
        }

        private readonly IMachine machine;
        private TranslationCPU cpu;
        private volatile bool stale;
        private double at;
        private double last;
        private ulong since;
        private uint mips;
        private ulong skipped;

        private static readonly Dictionary<IMachine, VirtualClock> clocks =
            new Dictionary<IMachine, VirtualClock>();
    }
}
