// Coaxial63100_STO.cs - The STO chain in the emulator: world_sto.c in the world library, the
// circuit of electronic_simulations/sto/sto.asc. The master's common-mode pilot on A1/B1 (UART5's
// pair), U16C's pulse train into Cinj, U4, the keepalive's pump into Clevel off PA10's edges at
// their instants, Q10, U11 onto PE15 (nFAULT, TIM1's BKIN), U9's +15V7 for the gate drivers.
// The edges are kept with their instants and the chain stepped over them at StepHz, or as they
// fill the buffer: a step an edge cost 3.4 us, 50 000 a virtual second idle (2026-09-27). Cinj,
// Clevel and +15V7 are written into the front end. The pilot is the master's: PilotVolts 1.5,
// PilotHz 5000 (sto.asc's PAM8406), 0 V none. The chain runs on the part's time, so the
// keepalive's edges come as the part spaces them: main()'s instructions at PartMips, stretched
// by the interrupts' share of the part's budget, a sleep at its length. Renode's pacing, the
// idle core at 100 MIPS and main() skipped under the drive, stretched the edges in virtual time
// and starved the pump; the drive's ISR counted as the chain's time put them 46 us apart, the
// part's 9, and the observer's slice 850 us, the part's 190 (2026-09-27). Unpaced, it is the
// virtual time.

using System;
using System.Linq;
using System.Runtime.InteropServices;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Peripherals.Miscellaneous;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Analog
{
    public class Coaxial63100_STO : IGPIOReceiver
    {
        public Coaxial63100_STO(IMachine machine, Coaxial63100_AFE afe, Coaxial63100_TIM1 tim1)
        {
            this.machine = machine;
            this.afe = afe;
            this.tim1 = tim1;
            Faultout = new GPIO();
            steps = new LimitTimer(machine.ClockSource, StepHz, this, "sto", limit: 1,
                                   workMode: WorkMode.Periodic, eventEnabled: true);
            steps.LimitReached += Advance;
            steps.Enabled = true;
            Reset();
        }

        /// <summary>U11's Y, FAULTOUT: PE15 and TIM1's break input, high while the chain is
        /// released.</summary>
        public GPIO Faultout { get; }

        public void Reset()
        {
            pin = false;
            level = false;
            started = false;
            edges = 0;
            Show(new float[Outputs]);
        }

        /// <summary>PA10, KEEPALIVE: the edge's instant kept for the next step.</summary>
        public void OnGPIO(int number, bool value)
        {
            if(value == pin)
            {
                return;
            }
            pin = value;
            if(!started)
            {
                return;
            }
            var now = PartTime(Main());
            worstGap = Math.Max(worstGap, now - edgeAt);
            edgeAt = now;
            at[edges++] = (float)(now - steppedAt);
            if(edges == at.Length)
            {
                Advance();
            }
        }

        /// <summary>The keepalive's longest gap since the last read, us of the part's time: an
        /// open one as of the last step.</summary>
        public double WorstGap
        {
            get
            {
                var worst = Math.Max(worstGap, steppedAt - edgeAt) * 1e6;
                worstGap = 0.0;
                return worst;
            }
        }

        /// <summary>The world core's library (Coaxial63100_Plant's), loaded once a process.</summary>
        public void Library(string path)
        {
            if(native == IntPtr.Zero)
            {
                native = NativeLibrary.Load(path);
                stoReset = Export<StoReset>("emu_sto_reset");
                stoStep = Export<StoStep>("emu_sto_step");
            }
            started = false;
        }

        /// <summary>This board's chain in the library: its index among a process's boards.</summary>
        public int Node { get; set; }

        /// <summary>The part's instructions a second, millions: its time's rate.</summary>
        public uint PartMips { get; set; } = 475;

        /// <summary>The master's pilot, amplitude V at its amplifier; 0 none.</summary>
        public double PilotVolts { get; set; } = 1.5;

        public double PilotHz { get; set; } = 5000.0;

        /// <summary>The far end's common mode at 100 kHz, amplitude V (sto.asc's CMNOISE, 1.8).</summary>
        public double NoiseVolts { get; set; }

        /// <summary>Cinj, Clevel, RESET, FAULTOUT and +15V7, V, as last stepped.</summary>
        public string Chain => string.Format("{0:F3} {1:F3} {2:F3} {3:F3} {4:F3}",
                                             pins[0], pins[1], pins[2], pins[3], pins[4]);

        /// <summary>The instructions a second, millions, the part gives main(): its rate less
        /// the interrupts' share, as last stepped.</summary>
        public double MainMips => PartMips / stretch;

        /// <summary>main()'s instructions so far: the CPU's less the interrupts'. The CPU
        /// reset counts from 0 again: the chain's clock goes on from where it stood.</summary>
        private ulong Main()
        {
            var executed = cpu.ExecutedInstructions;
            if(executed < lastExecuted)
            {
                isr = 0;
                isrFrom = executed;
                depth = 0;
                mainAt = executed;
            }
            lastExecuted = executed;
            var count = executed - isr;
            return count < mainAt ? mainAt : count;
        }

        /// <summary>The part's time, s, at main()'s instruction `count`: the instructions since
        /// the last step at PartMips stretched by the interrupts' share, the sleeps closed since.</summary>
        private double PartTime(ulong count)
        {
            return origin + (count - mainAt) * stretch / (PartMips * 1e6) + slept;
        }

        /// <summary>The chain stepped to now over the edges since the last step.</summary>
        private void Advance()
        {
            if(stoStep == null)
            {
                return;
            }
            if(cpu == null)
            {
                cpu = machine.SystemBus.GetCPUs().OfType<TranslationCPU>().First();
                cpu.AddHookAtWfiStateChange(Sleep);
                // The outermost handler's span: a nested one returns to a handler.
                cpu.AddHookAtInterruptBegin(_ =>
                {
                    if(depth++ == 0)
                    {
                        isrFrom = cpu.ExecutedInstructions;
                    }
                });
                cpu.AddHookAtInterruptEnd(_ =>
                {
                    var executed = cpu.ExecutedInstructions;
                    if(depth > 0 && --depth == 0 && executed > isrFrom)
                    {
                        isr += executed - isrFrom;
                    }
                });
            }
            var count = Main();
            var now = PartTime(count);
            var virtualNow = VirtualClock.Of(machine).Seconds;
            if(asleep)
            {
                var synced = VirtualClock.Of(machine).Synced;
                now += synced - sleptFrom;
                sleptFrom = synced;
            }
            // The interrupts' share of the part's budget over this step, for the next.
            if(virtualNow > virtualAt)
            {
                var share = (isr - isrAt) / (virtualNow - virtualAt) / (PartMips * 1e6);
                stretch = 1.0 / Math.Max(1.0 - share, MainShareFloor);
            }
            isrAt = isr;
            virtualAt = virtualNow;
            origin = now;
            mainAt = count;
            slept = 0.0;
            if(!started)
            {
                stoReset(Node);
                started = true;
                steppedAt = now;
                edgeAt = now;
                edges = 0;
                level = pin;
                return;
            }
            var dt = now - steppedAt;
            if(dt <= 0.0)
            {
                level = pin;                    // no time for the edges to have pumped
                edges = 0;
                return;
            }
            given[0] = (float)PilotVolts;
            given[1] = (float)PilotHz;
            given[2] = (float)NoiseVolts;
            given[3] = (float)afe.DcBusVolts;
            given[4] = afe.Powered ? 1f : 0f;
            given[5] = level ? 1f : 0f;
            stoStep(Node, (float)dt, given, edges, edges > 0 ? at : null, pins);
            level = pin;
            steppedAt = now;
            edges = 0;
            Show(pins);
        }

        private void Show(float[] got)
        {
            afe.CinjVolts = got[0];
            afe.ClevelVolts = got[1];
            afe.GateVolts = got[4];
            var high = got[3] > FaultoutThreshold;
            if(high != released || !started)
            {
                released = high;
                Faultout.Set(high);
                tim1.BreakInput(high);
            }
        }

        /// <summary>WFI entered or left: the sleep's length, virtual.</summary>
        private void Sleep(bool entering)
        {
            var now = VirtualClock.Of(machine).Synced;
            if(entering)
            {
                sleptFrom = now;
            }
            else if(asleep)
            {
                slept += now - sleptFrom;
            }
            asleep = entering;
        }

        private static T Export<T>(string name) where T : Delegate
        {
            return Marshal.GetDelegateForFunctionPointer<T>(NativeLibrary.GetExport(native, name));
        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void StoReset(int i);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void StoStep(int i, float dt, float[] given, int edges, float[] at,
                                      [Out] float[] got);

        private static IntPtr native;
        private static StoReset stoReset;
        private static StoStep stoStep;

        private readonly IMachine machine;
        private readonly Coaxial63100_AFE afe;
        private readonly Coaxial63100_TIM1 tim1;
        private readonly LimitTimer steps;
        // The pilot's amplitude and Hz, the noise, the link, +5, PA10: emu_sto_step's.
        private readonly float[] given = new float[6];
        private readonly float[] pins = new float[Outputs];
        // The edges since the last step, s from it.
        private readonly float[] at = new float[Edges];
        private int edges;
        private TranslationCPU cpu;
        private double origin;
        private ulong mainAt;
        private ulong lastExecuted;
        private ulong isr;
        private ulong isrFrom;
        private ulong isrAt;
        private int depth;
        private double stretch = 1.0;
        private double virtualAt;
        private double slept;
        private double sleptFrom;
        private bool asleep;
        private bool started;
        private double steppedAt;
        private double edgeAt;
        private double worstGap;
        private bool pin;
        private bool level;
        private bool released;

        private const int Outputs = 5;
        /// <summary>Edges a step holds at most: 200 000 a second at the part's, a fill a step
        /// at 5 ms.</summary>
        private const int Edges = 1024;
        /// <summary>The chain's step, Hz: a trip within a ms.</summary>
        private const uint StepHz = 1000;
        /// <summary>The least of the budget left to main(): the drive's ISR takes 0.31.</summary>
        private const double MainShareFloor = 0.25;
        /// <summary>PE15's threshold, V: U11 drives 3.27 or 0.</summary>
        private const float FaultoutThreshold = 1.65f;
    }
}
