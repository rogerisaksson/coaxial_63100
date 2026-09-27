// Coaxial63100_STO.cs - The STO chain in the emulator: world_sto.c in the world library, the
// circuit of electronic_simulations/sto/sto.asc. The master's common-mode pilot on A1/B1 (UART5's
// pair), U16C's pulse train into Cinj, U4, the keepalive's pump into Clevel off PA10's edges at
// their virtual instants, Q10, U11 onto PE15 (nFAULT, TIM1's BKIN), U9's +15V7 for the gate
// drivers. Stepped at every PA10 edge and at IdleHz while none comes (a sleep); Cinj, Clevel and
// +15V7 written into the front end. The pilot is the master's: PilotVolts 1.5, PilotHz 5000
// (sto.asc's PAM8406), 0 V none. The chain runs on the part's time - the instructions at
// PartMips, a sleep at its length - so the keepalive's edges come as the part spaces them:
// Renode's pacing, the idle core at 100 MIPS and main() skipped under the drive, stretched them
// in virtual time and starved the pump (2026-09-27). Unpaced, it is the virtual time.

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
            idle = new LimitTimer(machine.ClockSource, IdleHz, this, "sto", limit: 1,
                                  workMode: WorkMode.Periodic, eventEnabled: true);
            idle.LimitReached += Advance;
            idle.Enabled = true;
            Reset();
        }

        /// <summary>U11's Y, FAULTOUT: PE15 and TIM1's break input, high while the chain is
        /// released.</summary>
        public GPIO Faultout { get; }

        public void Reset()
        {
            pin = false;
            started = false;
            Show(new float[Outputs]);
        }

        /// <summary>PA10, KEEPALIVE.</summary>
        public void OnGPIO(int number, bool value)
        {
            if(value == pin)
            {
                return;
            }
            Advance();
            worstGap = Math.Max(worstGap, sinceEdge);
            sinceEdge = 0.0;
            pin = value;
        }

        /// <summary>The keepalive's longest gap since the last read, us of the part's time.</summary>
        public double WorstGap
        {
            get
            {
                var worst = Math.Max(worstGap, sinceEdge) * 1e6;
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
            }
            var executed = cpu.ExecutedInstructions;
            if(asleep)
            {
                var now = VirtualClock.Of(machine).Synced;
                slept += now - sleptFrom;
                sleptFrom = now;
            }
            if(!started)
            {
                stoReset(Node);
                started = true;
                executedAt = executed;
                slept = 0.0;
                return;
            }
            var dt = (executed - executedAt) / (PartMips * 1e6) + slept;
            if(dt <= 0.0)
            {
                return;
            }
            executedAt = executed;
            slept = 0.0;
            sinceEdge += dt;
            given[0] = (float)PilotVolts;
            given[1] = (float)PilotHz;
            given[2] = (float)NoiseVolts;
            given[3] = (float)afe.DcBusVolts;
            given[4] = afe.Powered ? 1f : 0f;
            given[5] = pin ? 1f : 0f;
            stoStep(Node, (float)dt, given, 0, IntPtr.Zero, pins);
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
        private delegate void StoStep(int i, float dt, float[] given, int edges, IntPtr at,
                                      [Out] float[] got);

        private static IntPtr native;
        private static StoReset stoReset;
        private static StoStep stoStep;

        private readonly IMachine machine;
        private readonly Coaxial63100_AFE afe;
        private readonly Coaxial63100_TIM1 tim1;
        private readonly LimitTimer idle;
        // The pilot's amplitude and Hz, the noise, the link, +5, PA10: emu_sto_step's.
        private readonly float[] given = new float[6];
        private readonly float[] pins = new float[Outputs];
        private TranslationCPU cpu;
        private ulong executedAt;
        private double slept;
        private double sleptFrom;
        private bool asleep;
        private bool started;
        private double sinceEdge;
        private double worstGap;
        private bool pin;
        private bool released;

        private const int Outputs = 5;
        /// <summary>The chain's step while PA10 is still, Hz: a sleep's trip within a ms.</summary>
        private const uint IdleHz = 1000;
        /// <summary>PE15's threshold, V: U11 drives 3.27 or 0.</summary>
        private const float FaultoutThreshold = 1.65f;
    }
}
