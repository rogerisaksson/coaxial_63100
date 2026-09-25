// Coaxial63100_Plant.cs - The power stage and what it turns, in the emulator: every PWM period
// TIM1's duties and MOE drive the board's motor (the world core, world/, built for this host)
// on its load in the machine's world, and the front end reads the currents and link it gives.
// One world a Renode process: a limb's boards share it. While TIM1 counts it is TRGO2 too, the
// ADCs' injected trigger, once a period - and with IdleMips set the core runs BusyMips only while
// an ADC waits on it: the drive's ISR needs the part's speed, the rest runs faster at Renode's.
// The period is the PWM's while the stage is driven or an ADC waits, 1 kHz while the world only
// coasts. The board's heat is one lumped node on its measured 8.33 K/W and 49 J/K
// (coaxial.model.thermal), fed a quiescent 1.2 W - the model's 10 K calibration rise - and the
// FETs' conduction the periods saw; the NTC follows it at its 215 s lag and the MCU die sits over
// it, written into the front end ten times a virtual second. `Thermal` false leaves the two to
// the monitor. It hangs on TIM1_CH1, PE9, and follows TIM1 (Coaxial63100_TIM1.cs) through what
// is written to it.

using System;
using System.Linq;
using System.Runtime.InteropServices;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Peripherals.Sensors;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Analog
{
    public class Coaxial63100_Plant : IGPIOReceiver
    {
        public Coaxial63100_Plant(IMachine machine, Coaxial63100_AFE afe, Coaxial63100_TIM1 tim1,
                                  Coaxial63100_ADC adc1 = null, Coaxial63100_ADC adc2 = null,
                                  Coaxial63100_ADC adc3 = null, Coaxial63100_A1335 angle = null,
                                  uint pwmHz = 50000)
        {
            this.machine = machine;
            this.afe = afe;
            this.angle = angle;
            adcs = new[] { adc1, adc2, adc3 };
            foreach(var adc in adcs)
            {
                if(adc != null)
                {
                    adc.Arming += Run;
                }
            }
            this.tim1 = tim1;
            tim1.Written += (offset, value) =>
            {
                switch(offset)
                {
                    case Bdtr:
                        bdtr = value;
                        Run();
                        break;
                    case Cr2:
                        cr2 = value;
                        break;
                    case Cr1:
                        counting = (value & CounterEnable) != 0;
                        Run();
                        break;
                }
            };
            this.pwmHz = pwmHz;
            period = 1.0f / pwmHz;
            timer = new LimitTimer(machine.ClockSource, pwmHz, this, "period", limit: 1,
                                   workMode: WorkMode.Periodic, eventEnabled: true);
            timer.LimitReached += Step;
            heat = new LimitTimer(machine.ClockSource, HeatHz, this, "heat", limit: 1,
                                  workMode: WorkMode.Periodic, eventEnabled: true);
            heat.LimitReached += Heat;
            board = ntc = Ambient;
            heat.Enabled = true;
        }

        public void Reset()
        {
        }

        public void OnGPIO(int number, bool value)
        {
        }

        /// <summary>The world core's library, loaded once a process: world.dll, world.so.</summary>
        public static void Library(string path)
        {
            if(native != IntPtr.Zero)
            {
                return;
            }
            native = NativeLibrary.Load(path);
            worldReset = Export<WorldReset>("emu_world_reset");
            worldBody = Export<WorldBody>("emu_world_body");
            worldLoad = Export<WorldLoad>("emu_world_load");
            plantAttach = Export<PlantAttach>("emu_plant_attach");
            plantStep = Export<PlantStep>("emu_plant_step");
            plantState = Export<PlantState>("emu_plant_state");
            worldState = Export<WorldState>("emu_world_state");
        }

        public void World(string library, int motors)
        {
            Library(library);
            worldReset(motors);
        }

        public void Body(int kind, float mass, float gravity, float slope, float crr, float cda,
                         float rho, float rider, float kick, float duty)
        {
            worldBody(kind, mass, gravity, slope, crr, cda, rho, rider, kick, duty);
        }

        public void Load(int kind, float gear, float inertia, float mass, float arm, float damping,
                         float kDrag, float kThrust, float radius, float angle)
        {
            worldLoad(Node, kind, gear, inertia, mass, arm, damping, kDrag, kThrust, radius, angle);
        }

        public void Motor(float r, float ld, float lq, float lambda, float polePairs, float j, float b,
                          float vdc, float noise)
        {
            plantAttach(Node, r, ld, lq, lambda, polePairs, j, b, vdc, noise);
            this.polePairs = Math.Max(1.0f, polePairs);
            attached = true;
            Run();
        }

        /// <summary>The core's MIPS while no ADC waits on TRGO2, 0 to leave it alone.</summary>
        public uint IdleMips { get; set; }

        /// <summary>The core's MIPS while one does: the part's own.</summary>
        public uint BusyMips { get; set; } = 475;

        /// <summary>Whether the board's heat drives the NTC and the MCU die.</summary>
        public bool Thermal { get; set; } = true;

        /// <summary>The room, C.</summary>
        public double Ambient { get; set; } = 25.0;

        public double BoardKPerW { get; set; } = 8.33;
        public double BoardJPerK { get; set; } = 49.0;
        public double QuiescentWatts { get; set; } = 1.2;
        /// <summary>A leg's conduction resistance, ohm: its FET on (IAUCN10S7N021, 2.1 mOhm).</summary>
        public double OnOhms { get; set; } = 0.0021;
        public double NtcTauSeconds { get; set; } = 215.0;
        /// <summary>The MCU die over the board, K.</summary>
        public double DieRiseK { get; set; } = 8.0;

        /// <summary>The board node and the NTC, C.</summary>
        public string Temperatures => string.Format("{0:F3} {1:F3}", board, ntc);

        /// <summary>This board's place in the world: its motor's index.</summary>
        public int Node { get; set; }

        /// <summary>The shaft: angle rad electrical, speed rad/s mechanical, the load's output.</summary>
        public string Shaft
        {
            get
            {
                var s = new float[3];
                plantState(Node, s);
                return string.Format("{0} {1} {2}", s[0], s[1], s[2]);
            }
        }

        /// <summary>The body: height m, climb m/s, velocity m/s, distance m, time s.</summary>
        public string BodyState
        {
            get
            {
                var s = new float[5];
                worldState(s);
                return string.Join(" ", s);
            }
        }

        /// <summary>The period's timer: while a world turns, or while TIM1 counts and an ADC waits
        /// on TRGO2 - a 50 kHz event from boot made the idle image 2.9 times slower (2026-09-25).</summary>
        private void Run()
        {
            var waits = false;
            foreach(var adc in adcs)
            {
                waits |= adc != null && adc.WaitsOnTrgo2;
            }
            timer.Enabled = attached || (counting && waits);
            var hz = (waits && counting) || (bdtr & MoeBit) != 0 ? pwmHz : CoastHz;
            if(timer.Frequency != hz)
            {
                timer.Frequency = hz;
                period = 1.0f / hz;
            }
            if(IdleMips > 0)
            {
                var cpu = machine.SystemBus.GetCPUs().OfType<BaseCPU>().First();
                cpu.PerformanceInMips = waits ? BusyMips : IdleMips;
            }
        }

        private void Step()
        {
            if(attached)
            {
                var arr = (float)Math.Max(1U, tim1.ReadDoubleWord(Arr));
                var moe = (bdtr & MoeBit) != 0;
                var got = new float[4];

                plantStep(Node, tim1.ReadDoubleWord(Ccr1) / arr, tim1.ReadDoubleWord(Ccr2) / arr,
                          tim1.ReadDoubleWord(Ccr3) / arr, moe ? 1 : 0, period, got);
                afe.PhaseUAmps = got[0];
                afe.PhaseVAmps = got[1];
                afe.PhaseWAmps = got[2];
                afe.DcBusVolts = got[3];
                squares += got[0] * got[0] + got[1] * got[1] + got[2] * got[2];
                periods++;
                if(angle != null)
                {
                    // The electrical angle unwrapped, then over the pole pairs: wrapped first, the
                    // shaft turned through 360 / pp degrees and back.
                    plantState(Node, shaft);
                    var turned = shaft[0] - electrical;
                    turned -= 2.0 * Math.PI * Math.Round(turned / (2.0 * Math.PI));
                    electrical = shaft[0];
                    mechanical += turned / polePairs;
                    angle.Degrees = mechanical * 180.0 / Math.PI;
                }
            }
            // TRGO2 off OC5REF: the injected sequences, on this period's currents.
            if(counting && (cr2 & Mms2Mask) == Mms2Oc5Ref)
            {
                foreach(var adc in adcs)
                {
                    adc?.OnTrgo2();
                }
            }
        }

        /// <summary>A tenth of a virtual second of the board's heat.</summary>
        private void Heat()
        {
            var dt = 1.0 / HeatHz;
            var conduction = periods > 0 ? squares / periods * OnOhms : 0.0;
            squares = 0.0;
            periods = 0;
            board += dt * (QuiescentWatts + conduction - (board - Ambient) / BoardKPerW) / BoardJPerK;
            ntc += dt / NtcTauSeconds * (board - ntc);
            if(Thermal)
            {
                afe.NtcCelsius = ntc;
                afe.DieCelsius = board + DieRiseK;
            }
        }

        private static T Export<T>(string name) where T : Delegate
        {
            return Marshal.GetDelegateForFunctionPointer<T>(NativeLibrary.GetExport(native, name));
        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void WorldReset(int motors);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void WorldBody(int kind, float mass, float gravity, float slope, float crr,
                                        float cda, float rho, float rider, float period, float duty);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void WorldLoad(int i, int kind, float gear, float inertia, float mass, float arm,
                                        float damping, float kDrag, float kThrust, float radius, float angle);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void PlantAttach(int i, float r, float ld, float lq, float lambda, float polePairs,
                                          float j, float b, float vdc, float noise);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void PlantStep(int i, float d0, float d1, float d2, int driven, float ts,
                                        [Out] float[] got);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void PlantState(int i, [Out] float[] got);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void WorldState([Out] float[] got);

        private static IntPtr native;
        private static WorldReset worldReset;
        private static WorldBody worldBody;
        private static WorldLoad worldLoad;
        private static PlantAttach plantAttach;
        private static PlantStep plantStep;
        private static PlantState plantState;
        private static WorldState worldState;

        private readonly IMachine machine;
        private readonly Coaxial63100_AFE afe;
        private readonly LimitTimer timer;
        private readonly LimitTimer heat;
        private double board;
        private double ntc;
        private double squares;
        private long periods;

        private const uint HeatHz = 10;
        /// <summary>The world's step while nothing drives it, Hz.</summary>
        private const uint CoastHz = 1000;
        private readonly uint pwmHz;
        private float period;
        private readonly Coaxial63100_ADC[] adcs;
        private readonly Coaxial63100_TIM1 tim1;
        private readonly Coaxial63100_A1335 angle;
        private readonly float[] shaft = new float[3];
        private float polePairs = 1.0f;
        private double electrical;
        private double mechanical;
        private uint bdtr;
        private uint cr2;
        private bool counting;
        private bool attached;

        // TIM1 (RM0433): the base, CR1's CEN, CR2's MMS2, the auto-reload, the three compares, MOE
        // in BDTR.
        private const long Cr1 = 0x00;
        private const long Cr2 = 0x04;
        private const uint CounterEnable = 1U;
        private const uint Mms2Mask = 0xFU << 20;
        private const uint Mms2Oc5Ref = 0x8U << 20;        // TIM_TRGO2_OC5REF, MMS2 1000
        private const long Arr = 0x2C;
        private const long Ccr1 = 0x34;
        private const long Ccr2 = 0x38;
        private const long Ccr3 = 0x3C;
        private const long Bdtr = 0x44;
        private const uint MoeBit = 1U << 15;
    }
}
