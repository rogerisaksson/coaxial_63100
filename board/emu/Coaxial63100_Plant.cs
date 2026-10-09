// Coaxial63100_Plant.cs - The power stage and what it turns, in the emulator: every PWM period
// TIM1's duties and MOE drive the board's motor (the world core, world/, built for this host)
// on its load in the machine's world, and the front end reads the currents and link it gives.
// One world a Renode process: a limb's boards share it. While TIM1 counts it is TRGO2 too, the
// ADCs' injected trigger, once a period - and with IdleMips set the core runs BusyMips only while
// an ADC waits on it: the drive's ISR needs the part's speed, the rest runs faster at Renode's.
// With LoopSlice set, main() runs that long short of each timer event while one waits, the rest
// skipped: the handlers whole, the polling loop throttled - 16.5 -> 2.5 wall s a virtual s under
// the drive at 0.5 us, 50 000 updates a virtual second kept (2026-09-25).
// The period is the PWM's while the stage is driven or an ADC waits, 1 kHz while the world only
// coasts. The board's heat is thermal.c's network in the world library (world_heat.c), the truth
// its observer is judged by: the duties, MOE, the legs' mean squares, the link and the shaft ten
// times a virtual second, the NTC's element and the two dies written into the front end.
// `Thermal` false leaves the three to the monitor. The stage is driven while MOE is set and the
// gate drivers' supply, the STO chain's +15V7 (Coaxial63100_STO.cs), stands over their UVLO. It
// hangs on TIM1_CH1, PE9, and follows TIM1 (Coaxial63100_TIM1.cs) through what is written to it.

using System;
using System.Linq;
using System.Runtime.InteropServices;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Peripherals.Sensors;
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
                    case Ccer:
                        ccer = value;
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
            period = 0.5f / pwmHz;
            timer = new LimitTimer(machine.ClockSource, 2 * pwmHz, this, "period", limit: 1,
                                   workMode: WorkMode.Periodic, eventEnabled: true);
            timer.LimitReached += Step;
            heat = new LimitTimer(machine.ClockSource, HeatHz, this, "heat", limit: 1,
                                  workMode: WorkMode.Periodic, eventEnabled: true);
            heat.LimitReached += Heat;
            heat.Enabled = true;
        }

        public void Reset()
        {
            depth = 0;
            trigger = false;
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
            heatReset = Export<HeatReset>("emu_heat_reset");
            heatStep = Export<HeatStep>("emu_heat_step");
            heatRoom = Export<HeatRoom>("emu_heat_room");
            heatApplication = Export<HeatApplication>("emu_heat_application");
            worldDrag = Export<WorldDrag>("emu_world_drag");
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

        private void LayDrag()
        {
            if(native != IntPtr.Zero)
            {
                worldDrag(Node, (float)drag, (float)loadTorque);
            }
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

        /// <summary>While one does, main()'s time between the handlers, us: the rest to the next
        /// timer event skipped, time passing with nothing run. 0 runs it all.</summary>
        public double LoopSlice { get; set; }

        /// <summary>Whether the board's heat drives the NTC and the two dies.</summary>
        public bool Thermal { get; set; } = true;

        /// <summary>The room, C, and its air path and the laminate's capacity scaled on the
        /// board's - a situation's (coaxial.model.rooms) - laid at the next heat step.</summary>
        public double Ambient { get => ambient; set { ambient = value; roomLaid = false; } }

        public double Air { get => air; set { air = value; roomLaid = false; } }

        public double Capacity { get => capacity; set { capacity = value; roomLaid = false; } }

        /// <summary>What the board is mounted in, thermal_app_t: its heat's network so, laid at
        /// the next heat step.</summary>
        public int Application { get => application; set { application = value; roomLaid = false; } }

        /// <summary>The load's drag, N m per (rad/s)^2, and a torque against its turning, N m -
        /// a page's propeller and its stage - laid live on the world.</summary>
        public double Drag { get => drag; set { drag = value; LayDrag(); } }

        public double LoadTorque { get => loadTorque; set { loadTorque = value; LayDrag(); } }

        /// <summary>The heat's clock, thermal s per virtual s: 1 until the rig sets the world's
        /// (coaxial.model.thermal.HASTE) with the board's observer's (thermal op 13). A step is
        /// a tenth of a thermal second at any clock.</summary>
        public double Haste
        {
            get => haste;
            set
            {
                haste = Math.Max(1.0, value);
                heat.Frequency = (ulong)Math.Round(HeatHz * haste);
            }
        }

        /// <summary>The NTC's element, the MCU's die and the A1335's, C.</summary>
        public string Temperatures => string.Format("{0:F3} {1:F3} {2:F3}", seen[0], seen[1], seen[2]);

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

        /// <summary>The period's timer, two events a period: while a world turns, or while TIM1
        /// counts and an ADC waits on TRGO2 - a 50 kHz event from boot made the idle image 2.9
        /// times slower (2026-09-25).</summary>
        private void Run()
        {
            var waits = false;
            foreach(var adc in adcs)
            {
                waits |= adc != null && adc.WaitsOnTrgo2;
            }
            timer.Enabled = attached || (counting && waits);
            var hz = (waits && counting) || (bdtr & MoeBit) != 0 ? pwmHz : CoastHz;
            if(timer.Frequency != 2 * hz)
            {
                timer.Frequency = 2 * hz;           // the update and the trigger, half a period apart
                period = 0.5f / hz;
            }
            busy = waits;
            if(IdleMips > 0)
            {
                if(cpu == null)
                {
                    cpu = machine.SystemBus.GetCPUs().OfType<TranslationCPU>().First();
                }
                if(busy && LoopSlice > 0 && !paced)
                {
                    // The outermost handler's exit: a nested one returns to a handler.
                    cpu.AddHookAtInterruptBegin(_ =>
                    {
                        depth++;
                    });
                    cpu.AddHookAtInterruptEnd(_ =>
                    {
                        if(depth > 0 && --depth == 0 && busy)
                        {
                            Skip();
                        }
                    });
                    paced = true;
                }
                Pace();
            }
        }

        /// <summary>BusyMips while an ADC waits on TRGO2, IdleMips else.</summary>
        private void Pace()
        {
            var mips = busy ? BusyMips : IdleMips;
            if(IdleMips == 0 || cpu.PerformanceInMips == mips)
            {
                return;
            }
            cpu.PerformanceInMips = mips;
        }

        /// <summary>The core skipped to LoopSlice short of the next timer event, main() running
        /// that last slice: in whole grains of the CPU's rate, which SkipTime takes exactly.</summary>
        private void Skip()
        {
            cpu.SyncTime();
            var gap = ((BaseClockSource)machine.ClockSource).NearestLimitIn.Ticks;
            var slice = (ulong)(LoopSlice * 1e3);
            if(gap <= slice)
            {
                return;
            }
            var grain = 1000UL / Gcd(cpu.PerformanceInMips, 1000U);
            var skip = (gap - slice) / grain * grain;
            if(skip > 0)
            {
                cpu.SkipTime(TimeInterval.FromTicks(skip));
            }
        }

        private static uint Gcd(uint a, uint b)
        {
            return b == 0 ? a : Gcd(b, a % b);
        }

        /// <summary>Half a period of the stage and the world, at the underflow and at the top: as
        /// the part has it, the compares written since the last update land at the underflow
        /// (OCxPE) for the period to come, and the injected sequences sample at the top, TRGO2
        /// 15 ticks under it (board_sync.c), where the low switches are on and the shunts read.
        /// A duty asked in the handler at the top lands at the next underflow and shows whole
        /// in the sample after next (drive.c's PIPELINE). One step a period, the compares read
        /// at its start, showed them a period early and the injection's fs/2 demodulator read
        /// their sign turned; sampled at the period's end they showed the injection's peaks,
        /// which the part's samples at the top do not (2026-09-27).</summary>
        private void Step()
        {
            if(attached)
            {
                var got = new float[4];

                plantStep(Node, duty[0], duty[1], duty[2], Driven ? 1 : 0, period, got);
                afe.PhaseUAmps = got[0];
                afe.PhaseVAmps = got[1];
                afe.PhaseWAmps = got[2];
                afe.DcBusVolts = got[3];
                for(var k = 0; k < 3; k++)
                {
                    squares[k] += got[k] * got[k];
                }
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
            if(trigger)
            {
                // TRGO2 off OC5REF: the injected sequences, on the currents at the top.
                if(counting && (cr2 & Mms2Mask) == Mms2Oc5Ref)
                {
                    foreach(var adc in adcs)
                    {
                        adc?.OnTrgo2();
                    }
                }
            }
            else
            {
                var arr = (float)Math.Max(1U, tim1.ReadDoubleWord(Arr));
                for(var k = 0; k < 3; k++)
                {
                    duty[k] = tim1.ReadDoubleWord(Ccr1 + 4 * k) / arr;
                }
                // main()'s slice before the trigger as well as before the update.
                if(busy && paced)
                {
                    Skip();
                }
            }
            trigger = !trigger;
        }

        /// <summary>A tenth of a thermal second of the board's heat.</summary>
        private void Heat()
        {
            var n = Math.Max(1L, periods);
            var arr = (float)Math.Max(1U, tim1.ReadDoubleWord(Arr));
            load[0] = afe.Powered ? 1f : 0f;
            load[1] = Driven ? 1f : 0f;
            load[2] = tim1.ReadDoubleWord(Ccr1) / arr;
            load[3] = tim1.ReadDoubleWord(Ccr2) / arr;
            load[4] = tim1.ReadDoubleWord(Ccr3) / arr;
            for(var k = 0; k < 3; k++)
            {
                load[5 + k] = (float)(squares[k] / n);
                squares[k] = 0.0;
            }
            periods = 0;
            load[8] = (float)afe.DcBusVolts;
            load[9] = Math.Abs(shaft[1]) * 60f / (2f * (float)Math.PI);
            if(native == IntPtr.Zero || !Thermal)
            {
                return;
            }
            if(!heated)
            {
                heatReset(Node, (float)ambient);
                heated = true;
            }
            if(!roomLaid)
            {
                heatApplication(Node, application);
                heatRoom(Node, (float)ambient, (float)air, (float)capacity);
                roomLaid = true;
            }
            heatStep(Node, 1f / HeatHz, load, seen);
            afe.NtcCelsius = seen[0];
            afe.DieCelsius = seen[1];
            afe.AngleCelsius = seen[2];
        }

        /// <summary>MOE set, the six outputs enabled and the drivers supplied: the 2EDL8034's
        /// outputs follow TIM1. The firmware enables the six together: one leg open alone is not
        /// modelled.</summary>
        private bool Driven => (bdtr & MoeBit) != 0 && (ccer & Outputs) == Outputs
                               && afe.GateVolts >= GateUvloVolts;

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
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void HeatReset(int i, float ambient);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void HeatStep(int i, float dt, float[] load, [Out] float[] seen);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void HeatRoom(int i, float ambient, float air, float capacity);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void WorldDrag(int i, float kDrag, float torque);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void HeatApplication(int i, int app);

        private static IntPtr native;
        private static WorldReset worldReset;
        private static WorldBody worldBody;
        private static WorldLoad worldLoad;
        private static PlantAttach plantAttach;
        private static PlantStep plantStep;
        private static PlantState plantState;
        private static WorldState worldState;
        private static HeatReset heatReset;
        private static HeatStep heatStep;
        private static HeatRoom heatRoom;
        private static HeatApplication heatApplication;
        private static WorldDrag worldDrag;

        private readonly IMachine machine;
        private readonly Coaxial63100_AFE afe;
        private readonly LimitTimer timer;
        private readonly LimitTimer heat;
        private readonly double[] squares = new double[3];
        private long periods;
        // AFE_ON, MOE, the duties, the legs' mean squares, the link, the shaft's rpm: emu_heat_step's.
        private readonly float[] load = new float[10];
        private readonly float[] seen = { 25f, 25f, 25f };
        private bool heated;
        private double haste = 1.0;
        private double ambient = 25.0;
        private double air = 1.0;
        private double capacity = 1.0;
        private int application;
        private bool roomLaid = true;
        private double drag;
        private double loadTorque;

        private const uint HeatHz = 10;
        /// <summary>The 2EDL8034's UVLO, V (motor_inverters/half_bridge/2EDL8034F5.lib).</summary>
        private const double GateUvloVolts = 7.5;
        /// <summary>The world's step while nothing drives it, Hz.</summary>
        private const uint CoastHz = 1000;
        private readonly uint pwmHz;
        private float period;
        private readonly Coaxial63100_ADC[] adcs;
        private readonly Coaxial63100_TIM1 tim1;
        private readonly Coaxial63100_A1335 angle;
        private readonly float[] shaft = new float[3];
        // The three compares as the last update loaded them, of ARR.
        private readonly float[] duty = new float[3];
        private float polePairs = 1.0f;
        private double electrical;
        private double mechanical;
        private uint bdtr;
        private uint ccer;
        private uint cr2;
        private bool counting;
        private bool attached;
        private bool busy;
        private bool paced;
        private bool trigger;
        private int depth;
        private TranslationCPU cpu;

        // TIM1 (RM0433): the base, CR1's CEN, CR2's MMS2, the six outputs' enables in CCER, the
        // auto-reload, the three compares, MOE in BDTR.
        private const long Cr1 = 0x00;
        private const long Cr2 = 0x04;
        private const uint CounterEnable = 1U;
        private const uint Mms2Mask = 0xFU << 20;
        private const uint Mms2Oc5Ref = 0x8U << 20;        // TIM_TRGO2_OC5REF, MMS2 1000
        private const long Ccer = 0x20;
        private const uint Outputs = 0x555U;                // CC1E CC1NE CC2E CC2NE CC3E CC3NE
        private const long Arr = 0x2C;
        private const long Ccr1 = 0x34;
        private const long Ccr2 = 0x38;
        private const long Ccr3 = 0x3C;
        private const long Bdtr = 0x44;
        private const uint MoeBit = 1U << 15;
    }
}
