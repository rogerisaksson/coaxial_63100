// Coaxial63100_Plant.cs - The power stage and what it turns, in the emulator: every PWM period
// TIM1's duties and MOE drive the board's motor (the world core, world/, built for this host)
// on its load in the machine's world, and the front end reads the currents and link it gives.
// One world a Renode process: a limb's boards share it. While TIM1 counts it is TRGO2 too, the
// ADCs' injected trigger, once a period. It hangs on TIM1_CH1, PE9, and keeps TIM1's BDTR -
// dead time, MOE - and CR2 - TRGO2's source - which Renode's STM32_Timer drops (2026-09-25).

using System;
using System.Runtime.InteropServices;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Sensors;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Analog
{
    public class Coaxial63100_Plant : IGPIOReceiver
    {
        public Coaxial63100_Plant(IMachine machine, Coaxial63100_AFE afe, IBusPeripheral tim1,
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
            machine.SystemBus.SetHookBeforePeripheralWrite<uint>(tim1, (value, offset) =>
            {
                switch(offset)
                {
                    case Bdtr:
                        bdtr = value;
                        break;
                    case Cr2:
                        cr2 = value;
                        break;
                    case Cr1:
                        counting = (value & CounterEnable) != 0;
                        Run();
                        break;
                }
                return value;
            });
            machine.SystemBus.SetHookAfterPeripheralRead<uint>(tim1,
                (value, offset) => (offset == Bdtr) ? bdtr : (offset == Cr2) ? cr2 : value);
            period = 1.0f / pwmHz;
            timer = new LimitTimer(machine.ClockSource, pwmHz, this, "period", limit: 1,
                                   workMode: WorkMode.Periodic, eventEnabled: true);
            timer.LimitReached += Step;
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
        }

        private void Step()
        {
            if(attached)
            {
                var bus = machine.SystemBus;
                var arr = (float)Math.Max(1U, bus.ReadDoubleWord(Tim1 + Arr));
                var moe = (bdtr & MoeBit) != 0;
                var got = new float[4];

                plantStep(Node, bus.ReadDoubleWord(Tim1 + Ccr1) / arr, bus.ReadDoubleWord(Tim1 + Ccr2) / arr,
                          bus.ReadDoubleWord(Tim1 + Ccr3) / arr, moe ? 1 : 0, period, got);
                afe.PhaseUAmps = got[0];
                afe.PhaseVAmps = got[1];
                afe.PhaseWAmps = got[2];
                afe.DcBusVolts = got[3];
                if(angle != null)
                {
                    plantState(Node, shaft);
                    angle.Degrees = shaft[0] / polePairs * 180.0 / Math.PI;
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
        private readonly float period;
        private readonly Coaxial63100_ADC[] adcs;
        private readonly Coaxial63100_A1335 angle;
        private readonly float[] shaft = new float[3];
        private float polePairs = 1.0f;
        private uint bdtr;
        private uint cr2;
        private bool counting;
        private bool attached;

        // TIM1 (RM0433): the base, CR1's CEN, CR2's MMS2, the auto-reload, the three compares, MOE
        // in BDTR.
        private const ulong Tim1 = 0x40010000;
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
