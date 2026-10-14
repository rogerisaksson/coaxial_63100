// Coaxial63100_BNO085.cs - The BNO085 on SPI2 as board_imu.c drives it: world_bno085.c in the
// world library. SHTP packets, the part's clocked out whole under one chip select (PB12) while
// the host's goes in; H_INTN (PD8) low while a packet waits or a WAKE (PD9) asks, released as
// the chip select falls; NRSTN (PD10) rising boots it - reset complete on channel 1, the
// initialize response on 2. On channel 2 it answers Product ID (0xF9, four 0xF8s) and Get
// Feature (0xFE); Set Feature (0xFD) answers 0xFC and starts that report on channel 3 at its
// interval in virtual time, behind a timebase reference: accelerometer Q8 m/s^2, gyroscope Q9
// rad/s, magnetic field Q4 uT, rotation vectors Q14. A reading set through the monitor
// (AccelX .. QuatReal) pipes the values given instead; the rates at zero hold it still. AFE_ON
// powers it.

using System;
using System.Runtime.InteropServices;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Analog;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class Coaxial63100_BNO085 : ISPIPeripheral, IGPIOReceiver
    {
        public Coaxial63100_BNO085(IMachine machine, Coaxial63100_AFE afe)
        {
            this.afe = afe;
            Interrupt = new GPIO();
            ticker = new LimitTimer(machine.ClockSource, TickHz, this, "reports", limit: 1,
                                    workMode: WorkMode.Periodic, eventEnabled: true);
            ticker.LimitReached += Tick;
            afe.Bound += Bind;
            Reset();
        }

        private void Bind(IntPtr library, int nodeIndex)
        {
            if(native == IntPtr.Zero)
            {
                native = library;
                bnoOpen = Export<BnoNode>("emu_bno085_open");
                bnoReset = Export<BnoNode>("emu_bno085_reset");
                bnoPin = Export<BnoPin>("emu_bno085_pin");
                bnoTransmit = Export<BnoTransmit>("emu_bno085_transmit");
                bnoInterrupt = Export<BnoFlag>("emu_bno085_interrupt");
                bnoActive = Export<BnoActive>("emu_bno085_active");
                bnoTick = Export<BnoTick>("emu_bno085_tick");
                bnoRead = Export<BnoRead>("emu_bno085_read");
                bnoPipe = Export<BnoPipe>("emu_bno085_pipe");
                bnoRates = Export<BnoRates>("emu_bno085_rates");
            }
            node = nodeIndex;
            bound = true;
            bnoOpen(node);
            bnoRates(node, rollRate, pitchRate, yawRate, piped ? 1 : 0);
        }

        /// <summary>H_INTN, active low: connected to PD8.</summary>
        public GPIO Interrupt { get; }

        /// <summary>The tumble's angle rates, rad/s: roll about x, pitch about y, yaw about z.</summary>
        public double RollRate
        {
            get => rollRate;
            set { rollRate = value; SyncRates(); }
        }
        public double PitchRate
        {
            get => pitchRate;
            set { pitchRate = value; SyncRates(); }
        }
        public double YawRate
        {
            get => yawRate;
            set { yawRate = value; SyncRates(); }
        }

        /// <summary>Whether the readings are piped - set through the monitor - not moved.</summary>
        public bool Piped
        {
            get => piped;
            set { piped = value; SyncRates(); }
        }

        public double AccelX { get => Read(0); set => Pipe(0, value); }
        public double AccelY { get => Read(1); set => Pipe(1, value); }
        public double AccelZ { get => Read(2); set => Pipe(2, value); }
        public double GyroX { get => Read(3); set => Pipe(3, value); }
        public double GyroY { get => Read(4); set => Pipe(4, value); }
        public double GyroZ { get => Read(5); set => Pipe(5, value); }
        public double MagX { get => Read(6); set => Pipe(6, value); }
        public double MagY { get => Read(7); set => Pipe(7, value); }
        public double MagZ { get => Read(8); set => Pipe(8, value); }
        public double QuatI { get => Read(9); set => Pipe(9, value); }
        public double QuatJ { get => Read(10); set => Pipe(10, value); }
        public double QuatK { get => Read(11); set => Pipe(11, value); }
        public double QuatReal { get => Read(12); set => Pipe(12, value); }

        public void Reset()
        {
            if(bound)
            {
                bnoReset(node);
            }
            ticker.Enabled = false;
            Interrupt.Set(true);
        }

        /// <summary>0 chip select (PB12), 1 WAKE (PD9), 2 NRSTN (PD10); all active low.</summary>
        public void OnGPIO(int number, bool value)
        {
            if(!bound)
            {
                return;
            }
            var powered = afe.Powered ? 1 : 0;
            bnoPin(node, number, value ? 1 : 0, powered);
            ticker.Enabled = bnoActive(node) != 0;
            Interrupt.Set(bnoInterrupt(node, powered) != 0);
        }

        public byte Transmit(byte data)
        {
            return bound ? bnoTransmit(node, data) : (byte)0;
        }

        public void FinishTransmission()
        {
        }

        private void Tick()
        {
            if(!bound)
            {
                return;
            }
            var powered = afe.Powered ? 1 : 0;
            bnoTick(node, powered);
            ticker.Enabled = bnoActive(node) != 0;
            Interrupt.Set(bnoInterrupt(node, powered) != 0);
        }

        private double Read(int index)
        {
            return bound ? bnoRead(node, index) : 0.0;
        }

        private void Pipe(int index, double value)
        {
            piped = true;
            if(bound)
            {
                bnoPipe(node, index, value);
            }
        }

        private void SyncRates()
        {
            if(bound)
            {
                bnoRates(node, rollRate, pitchRate, yawRate, piped ? 1 : 0);
            }
        }

        private static T Export<T>(string name) where T : Delegate
        {
            return Marshal.GetDelegateForFunctionPointer<T>(NativeLibrary.GetExport(native, name));
        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void BnoNode(int i);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void BnoPin(int i, int which, int level, int powered);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate byte BnoTransmit(int i, byte b);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate int BnoFlag(int i, int powered);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate int BnoActive(int i);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void BnoTick(int i, int powered);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate double BnoRead(int i, int index);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void BnoPipe(int i, int index, double value);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void BnoRates(int i, double roll, double pitch, double yaw, int piped);

        private static IntPtr native;
        private static BnoNode bnoOpen;
        private static BnoNode bnoReset;
        private static BnoPin bnoPin;
        private static BnoTransmit bnoTransmit;
        private static BnoFlag bnoInterrupt;
        private static BnoActive bnoActive;
        private static BnoTick bnoTick;
        private static BnoRead bnoRead;
        private static BnoPipe bnoPipe;
        private static BnoRates bnoRates;

        private readonly Coaxial63100_AFE afe;
        private readonly LimitTimer ticker;
        private int node;
        private bool bound;
        private double rollRate = 2.0 * Math.PI / 25.6;
        private double pitchRate = 4.0 * Math.PI / 25.6;
        private double yawRate;
        private bool piped;

        private const uint TickHz = 1000;
    }
}
