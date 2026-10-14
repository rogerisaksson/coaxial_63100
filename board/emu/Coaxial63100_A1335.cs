// Coaxial63100_A1335.cs - The Allegro A1335 angle sensor on SPI4, its chip select PE4:
// world_parts.c in the world library. 20-bit packets as board_angle.c sends them, four 5-bit
// words, MSB first. A read is answered two packets on, as the bench's part does (the manual's
// read cycle has it one) - the register's 16 bits, then a 4-bit CRC (x^4 + x + 1, seed 0xF). ANG
// is the shaft's mechanical angle in twelve bits: the plant's once a world turns it, `Degrees`
// once the monitor sets it, else an invented turn every `TurnSeconds` of virtual time, as the
// stand-in's. TSEN its die's temperature in eighths of a kelvin; FIELD `Gauss`. Unpowered -
// AFE_ON low - it clocks out all ones, as an absent part does.

using System;
using System.Runtime.InteropServices;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Analog;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class Coaxial63100_A1335 : ISPIPeripheral, IGPIOReceiver
    {
        public Coaxial63100_A1335(IMachine machine, Coaxial63100_AFE afe)
        {
            this.machine = machine;
            this.afe = afe;
            afe.Bound += Bind;
            Reset();
        }

        private void Bind(IntPtr library, int nodeIndex)
        {
            if(native == IntPtr.Zero)
            {
                native = library;
                a1335Open = Export<A1335Node>("emu_a1335_open");
                a1335Reset = Export<A1335Node>("emu_a1335_reset");
                a1335Select = Export<A1335Select>("emu_a1335_select");
                a1335Transmit = Export<A1335Transmit>("emu_a1335_transmit");
                a1335Gauss = Export<A1335Gauss>("emu_a1335_gauss");
            }
            node = nodeIndex;
            bound = true;
            a1335Open(node);
            a1335Gauss(node, gauss);
        }

        public void Reset()
        {
            if(bound)
            {
                a1335Reset(node);
                a1335Gauss(node, gauss);
            }
        }

        /// <summary>The magnet's mechanical angle, degrees: the plant writes it each period, and
        /// once written it stays where it is put.</summary>
        public double Degrees
        {
            get
            {
                return set ? degrees
                           : 360.0 * machine.LocalTimeSource.ElapsedVirtualTime.TotalSeconds / TurnSeconds;
            }
            set
            {
                degrees = value;
                set = true;
            }
        }

        /// <summary>The invented turn's period, s of virtual time, while nothing sets the angle.</summary>
        public double TurnSeconds { get; set; } = 2.0;

        /// <summary>The field the part reads, gauss: the stand-in's magnet.</summary>
        public double Gauss
        {
            get => gauss;
            set
            {
                gauss = value;
                if(bound)
                {
                    a1335Gauss(node, value);
                }
            }
        }

        /// <summary>Chip select, PE4: low selects; rising ends the packet.</summary>
        public void OnGPIO(int number, bool value)
        {
            if(bound)
            {
                a1335Select(node, value ? 1 : 0, afe.Powered ? 1 : 0, Degrees, afe.AngleCelsius);
            }
        }

        public byte Transmit(byte data)
        {
            return bound ? a1335Transmit(node, data, afe.Powered ? 1 : 0) : (byte)0x1F;
        }

        public void FinishTransmission()
        {
        }

        private static T Export<T>(string name) where T : Delegate
        {
            return Marshal.GetDelegateForFunctionPointer<T>(NativeLibrary.GetExport(native, name));
        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void A1335Node(int i);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void A1335Select(int i, int level, int powered, double degrees, double celsius);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate byte A1335Transmit(int i, byte word, int powered);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void A1335Gauss(int i, double gauss);

        private static IntPtr native;
        private static A1335Node a1335Open;
        private static A1335Node a1335Reset;
        private static A1335Select a1335Select;
        private static A1335Transmit a1335Transmit;
        private static A1335Gauss a1335Gauss;

        private readonly IMachine machine;
        private readonly Coaxial63100_AFE afe;
        private int node;
        private bool bound;
        private double degrees;
        private bool set;
        private double gauss = 380.0;
    }
}
