// Coaxial63100_A1335.cs - The Allegro A1335 angle sensor on SPI4, its chip select PE4: 20-bit
// packets as board_angle.c sends them, four 5-bit words, MSB first. A read is answered in the
// next packet - the register's 16 bits, then a 4-bit CRC (x^4 + x + 1, seed 0xF). ANG is the
// shaft's mechanical angle in twelve bits: the plant's once a world turns it, `Degrees` once the
// monitor sets it, else an invented turn every `TurnSeconds` of virtual time, as the stand-in's -
// one angle for ever looks like a dead link. TSEN its die's temperature in eighths of a
// kelvin; FIELD `Gauss`; the rest reads zero. Unpowered - AFE_ON low - it clocks out all ones,
// as an absent part does.

using System;
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
            Reset();
        }

        public void Reset()
        {
            words = 0;
            command = 0;
            reply = AllOnes;
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
        public double Gauss { get; set; } = 380.0;

        /// <summary>Chip select, PE4: low selects; rising ends the packet.</summary>
        public void OnGPIO(int number, bool value)
        {
            if(!value)
            {
                words = 0;
                command = 0;
                return;
            }
            if(words == Words)
            {
                Packet(command);
            }
            words = 0;
        }

        public byte Transmit(byte data)
        {
            var shift = WordBits * (Words - 1 - Math.Min(words, Words - 1));
            var outgoing = (byte)((afe.Powered ? reply : AllOnes) >> shift & WordMask);
            command = (command << WordBits) | (uint)(data & WordMask);
            words++;
            return outgoing;
        }

        public void FinishTransmission()
        {
        }

        private void Packet(uint packet)
        {
            var register = (int)((packet >> AddressShift) & 0x3FU);
            var write = ((packet >> ReadWriteShift) & 1U) != 0;
            if(write || !afe.Powered)
            {
                reply = AllOnes;
                return;
            }
            var value = Register(register);
            reply = ((uint)value << DataShift) | Crc4(value);
        }

        private ushort Register(int register)
        {
            switch(register)
            {
                case Ang:
                    var turns = Degrees / 360.0;
                    return (ushort)((int)Math.Floor((turns - Math.Floor(turns)) * Counts) & (Counts - 1));
                case Tsen:
                    return (ushort)Math.Max(0, Math.Min(0x0FFF, Math.Round((afe.AngleCelsius + 273.15) * 8.0)));
                case Field:
                    return (ushort)Math.Max(0, Math.Min(0x0FFF, Math.Round(Gauss)));
                default:
                    return 0;
            }
        }

        /// <summary>CRC-4, x^4 + x + 1, seed 0xF, over the 16 data bits MSB first.</summary>
        private static uint Crc4(ushort value)
        {
            uint crc = 0xF;
            for(var bit = 15; bit >= 0; bit--)
            {
                var top = ((crc >> 3) & 1U) ^ ((uint)(value >> bit) & 1U);
                crc = ((crc << 1) & 0xFU) ^ (top != 0 ? 0x3U : 0U);
            }
            return crc;
        }

        private readonly IMachine machine;
        private readonly Coaxial63100_AFE afe;
        private double degrees;
        private bool set;
        private int words;
        private uint command;
        private uint reply;

        private const int Words = 4;
        private const int WordBits = 5;
        private const uint WordMask = 0x1FU;
        private const int AddressShift = 12;
        private const int DataShift = 4;
        private const int ReadWriteShift = 18;
        private const uint AllOnes = 0xFFFFFU;
        private const int Counts = 4096;
        private const int Ang = 0x20;
        private const int Tsen = 0x28;
        private const int Field = 0x2A;
    }
}
