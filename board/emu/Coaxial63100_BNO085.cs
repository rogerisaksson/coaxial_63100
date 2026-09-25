// Coaxial63100_BNO085.cs - The BNO085 on SPI2 as board_imu.c drives it: SHTP packets, the part's
// clocked out whole under one chip select (PB12) while the host's goes in; H_INTN (PD8) low while
// a packet waits or a WAKE (PD9) asks, released as the chip select falls; NRSTN (PD10) rising
// boots it - reset complete on channel 1, the initialize response on 2. On channel 2 it answers
// Product ID (0xF9, four 0xF8s) and Get Feature (0xFE); Set Feature (0xFD) answers 0xFC and
// starts that report on channel 3 at its interval in virtual time, behind a timebase
// reference: accelerometer Q8 m/s^2, gyroscope Q9 rad/s, magnetic field Q4 uT, rotation
// vectors Q14. The readings are properties the monitor or a world sets. AFE_ON powers it.

using System;
using System.Collections.Generic;
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
            Reset();
        }

        /// <summary>H_INTN, active low: connected to PD8.</summary>
        public GPIO Interrupt { get; }

        public double AccelX { get; set; }
        public double AccelY { get; set; }
        public double AccelZ { get; set; } = 9.80665;
        public double GyroX { get; set; }
        public double GyroY { get; set; }
        public double GyroZ { get; set; }
        public double MagX { get; set; } = 20.0;
        public double MagY { get; set; }
        public double MagZ { get; set; } = -40.0;
        public double QuatI { get; set; }
        public double QuatJ { get; set; }
        public double QuatK { get; set; }
        public double QuatReal { get; set; } = 1.0;

        public void Reset()
        {
            pending.Clear();
            features.Clear();
            Array.Clear(sequence, 0, sequence.Length);
            selected = false;
            woken = false;
            outgoing = null;
            ticker.Enabled = false;
            Interrupt.Set(true);
        }

        /// <summary>0 chip select (PB12), 1 WAKE (PD9), 2 NRSTN (PD10); all active low.</summary>
        public void OnGPIO(int number, bool value)
        {
            switch(number)
            {
                case 0:
                    if(!value && !selected)
                    {
                        Select();
                    }
                    else if(value && selected)
                    {
                        Deselect();
                    }
                    break;
                case 1:
                    if(!value && Powered)
                    {
                        woken = true;
                    }
                    break;
                case 2:
                    if(!value)
                    {
                        Reset();
                    }
                    else
                    {
                        Boot();
                    }
                    break;
            }
            Signal();
        }

        public byte Transmit(byte data)
        {
            if(!selected)
            {
                return 0;
            }
            incoming.Add(data);
            var at = incoming.Count - 1;
            return (outgoing != null && at < outgoing.Length) ? outgoing[at] : (byte)0;
        }

        public void FinishTransmission()
        {
        }

        private bool Powered => afe.Powered;

        private void Select()
        {
            selected = true;
            woken = false;
            incoming.Clear();
            outgoing = Powered && pending.Count > 0 ? pending.Peek() : null;
        }

        private void Deselect()
        {
            selected = false;
            if(outgoing != null && incoming.Count >= outgoing.Length)
            {
                pending.Dequeue();          // clocked out whole
            }
            outgoing = null;
            if(Powered && incoming.Count >= Header)
            {
                var length = (incoming[0] | incoming[1] << 8) & 0x7FFF;
                if(length > Header && incoming.Count >= length)
                {
                    Received(incoming[2], incoming.GetRange(Header, length - Header).ToArray());
                }
            }
        }

        private void Signal()
        {
            Interrupt.Set(!(Powered && !selected && (woken || pending.Count > 0)));
        }

        private void Boot()
        {
            Reset();
            if(!Powered)
            {
                return;
            }
            Send(ChannelExecutable, new byte[] { 0x01 });                 // reset complete
            Send(ChannelControl, CommandResponse(0x84));                 // initialize, unsolicited
        }

        private void Received(byte channel, byte[] cargo)
        {
            if(channel == ChannelExecutable && cargo.Length > 0 && cargo[0] == 0x01)
            {
                Boot();
                return;
            }
            if(channel != ChannelControl || cargo.Length == 0)
            {
                return;
            }
            switch(cargo[0])
            {
                case ProductIdRequest:
                    for(var entry = 0; entry < 4; entry++)
                    {
                        Send(ChannelControl, ProductId(entry));
                    }
                    break;
                case SetFeature:
                    if(cargo.Length >= 9)
                    {
                        var id = cargo[1];
                        var us = BitConverter.ToUInt32(cargo, 5);
                        if(us == 0)
                        {
                            features.Remove(id);
                        }
                        else
                        {
                            features[id] = new Feature { IntervalTicks = Math.Max(1U, us * TickHz / 1000000U) };
                        }
                        Send(ChannelControl, FeatureResponse(id, us));
                        ticker.Enabled = features.Count > 0;
                    }
                    break;
                case GetFeature:
                    if(cargo.Length >= 2)
                    {
                        Feature f;
                        Send(ChannelControl, FeatureResponse(cargo[1], features.TryGetValue(cargo[1], out f)
                            ? f.IntervalTicks * 1000000U / TickHz : 0U));
                    }
                    break;
                case CommandRequest:
                    if(cargo.Length >= 3)
                    {
                        Send(ChannelControl, CommandResponse(cargo[2]));
                    }
                    break;
            }
        }

        private void Tick()
        {
            if(!Powered)
            {
                Reset();                    // its supply gone, what it was told is gone
                return;
            }
            var reports = new List<byte> { Timebase, 0, 0, 0, 0 };
            foreach(var pair in features)
            {
                var f = pair.Value;
                if(++f.Elapsed < f.IntervalTicks)
                {
                    continue;
                }
                f.Elapsed = 0;
                reports.AddRange(Report(pair.Key, f));
            }
            if(reports.Count > 5)
            {
                Send(ChannelInput, reports.ToArray());
            }
        }

        private byte[] Report(byte id, Feature f)
        {
            var head = new List<byte> { id, f.Sequence++, 3, 0 };        // status 3: accuracy high
            switch(id)
            {
                case Accelerometer:
                    return Vector(head, AccelX, AccelY, AccelZ, 256.0);
                case Gyroscope:
                    return Vector(head, GyroX, GyroY, GyroZ, 512.0);
                case MagneticField:
                    return Vector(head, MagX, MagY, MagZ, 16.0);
                case RotationVector:
                case GameRotationVector:
                    var q = Vector(head, QuatI, QuatJ, QuatK, 16384.0);
                    var tail = new List<byte>(q);
                    tail.AddRange(Q(QuatReal, 16384.0));
                    if(id == RotationVector)
                    {
                        tail.AddRange(Q(0.05, 4096.0));                  // accuracy, rad Q12
                    }
                    return tail.ToArray();
                default:
                    return Vector(head, 0.0, 0.0, 0.0, 1.0);
            }
        }

        private static byte[] Vector(List<byte> head, double x, double y, double z, double scale)
        {
            var out_ = new List<byte>(head);
            out_.AddRange(Q(x, scale));
            out_.AddRange(Q(y, scale));
            out_.AddRange(Q(z, scale));
            return out_.ToArray();
        }

        private static byte[] Q(double value, double scale)
        {
            var q = (short)Math.Max(short.MinValue, Math.Min(short.MaxValue, Math.Round(value * scale)));
            return new[] { (byte)q, (byte)(q >> 8) };
        }

        private static byte[] ProductId(int entry)
        {
            // Figure 1-29: reset cause, SW 3.2, part 10003606 + entry, build 324, patch 0.
            var cargo = new byte[16];
            cargo[0] = ProductIdResponse;
            cargo[1] = 1;                     // power-on reset
            cargo[2] = 3;
            cargo[3] = 2;
            BitConverter.GetBytes(10003606U + (uint)entry).CopyTo(cargo, 4);
            BitConverter.GetBytes(324U).CopyTo(cargo, 8);
            return cargo;
        }

        private static byte[] FeatureResponse(byte id, uint intervalUs)
        {
            var cargo = new byte[17];
            cargo[0] = GetFeatureResponse;
            cargo[1] = id;
            BitConverter.GetBytes(intervalUs).CopyTo(cargo, 5);
            return cargo;
        }

        private static byte[] CommandResponse(byte command)
        {
            var cargo = new byte[16];
            cargo[0] = 0xF1;
            cargo[2] = command;
            return cargo;
        }

        private void Send(byte channel, byte[] cargo)
        {
            var packet = new byte[Header + cargo.Length];
            packet[0] = (byte)packet.Length;
            packet[1] = (byte)(packet.Length >> 8);
            packet[2] = channel;
            packet[3] = sequence[channel]++;
            cargo.CopyTo(packet, Header);
            pending.Enqueue(packet);
            Signal();
        }

        private class Feature
        {
            public uint IntervalTicks;
            public uint Elapsed;
            public byte Sequence;
        }

        private readonly Coaxial63100_AFE afe;
        private readonly LimitTimer ticker;
        private readonly Queue<byte[]> pending = new Queue<byte[]>();
        private readonly Dictionary<byte, Feature> features = new Dictionary<byte, Feature>();
        private readonly List<byte> incoming = new List<byte>();
        private readonly byte[] sequence = new byte[6];
        private byte[] outgoing;
        private bool selected;
        private bool woken;

        private const int Header = 4;
        private const uint TickHz = 1000;
        private const byte ChannelExecutable = 1;
        private const byte ChannelControl = 2;
        private const byte ChannelInput = 3;
        private const byte ProductIdRequest = 0xF9;
        private const byte ProductIdResponse = 0xF8;
        private const byte SetFeature = 0xFD;
        private const byte GetFeature = 0xFE;
        private const byte GetFeatureResponse = 0xFC;
        private const byte CommandRequest = 0xF2;
        private const byte Timebase = 0xFB;
        private const byte Accelerometer = 0x01;
        private const byte Gyroscope = 0x02;
        private const byte MagneticField = 0x03;
        private const byte RotationVector = 0x05;
        private const byte GameRotationVector = 0x08;
    }
}
