// Coaxial63100_BNO085.cs - The BNO085 on SPI2 as board_imu.c drives it: SHTP packets, the part's
// clocked out whole under one chip select (PB12) while the host's goes in; H_INTN (PD8) low while
// a packet waits or a WAKE (PD9) asks, released as the chip select falls; NRSTN (PD10) rising
// boots it - reset complete on channel 1, the initialize response on 2. On channel 2 it answers
// Product ID (0xF9, four 0xF8s) and Get Feature (0xFE); Set Feature (0xFD) answers 0xFC and
// starts that report on channel 3 at its interval in virtual time, behind a timebase
// reference: accelerometer Q8 m/s^2, gyroscope Q9 rad/s, magnetic field Q4 uT, rotation
// vectors Q14. It moves: the stand-in's tumble in virtual time, q = q_y(pitch) q_x(roll)
// q_z(yaw), a turn of roll and two of pitch in 2.56 s - and the gyroscope, the body rate each
// tick's turn gives, gravity and the field read in the body frame off that one attitude. A reading set through the monitor
// (AccelX .. QuatReal) pipes the values given instead; the rates at zero hold it still. AFE_ON
// powers it.

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

        /// <summary>The tumble's angle rates, rad/s: roll about x, pitch about y, yaw about z.</summary>
        public double RollRate { get; set; } = 2.0 * Math.PI / 2.56;
        public double PitchRate { get; set; } = 4.0 * Math.PI / 2.56;
        public double YawRate { get; set; }

        /// <summary>Whether the readings are piped - set through the monitor - not moved.</summary>
        public bool Piped { get; set; }

        public double AccelX { get { return Read(0); } set { Pipe(0, value); } }
        public double AccelY { get { return Read(1); } set { Pipe(1, value); } }
        public double AccelZ { get { return Read(2); } set { Pipe(2, value); } }
        public double GyroX { get { return Read(3); } set { Pipe(3, value); } }
        public double GyroY { get { return Read(4); } set { Pipe(4, value); } }
        public double GyroZ { get { return Read(5); } set { Pipe(5, value); } }
        public double MagX { get { return Read(6); } set { Pipe(6, value); } }
        public double MagY { get { return Read(7); } set { Pipe(7, value); } }
        public double MagZ { get { return Read(8); } set { Pipe(8, value); } }
        public double QuatI { get { return Read(9); } set { Pipe(9, value); } }
        public double QuatJ { get { return Read(10); } set { Pipe(10, value); } }
        public double QuatK { get { return Read(11); } set { Pipe(11, value); } }
        public double QuatReal { get { return Read(12); } set { Pipe(12, value); } }

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
            Turn(1.0 / TickHz);
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

        /// <summary>The attitude a tick on, and the body rate that turn was: 2 vec(q* q') / dt.</summary>
        private void Turn(double dt)
        {
            elapsed += dt;
            var was = (double[])attitude.Clone();
            var now = Times(Axis(2, PitchRate * elapsed),
                            Times(Axis(1, RollRate * elapsed), Axis(3, YawRate * elapsed)));
            Array.Copy(now, attitude, 4);
            var turn = Times(new[] { was[0], -was[1], -was[2], -was[3] }, now);
            var sign = turn[0] < 0.0 ? -1.0 : 1.0;
            for(var i = 0; i < 3; i++)
            {
                body[i] = 2.0 * sign * turn[i + 1] / dt;
            }
        }

        /// <summary>A rotation of `angle` about axis 1 x, 2 y, 3 z, as (w, x, y, z).</summary>
        private static double[] Axis(int axis, double angle)
        {
            var q = new double[4];
            q[0] = Math.Cos(angle / 2.0);
            q[axis] = Math.Sin(angle / 2.0);
            return q;
        }

        private static double[] Times(double[] a, double[] b)
        {
            return new[]
            {
                a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
                a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
                a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
                a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
            };
        }

        /// <summary>An earth-frame vector in the body frame: R(q) transposed.</summary>
        private double[] ToBody(double ex, double ey, double ez)
        {
            double w = attitude[0], x = attitude[1], y = attitude[2], z = attitude[3];
            return new[]
            {
                (1 - 2 * (y * y + z * z)) * ex + 2 * (x * y + w * z) * ey + 2 * (x * z - w * y) * ez,
                2 * (x * y - w * z) * ex + (1 - 2 * (x * x + z * z)) * ey + 2 * (y * z + w * x) * ez,
                2 * (x * z + w * y) * ex + 2 * (y * z - w * x) * ey + (1 - 2 * (x * x + y * y)) * ez,
            };
        }

        /// <summary>Reading `index` - accel xyz, gyro xyz, mag xyz, quaternion i j k real - moved
        /// off the attitude, or as piped.</summary>
        private double Read(int index)
        {
            if(Piped)
            {
                return piped[index];
            }
            if(index < 3)
            {
                return ToBody(0.0, 0.0, Gravity)[index];
            }
            if(index < 6)
            {
                return body[index - 3];
            }
            if(index < 9)
            {
                return ToBody(Field[0], Field[1], Field[2])[index - 6];
            }
            return index == 12 ? attitude[0] : attitude[index - 8];
        }

        private void Pipe(int index, double value)
        {
            if(!Piped)
            {
                for(var i = 0; i < piped.Length; i++)
                {
                    piped[i] = Read(i);
                }
                Piped = true;
            }
            piped[index] = value;
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
        private readonly double[] attitude = { 1.0, 0.0, 0.0, 0.0 };
        private readonly double[] body = new double[3];
        private double elapsed;
        private readonly double[] piped = new double[13];

        private const double Gravity = 9.80665;
        /// <summary>The earth's field where the stand-in's is, uT.</summary>
        private static readonly double[] Field = { 22.0, -3.0, 41.0 };
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
