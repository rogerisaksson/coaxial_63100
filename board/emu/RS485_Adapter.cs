// RS485_Adapter.cs - The host's USB-RS485 adapter on an emulated limb: its host face on a socket
// terminal, its bus face on the limb's hub. The host's bytes reach the bus one character time
// apart at the bus's baud in virtual time, as UART_Line paces a console; the bus's reach the
// host as they come. A frame starts where the host's bytes arrive RTU's t1.5 apart in virtual
// time - they cross at quantum boundaries, a write sometimes split across two - and waits for
// the bus to have been quiet t3.5 and the master's 0.25 ms to act on the last, either way. The
// host keeps those gaps on its own clock; at a fraction of real time, and quantized, they are
// shorter on the bus's - a node still purging its own echo lost the next request's first
// bytes, and broadcast chunks queued t3.5 apart were lost, at 10 Mbit (2026-09-25). Renode
// joins a terminal to a UART only, hence two faces.

using System;
using System.Collections.Generic;
using Antmicro.Migrant;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.UART
{
    /// <summary>The face on the limb's hub.</summary>
    public class RS485_AdapterBus : IUART, IGPIOReceiver
    {
        public RS485_AdapterBus(IMachine machine, uint baudRate = 115200)
        {
            this.machine = machine;
            BaudRate = baudRate;
            // Built with room for the longest turnaround: a LimitTimer's Value is held to the limit
            // it was made with, not the one it has.
            timer = new LimitTimer(machine.ClockSource, baudRate, this, "character", limit: uint.MaxValue,
                                   workMode: WorkMode.Periodic, eventEnabled: true);
            timer.Limit = BitsPerCharacter;
            timer.LimitReached += Deliver;
        }

        public void Reset()
        {
            lock(queue)
            {
                queue.Clear();
                timer.Enabled = false;
            }
        }

        public void OnGPIO(int number, bool value)
        {
        }

        /// <summary>A byte off the bus, to the host.</summary>
        public void WriteChar(byte value)
        {
            lastBus = Now();
            Host?.FromBus(value);
        }

        [field: Transient]
        public event Action<byte> CharReceived;

        /// <summary>The bus's rate, bits a second: 115 200 as the app starts, 10 000 000 as the
        /// bootloader runs.</summary>
        public uint BaudRate { get; set; }

        public Bits StopBits => Bits.One;

        public Parity ParityBit => Parity.None;

        public RS485_AdapterHost Host { get; set; }

        /// <summary>A byte from the host, onto the bus at the bus's pace.</summary>
        public void Send(byte value)
        {
            lock(queue)
            {
                var at = Now();
                queue.Enqueue(new Pending { Value = value, Starts = at - lastHost > Interchar });
                lastHost = at;
                if(!timer.Enabled)
                {
                    timer.Frequency = BaudRate;
                    timer.Limit = BitsPerCharacter + Owed(queue.Peek().Starts);
                    timer.Value = timer.Limit;
                    timer.Enabled = true;
                }
            }
        }

        private void Deliver()
        {
            byte value;
            lock(queue)
            {
                value = queue.Dequeue().Value;
                lastSent = Now();
                if(queue.Count == 0)
                {
                    timer.Enabled = false;
                }
                else
                {
                    // The limit, and the count the timer reloaded from the last one's before
                    // this ran.
                    timer.Limit = BitsPerCharacter + Owed(queue.Peek().Starts);
                    timer.Value = timer.Limit;
                }
            }
            CharReceived?.Invoke(value);
        }

        /// <summary>Bit times a frame's first character waits past its own: what is left of
        /// t3.5 and Act since the bus last carried a byte, either way.</summary>
        private ulong Owed(bool starts)
        {
            if(!starts)
            {
                return 0;
            }
            var quiet = Now() - Math.Max(lastBus, lastSent);
            return (ulong)(Math.Max(0.0, Turnaround + Act - quiet) * BaudRate);
        }

        /// <summary>RTU's t1.5, s: 1.5 characters, and 750 us above 19 200 baud.</summary>
        private double Interchar => BaudRate > 19200 ? 0.00075 : 1.5 * BitsPerCharacter / BaudRate;

        /// <summary>RTU's t3.5, s: 3.5 characters, and 1.75 ms above 19 200 baud.</summary>
        private double Turnaround => BaudRate > 19200 ? 0.00175 : 3.5 * BitsPerCharacter / BaudRate;

        private double Now()
        {
            return machine.LocalTimeSource.ElapsedVirtualTime.TotalSeconds;
        }

        private readonly IMachine machine;
        private readonly LimitTimer timer;
        private readonly Queue<Pending> queue = new Queue<Pending>();
        private double lastBus = double.MinValue / 2;
        private double lastSent = double.MinValue / 2;
        private double lastHost = double.MinValue / 2;

        /// <summary>What a node is given past t3.5 to act on a frame, s: the boot master's
        /// CHUNK_S of 2 ms less t3.5.</summary>
        private const double Act = 0.00025;

        private struct Pending
        {
            public byte Value;
            public bool Starts;
        }

        // 8N1: a start bit, eight data bits, a stop bit.
        private const ulong BitsPerCharacter = 10;
    }

    /// <summary>The face on the host's socket terminal.</summary>
    public class RS485_AdapterHost : IUART, IGPIOReceiver
    {
        public RS485_AdapterHost(RS485_AdapterBus bus)
        {
            this.bus = bus;
            bus.Host = this;
        }

        public void Reset()
        {
        }

        public void OnGPIO(int number, bool value)
        {
        }

        /// <summary>A byte from the host.</summary>
        public void WriteChar(byte value)
        {
            bus.Send(value);
        }

        [field: Transient]
        public event Action<byte> CharReceived;

        public uint BaudRate => bus.BaudRate;

        public Bits StopBits => Bits.One;

        public Parity ParityBit => Parity.None;

        public void FromBus(byte value)
        {
            CharReceived?.Invoke(value);
        }

        private readonly RS485_AdapterBus bus;
    }
}
