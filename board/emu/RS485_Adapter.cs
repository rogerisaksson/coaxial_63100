// RS485_Adapter.cs - The host's USB-RS485 adapter on an emulated limb: its host face on a socket
// terminal, its bus face on the limb's hub. The host's bytes reach the bus one character time
// apart at the bus's baud in virtual time, as UART_Line paces a console; the bus's reach the
// host as they come. A new frame waits for the bus to have been quiet RTU's t3.5 in virtual
// time: the host keeps that gap on its own clock, and at a fraction of real time it is a far
// shorter one on the bus's - a node still purging its own echo lost the next request's first
// bytes at 10 Mbit (2026-09-25). Renode joins a terminal to a UART only, hence two faces.

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
                queue.Enqueue(value);
                if(!timer.Enabled)
                {
                    // The first character after the bus's t3.5 of quiet, then one a character time.
                    var owed = Math.Max(0.0, Turnaround - (Now() - lastBus));
                    timer.Frequency = BaudRate;
                    timer.Limit = BitsPerCharacter + (ulong)(owed * BaudRate);
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
                // Back to a character time from here: the limit, and the count the timer
                // reloaded from the turnaround's before this ran.
                timer.Limit = BitsPerCharacter;
                timer.Value = BitsPerCharacter;
                value = queue.Dequeue();
                if(queue.Count == 0)
                {
                    timer.Enabled = false;
                }
            }
            CharReceived?.Invoke(value);
        }

        /// <summary>RTU's t3.5, s: 3.5 characters, and 1.75 ms above 19 200 baud.</summary>
        private double Turnaround => BaudRate > 19200 ? 0.00175 : 3.5 * BitsPerCharacter / BaudRate;

        private double Now()
        {
            return machine.LocalTimeSource.ElapsedVirtualTime.TotalSeconds;
        }

        private readonly IMachine machine;
        private readonly LimitTimer timer;
        private readonly Queue<byte> queue = new Queue<byte>();
        private double lastBus = double.MinValue / 2;

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
