// RS485_Adapter.cs - The host's USB-RS485 adapter on an emulated limb: its host face on a socket
// terminal, its bus face on the limb's bus (RS485_Bus.cs). The host's bytes reach the bus as
// HostLine (UART_Line.cs) paces them; the bus's reach the host a frame at a time, framed by
// t3.5 on the machine's clock. Renode joins a terminal to a UART only, hence two faces.

using System;
using System.Collections.Generic;
using Antmicro.Migrant;
using Antmicro.Renode.Core;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.UART
{
    /// <summary>The face on the limb's bus.</summary>
    public class RS485_AdapterBus : IGPIOReceiver
    {
        public RS485_AdapterBus(IMachine machine, uint baudRate = 115200)
        {
            this.machine = machine;
            BaudRate = baudRate;
            line = new HostLine(machine, this, () => BaudRate, value => bus.Put(this, value));
            machine.LocalTimeSource.SyncHook += Framed;
        }

        /// <summary>The bus it is on, by the name a limb's script gives it.</summary>
        public string Bus
        {
            get => busName;
            set
            {
                busName = value;
                bus = RS485_Bus.Named(value);
                bus.Attach(machine, this, Heard);
            }
        }

        public void Reset()
        {
            line.Reset();
            lock(frame)
            {
                frame.Clear();
            }
        }

        public void OnGPIO(int number, bool value)
        {
        }

        /// <summary>A byte off the bus, held with its frame.</summary>
        private void Heard(byte value)
        {
            line.Heard();
            lock(frame)
            {
                frame.Add(value);
                heardAt = machine.LocalTimeSource.ElapsedVirtualTime.TotalSeconds;
            }
        }

        /// <summary>At a sync point, the bus's characters arriving at them: the frame heard handed
        /// to the host whole once the line has been quiet t3.5 behind it, as a master's receiver
        /// frames the line - on the line's clock. Handed over as they came, a reply that spanned
        /// two quanta reached the host with a quantum's wall time inside it, and the host's gap,
        /// on its own clock, cut it.</summary>
        private void Framed(TimeInterval now)
        {
            byte[] whole;
            lock(frame)
            {
                if(frame.Count == 0 || now.TotalSeconds - heardAt < line.Turnaround)
                {
                    return;
                }
                whole = frame.ToArray();
                frame.Clear();
            }
            Array.ForEach(whole, value => Host?.FromBus(value));
        }

        /// <summary>The bus's rate, bits a second: 115 200 as the app starts, 10 000 000 as the
        /// bootloader runs. The nodes' transceivers decode against it.</summary>
        public uint BaudRate
        {
            get => RS485_Transceiver.BusRate;
            set => RS485_Transceiver.BusRate = value;
        }

        public RS485_AdapterHost Host { get; set; }

        /// <summary>A byte of the host's framed stream, onto the bus at the bus's pace.</summary>
        public void Send(byte value)
        {
            line.Receive(value);
        }

        private readonly IMachine machine;
        private readonly HostLine line;
        private readonly List<byte> frame = new List<byte>();
        private RS485_Bus bus = new RS485_Bus();
        private string busName;
        private double heardAt;
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
