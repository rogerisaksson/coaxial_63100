// RS485_Adapter.cs - The host's USB-RS485 adapter on an emulated limb: its host face on a socket
// terminal, its bus face on the limb's hub. The host's bytes reach the bus as HostLine
// (UART_Line.cs) paces them; the bus's reach the host as they come. Renode joins a terminal to a
// UART only, hence two faces.

using System;
using Antmicro.Migrant;
using Antmicro.Renode.Core;

namespace Antmicro.Renode.Peripherals.UART
{
    /// <summary>The face on the limb's hub.</summary>
    public class RS485_AdapterBus : IUART, IGPIOReceiver
    {
        public RS485_AdapterBus(IMachine machine, uint baudRate = 115200)
        {
            BaudRate = baudRate;
            line = new HostLine(machine, this, () => BaudRate, value => CharReceived?.Invoke(value));
        }

        public void Reset()
        {
            line.Reset();
        }

        public void OnGPIO(int number, bool value)
        {
        }

        /// <summary>A byte off the bus, to the host.</summary>
        public void WriteChar(byte value)
        {
            line.Heard();
            Host?.FromBus(value);
        }

        [field: Transient]
        public event Action<byte> CharReceived;

        /// <summary>The bus's rate, bits a second: 115 200 as the app starts, 10 000 000 as the
        /// bootloader runs. The nodes' transceivers decode against it.</summary>
        public uint BaudRate
        {
            get => RS485_Transceiver.BusRate;
            set => RS485_Transceiver.BusRate = value;
        }

        public Bits StopBits => Bits.One;

        public Parity ParityBit => Parity.None;

        public RS485_AdapterHost Host { get; set; }

        /// <summary>A byte of the host's framed stream, onto the bus at the bus's pace.</summary>
        public void Send(byte value)
        {
            line.Receive(value);
        }

        private readonly HostLine line;
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
