// RS485_Transceiver.cs - A half-duplex RS485 transceiver with its receiver enabled, between its
// UART and the bus: what the UART sends comes straight back - the local echo USART2 and UART5
// hear on the board - and goes onto the bus; what the bus carries reaches the UART. Alone, it
// is joined to nothing; on a limb (coaxial_63100_limb.resc) every node's is on one hub. It
// hangs on its UART's DE pin; Renode's USART does not drive DE, so nothing waits for it.

using System;
using Antmicro.Migrant;
using Antmicro.Renode.Core;

namespace Antmicro.Renode.Peripherals.UART
{
    public class RS485_Transceiver : IUART, IGPIOReceiver
    {
        public RS485_Transceiver(IUART uart)
        {
            this.uart = uart;
            uart.CharReceived += Transmitted;
        }

        public void Reset()
        {
        }

        public void OnGPIO(int number, bool value)
        {
        }

        /// <summary>A byte off the bus, into the UART.</summary>
        public void WriteChar(byte value)
        {
            uart.WriteChar(value);
        }

        [field: Transient]
        public event Action<byte> CharReceived;

        public uint BaudRate => uart.BaudRate;

        public Bits StopBits => uart.StopBits;

        public Parity ParityBit => uart.ParityBit;

        private void Transmitted(byte value)
        {
            uart.WriteChar(value);
            CharReceived?.Invoke(value);
        }

        private readonly IUART uart;
    }
}
