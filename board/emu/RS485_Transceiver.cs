// RS485_Transceiver.cs - A half-duplex RS485 transceiver with its receiver enabled: what the
// UART sends comes straight back, the local echo USART2 and UART5 hear on the board. It hangs
// on its UART's DE pin; Renode's USART does not drive DE, so the echo does not wait for it.

using System;
using Antmicro.Migrant;
using Antmicro.Renode.Core;

namespace Antmicro.Renode.Peripherals.UART
{
    public class RS485_Transceiver : IUART, IGPIOReceiver
    {
        public void Reset()
        {
        }

        public void OnGPIO(int number, bool value)
        {
        }

        public void WriteChar(byte value)
        {
            CharReceived?.Invoke(value);
        }

        [field: Transient]
        public event Action<byte> CharReceived;

        public uint BaudRate => 0;

        public Bits StopBits => Bits.One;

        public Parity ParityBit => Parity.None;
    }
}
