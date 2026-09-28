// RS485_Transceiver.cs - A half-duplex RS485 transceiver with its receiver enabled, between its
// UART and the bus: what the UART sends comes straight back - the local echo USART2 and UART5
// hear on the board - and goes onto the bus; what the bus carries reaches the UART. Alone, it
// is joined to nothing; on a limb (coaxial_63100_limb.resc) every node's is on one hub. It
// hangs on its UART's DE pin; Renode's USART does not drive DE, so nothing waits for it.

using System;
using Antmicro.Migrant;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;

namespace Antmicro.Renode.Peripherals.UART
{
    public class RS485_Transceiver : IUART, IGPIOReceiver
    {
        public RS485_Transceiver(IMachine machine, IUART uart)
        {
            this.uart = uart;
            rates = new UART_RateCache(machine, uart);
            uart.CharReceived += Transmitted;
        }

        /// <summary>The bus's rate, bits a second: the host adapter's (RS485_Adapter.cs), 0 with
        /// none.</summary>
        public static uint BusRate { get; set; }

        public void Reset()
        {
        }

        public void OnGPIO(int number, bool value)
        {
        }

        /// <summary>A byte off the bus, into the UART.</summary>
        public void WriteChar(byte value)
        {
            if(Decodes())
            {
                uart.WriteChar(value);
            }
        }

        [field: Transient]
        public event Action<byte> CharReceived;

        public uint BaudRate => uart.BaudRate;

        public Bits StopBits => uart.StopBits;

        public Parity ParityBit => uart.ParityBit;

        private void Transmitted(byte value)
        {
            uart.WriteChar(value);
            if(Decodes())
            {
                CharReceived?.Invoke(value);
            }
        }

        /// <summary>Whether a character crosses between the UART and the bus: its stop bit,
        /// sampled 9.5 bits past the start edge, inside half a bit less the majority vote's two
        /// samples - 2.6 % at OVER8, 3.9 % at OVER16. Renode's UARTs pass a byte whatever the
        /// rates: the app's 115 200 answered a 10 Mbit adapter (2026-09-28).</summary>
        private bool Decodes()
        {
            var rate = rates.Rate;
            if(BusRate == 0 || rate == 0)
            {
                return true;
            }
            var ok = Math.Abs(rate / BusRate - 1.0)
                     < (0.5 - 2.0 / rates.Oversampling) / 9.5;
            if(ok != decodes)
            {
                decodes = ok;
                this.Log(ok ? LogLevel.Info : LogLevel.Warning, "{0:F0} bit/s on a {1} bit/s bus: {2}",
                         rate, BusRate, ok ? "decodes" : "garbles");
            }
            return ok;
        }

        private readonly IUART uart;
        private readonly UART_RateCache rates;
        private bool decodes = true;
    }
}
