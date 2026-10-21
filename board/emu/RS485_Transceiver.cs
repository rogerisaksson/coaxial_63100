// RS485_Transceiver.cs - A half-duplex RS485 transceiver with its receiver enabled, between its
// UART and the bus: what the UART sends comes straight back - the local echo USART2 and UART5
// hear on the board - and goes onto the bus; what the bus carries reaches the UART. Alone, it
// is joined to nothing; on a limb (coaxial_63100_limb.resc) every node's is on one hub. It
// hangs on its UART's DE pin; Renode's USART does not drive DE, so nothing waits for it.

using System;
using System.Collections.Generic;
using Antmicro.Migrant;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Miscellaneous;

namespace Antmicro.Renode.Peripherals.UART
{
    public class RS485_Transceiver : IUART, IGPIOReceiver
    {
        public RS485_Transceiver(IMachine machine, IUART uart)
        {
            this.uart = uart;
            clock = VirtualClock.Of(machine);
            rates = new UART_RateCache(machine, uart);
            uart.CharReceived += Transmitted;
            lock(peers)
            {
                peers.Add(this);
            }
        }

        /// <summary>The bus's rate, bits a second: the host adapter's (RS485_Adapter.cs), 0 with
        /// none.</summary>
        public static uint BusRate { get; set; }

        /// <summary>Bytes on the way to the host adapter, in the order they went onto the bus.</summary>
        public static readonly Queue<byte> AdapterRx = new Queue<byte>();

        /// <summary>A byte queued in FIFO order for each peer on the hub before UARTHub schedules
        /// its TimeDomainEvents: Renode's min-heap pops equal quantum timestamps out of order once
        /// three or more bytes fall in one quantum.</summary>
        public static void Enqueue(RS485_Transceiver sender, byte value)
        {
            lock(peers)
            {
                foreach(var peer in peers)
                {
                    if(peer != sender && peer.CharReceived != null)
                    {
                        peer.rx.Enqueue(value);
                    }
                }
                if(sender != null)
                {
                    lock(AdapterRx)
                    {
                        AdapterRx.Enqueue(value);
                    }
                }
            }
        }

        public void Reset()
        {
            lock(peers)
            {
                rx.Clear();
            }
        }

        public void OnGPIO(int number, bool value)
        {
        }

        /// <summary>A byte off the bus, into the UART.</summary>
        public void WriteChar(byte value)
        {
            lock(peers)
            {
                if(rx.Count > 0)
                {
                    value = rx.Dequeue();
                }
            }
            if(Decodes())
            {
                clock.TouchUart();
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
            clock.TouchUart();
            uart.WriteChar(value);
            if(Decodes() && CharReceived != null)
            {
                Enqueue(this, value);
                CharReceived.Invoke(value);
            }
        }

        /// <summary>Whether a character crosses between the UART and the bus: its stop bit,
        /// sampled 9.5 bits past the start edge, inside half a bit less the majority vote's two
        /// samples - 2.6 % at OVER8, 3.9 % at OVER16. Renode's UARTs pass a byte whatever the
        /// rates: the app's 115 200 answered a 10 Mbit adapter.</summary>
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

        private static readonly List<RS485_Transceiver> peers = new List<RS485_Transceiver>();
        private readonly Queue<byte> rx = new Queue<byte>();
        private readonly IUART uart;
        private readonly VirtualClock clock;
        private readonly UART_RateCache rates;
        private bool decodes = true;
    }
}
