// UART_Line.cs - The wire between a terminal and a UART: the terminal's bytes reach the UART one
// character time apart in virtual time, at the UART's own baud rate, as on the board. Renode's
// terminals hand over a whole TCP chunk at once, which a 257-byte ADU into a 256-byte ring
// shows (2026-09-25). It hangs on the UART's RX pin.

using System;
using System.Collections.Generic;
using Antmicro.Migrant;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.UART
{
    public class UART_Line : IUART, IGPIOReceiver
    {
        public UART_Line(IMachine machine, IUART uart)
        {
            this.uart = uart;
            uart.CharReceived += value => CharReceived?.Invoke(value);
            timer = new LimitTimer(machine.ClockSource, FallbackBaud, this, "character", limit: BitsPerCharacter,
                                   workMode: WorkMode.Periodic, eventEnabled: true);
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

        /// <summary>A byte from the terminal, onto the wire.</summary>
        public void WriteChar(byte value)
        {
            lock(queue)
            {
                queue.Enqueue(value);
                if(!timer.Enabled)
                {
                    timer.Frequency = uart.BaudRate != 0 ? uart.BaudRate : FallbackBaud;
                    timer.Value = BitsPerCharacter;
                    timer.Enabled = true;
                }
            }
        }

        [field: Transient]
        public event Action<byte> CharReceived;

        public uint BaudRate => uart.BaudRate;

        public Bits StopBits => uart.StopBits;

        public Parity ParityBit => uart.ParityBit;

        private void Deliver()
        {
            byte value;
            lock(queue)
            {
                value = queue.Dequeue();
                if(queue.Count == 0)
                {
                    timer.Enabled = false;
                }
            }
            uart.WriteChar(value);
        }

        private readonly IUART uart;
        private readonly LimitTimer timer;
        private readonly Queue<byte> queue = new Queue<byte>();

        // 8N1: a start bit, eight data bits, a stop bit.
        private const ulong BitsPerCharacter = 10;
        private const uint FallbackBaud = 115200;
    }
}
