// UART_Line.cs - The wire between a terminal and a UART, and the host's side of an RTU line
// (HostLine), which the console's wire and the limb's adapter (RS485_Adapter.cs) both pace
// with. It hangs on the UART's RX pin.

using System;
using System.Collections.Generic;
using Antmicro.Migrant;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.UART
{
    /// <summary>A host's bytes onto an RTU line, one character time apart at the line's baud in
    /// virtual time. A frame starts where the host's bytes arrive RTU's t1.5 apart in virtual
    /// time - Renode's terminals hand a TCP chunk over at once, sometimes split across two
    /// quantum boundaries - and waits for the line to have been quiet t3.5 and a master's 0.25 ms
    /// to act on the last, either way. The host keeps those gaps on its own clock; at a fraction
    /// of real time, and quantized, they are shorter on the line's: a node still purging its own
    /// echo lost the next request's first bytes, broadcast chunks queued t3.5 apart were lost at
    /// 10 Mbit (2026-09-25), and FC05 sent 0.18 s after a 257-byte ADU reached the console 18 ms
    /// virtual later, inside that ADU's 21 ms, and joined it (2026-09-28).</summary>
    public sealed class HostLine
    {
        public HostLine(IMachine machine, IPeripheral owner, Func<uint> baud, Action<byte> deliver)
        {
            this.machine = machine;
            this.owner = owner;
            this.baud = baud;
            this.deliver = deliver;
            // Built with room for the longest turnaround: a LimitTimer's Value is held to the
            // limit it was made with, not the one it has.
            timer = new LimitTimer(machine.ClockSource, Rate(), owner, "character", limit: uint.MaxValue,
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

        /// <summary>A byte from the host, onto the line at its pace.</summary>
        public void Send(byte value)
        {
            lock(queue)
            {
                var at = Now();
                var starts = at - lastHost > Interchar;
                if(starts && at - lastHost < Split)
                {
                    // A frame this soon after the host's last byte: a write split on its way in?
                    owner.Log(LogLevel.Warning, "host frame {0:F3} ms after the host's last byte",
                              (at - lastHost) * 1e3);
                }
                queue.Enqueue(new Pending { Value = value, Starts = starts });
                lastHost = at;
                if(!timer.Enabled)
                {
                    timer.Frequency = Rate();
                    timer.Limit = BitsPerCharacter + Owed(queue.Peek().Starts);
                    timer.Value = timer.Limit;
                    timer.Enabled = true;
                }
            }
        }

        /// <summary>A byte on a half-duplex line the other way: the far end's reply.</summary>
        public void Heard()
        {
            lastHeard = Now();
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
            deliver(value);
        }

        /// <summary>Bit times a frame's first character waits past its own: what is left of
        /// t3.5 and Act since the line last carried a byte, either way.</summary>
        private ulong Owed(bool starts)
        {
            if(!starts)
            {
                return 0;
            }
            var quiet = Now() - Math.Max(lastHeard, lastSent);
            return (ulong)(Math.Max(0.0, Turnaround + Act - quiet) * Rate());
        }

        private uint Rate()
        {
            var rate = baud();
            return rate != 0 ? rate : FallbackBaud;
        }

        /// <summary>RTU's t1.5, s: 1.5 characters, and 750 us above 19 200 baud.</summary>
        private double Interchar => Rate() > 19200 ? 0.00075 : 1.5 * BitsPerCharacter / Rate();

        /// <summary>RTU's t3.5, s: 3.5 characters, and 1.75 ms above 19 200 baud.</summary>
        private double Turnaround => Rate() > 19200 ? 0.00175 : 3.5 * BitsPerCharacter / Rate();

        private double Now()
        {
            return machine.LocalTimeSource.ElapsedVirtualTime.TotalSeconds;
        }

        private readonly IMachine machine;
        private readonly IPeripheral owner;
        private readonly Func<uint> baud;
        private readonly Action<byte> deliver;
        private readonly LimitTimer timer;
        private readonly Queue<Pending> queue = new Queue<Pending>();
        private double lastHeard = double.MinValue / 2;
        private double lastSent = double.MinValue / 2;
        private double lastHost = double.MinValue / 2;

        /// <summary>What a node is given past t3.5 to act on a frame, s: the boot master's
        /// CHUNK_S of 2 ms less t3.5.</summary>
        private const double Act = 0.00025;

        /// <summary>Under this after the host's last byte a frame is suspect, s: a request comes after
        /// its predecessor's reply, milliseconds on at any rate here.</summary>
        private const double Split = 0.005;

        private struct Pending
        {
            public byte Value;
            public bool Starts;
        }

        // 8N1: a start bit, eight data bits, a stop bit.
        private const ulong BitsPerCharacter = 10;
        private const uint FallbackBaud = 115200;
    }

    /// <summary>The console's wire: the terminal's bytes onto the UART as a host's on an RTU line
    /// - Renode's terminals hand over a whole TCP chunk at once, which a 257-byte ADU into a
    /// 256-byte ring shows (2026-09-25).</summary>
    public class UART_Line : IUART, IGPIOReceiver
    {
        public UART_Line(IMachine machine, IUART uart)
        {
            this.uart = uart;
            line = new HostLine(machine, this, () =>
            {
                var rate = UART_Rate.Of(machine, uart);
                return rate > 0 ? (uint)Math.Round(rate) : uart.BaudRate;
            }, uart.WriteChar);
            // Full duplex: the UART's own bytes leave on another wire.
            uart.CharReceived += value => CharReceived?.Invoke(value);
        }

        public void Reset()
        {
            line.Reset();
        }

        public void OnGPIO(int number, bool value)
        {
        }

        /// <summary>A byte from the terminal, onto the wire.</summary>
        public void WriteChar(byte value)
        {
            line.Send(value);
        }

        [field: Transient]
        public event Action<byte> CharReceived;

        public uint BaudRate => uart.BaudRate;

        public Bits StopBits => uart.StopBits;

        public Parity ParityBit => uart.ParityBit;

        private readonly IUART uart;
        private readonly HostLine line;
    }
}
