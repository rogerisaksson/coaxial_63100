// Coaxial63100_TIM1.cs - TIM1 as board_pwm.c drives it, counted lazily: CNT and DIR worked out of
// the instructions executed when read, at the kernel clock the firmware's tree gives it
// (237.5 MHz: ARR 2375 is 50 kHz centre-aligned), and an event only while the update interrupt is
// enabled. Renode's STM32_Timer ran an event every update, ~100 000 a virtual second, 3.5 wall
// seconds of each at 100 MIPS, and at 250 MHz its period was 52.6 kHz (2026-09-25). Centre-aligned
// or edge-aligned; the update every RCR+1 over- or underflows, the first at the overflow, as
// RM0433 has it for an RCR written before the counter starts. The gate pins, PE8..PE13, read at
// the instant GPIOE's IDR is read: OCxREF of PWM mode 1 or 2 against CNT, CCxE/CCxNE, the
// polarities, MOE; dead time is not drawn. The rest of the registers are kept as written.

using System;
using System.Linq;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Peripherals.Miscellaneous;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Timers
{
    public class Coaxial63100_TIM1 : IDoubleWordPeripheral, IKnownSize
    {
        public Coaxial63100_TIM1(IMachine machine, IBusPeripheral gpio, ulong frequency = 237500000)
        {
            this.machine = machine;
            this.frequency = frequency;
            UpdateInterrupt = new GPIO();
            updates = new LimitTimer(machine.ClockSource, frequency, this, "update", limit: uint.MaxValue,
                                     workMode: WorkMode.Periodic, eventEnabled: true);
            updates.LimitReached += Updated;
            machine.SystemBus.SetHookAfterPeripheralRead<uint>(gpio,
                (value, offset) => offset == GpioIdr ? Pins(value) : value);
            Reset();
        }

        public GPIO UpdateInterrupt { get; }

        /// <summary>A register written: its offset and the value, after it took.</summary>
        public event Action<long, uint> Written;

        public long Size => 0x400;

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[Arr / 4] = 0xFFFF;
            startTicks = 0.0;
            frozen = 0;
            cleared = 0;
            flagged = false;
            updates.Enabled = false;
            UpdateInterrupt.Unset();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case Cnt:
                    return Counter();
                case Cr1:
                    return (registers[Cr1 / 4] & ~Dir) | (Down() ? Dir : 0U);
                case Sr:
                    return registers[Sr / 4] | (Uif() ? UifBit : 0U);
                default:
                    return offset >= 0 && offset < Size ? registers[offset / 4] : 0U;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
                case Cr1:
                    var was = Counting;
                    var cnt = Counter();
                    registers[Cr1 / 4] = value & ~Dir;
                    if(!was && Counting)
                    {
                        Start(cnt);
                    }
                    else if(was && !Counting)
                    {
                        frozen = cnt;
                    }
                    break;
                case Sr:
                    // rc_w0: a 0 written clears.
                    if((value & UifBit) == 0)
                    {
                        cleared = Updates();
                        flagged = false;
                    }
                    registers[Sr / 4] &= value | UifBit;
                    registers[Sr / 4] &= ~UifBit;
                    break;
                case Egr:
                    if((value & UgBit) != 0)
                    {
                        Start(0);
                        if((registers[Cr1 / 4] & UrsUdis) == 0)
                        {
                            flagged = true;
                        }
                    }
                    break;
                case Cnt:
                    if(Counting)
                    {
                        Start(value & 0xFFFF);
                    }
                    else
                    {
                        frozen = value & 0xFFFF;
                    }
                    break;
                default:
                    if(offset >= 0 && offset < Size)
                    {
                        registers[offset / 4] = value;
                    }
                    break;
            }
            switch(offset)
            {
                case Cr1:
                case Dier:
                case Egr:
                case Cnt:
                case Psc:
                case Arr:
                case Rcr:
                    Schedule();
                    Signal();
                    break;
                case Sr:
                    Signal();
                    break;
            }
            Written?.Invoke(offset, value);
        }

        private bool Counting => (registers[Cr1 / 4] & CenBit) != 0;

        private bool Centred => ((registers[Cr1 / 4] >> 5) & 3U) != 0;

        private uint Top => Math.Max(1U, registers[Arr / 4] & 0xFFFF);

        private double TickHz => frequency / (double)((registers[Psc / 4] & 0xFFFF) + 1);

        /// <summary>Counter ticks since the machine's start, on its virtual clock.</summary>
        private double Ticks()
        {
            return VirtualClock.Of(machine).Seconds * TickHz;
        }

        /// <summary>Where the count is in its period, ticks from its start.</summary>
        private double Position()
        {
            return Counting ? Ticks() - startTicks : frozen;
        }

        private double Period => Centred ? 2.0 * Top : Top + 1.0;

        private uint Counter()
        {
            if(!Counting)
            {
                return frozen;
            }
            var p = Position() % Period;
            return (uint)(Centred && p > Top ? 2.0 * Top - p : p);
        }

        private bool Down()
        {
            return Counting && Centred && Position() % Period >= Top;
        }

        private void Start(uint cnt)
        {
            startTicks = Ticks() - cnt;
            cleared = 0;
        }

        /// <summary>Over- and underflows since the start: at ARR, 2 ARR, 3 ARR centred, at every
        /// ARR+1 edge-aligned.</summary>
        private long Flows()
        {
            if(!Counting)
            {
                return 0;
            }
            var step = Centred ? (double)Top : Top + 1.0;
            return (long)Math.Floor(Position() / step);
        }

        /// <summary>Update events since the start: the first flow, then every RCR+1th.</summary>
        private long Updates()
        {
            var flows = Flows();
            return flows <= 0 ? 0 : (flows - 1) / Repetitions + 1;
        }

        private long Repetitions => (registers[Rcr / 4] & 0xFFFF) + 1;

        private bool Uif()
        {
            return flagged || Updates() > cleared;
        }

        /// <summary>The update event's timer: only while UIE is set and the count runs.</summary>
        private void Schedule()
        {
            var wanted = Counting && (registers[Dier / 4] & UieBit) != 0;
            if(!wanted)
            {
                if(updates.Enabled)
                {
                    updates.Enabled = false;
                }
                return;
            }
            var step = (Centred ? (double)Top : Top + 1.0) * Repetitions;
            var first = Centred ? (double)Top : Top + 1.0;
            var p = Position();
            var next = p < first ? first - p : step - ((p - first) % step);
            updates.Frequency = (ulong)Math.Round(TickHz);
            updates.Limit = (ulong)Math.Max(1.0, Math.Round(step));
            updates.Value = (ulong)Math.Max(1.0, Math.Round(next));
            updates.Enabled = true;
        }

        private void Updated()
        {
            // Flagged as well: at this instant the lazy count may stand a hair short of it.
            flagged = true;
            Signal();
        }

        private void Signal()
        {
            UpdateInterrupt.Set(Uif() && (registers[Dier / 4] & UieBit) != 0);
        }

        /// <summary>GPIOE's IDR with the six gate pins as the outputs stand this instant.</summary>
        private uint Pins(uint idr)
        {
            idr &= ~(0x3FU << 8);
            var moe = (registers[Bdtr / 4] & MoeBit) != 0;
            if(!moe || !Counting)
            {
                return idr;
            }
            var cnt = Counter();
            var ccer = registers[Ccer / 4];
            for(var ch = 0; ch < 3; ch++)
            {
                var reference = Reference(ch, cnt);
                var e = (ccer >> (4 * ch)) & 1U;
                var p = (ccer >> (4 * ch + 1)) & 1U;
                var ne = (ccer >> (4 * ch + 2)) & 1U;
                var np = (ccer >> (4 * ch + 3)) & 1U;
                if(e != 0 && (reference ^ (p != 0)))
                {
                    idr |= 1U << (9 + 2 * ch);
                }
                if(ne != 0 && (!reference ^ (np != 0)))
                {
                    idr |= 1U << (8 + 2 * ch);
                }
            }
            return idr;
        }

        /// <summary>OCxREF: PWM mode 1 active below CCR, mode 2 above, forced as forced.</summary>
        private bool Reference(int ch, uint cnt)
        {
            var ccmr = registers[(ch < 2 ? Ccmr1 : Ccmr2) / 4];
            var shift = ch == 1 ? 8 : 0;
            var mode = ((ccmr >> (shift + 4)) & 7U) | (((ccmr >> (shift + 16)) & 1U) << 3);
            var ccr = registers[(Ccr1 + 4 * ch) / 4] & 0xFFFF;
            switch(mode)
            {
                case 4:
                    return false;
                case 5:
                    return true;
                case 6:
                    return cnt < ccr;
                case 7:
                    return cnt >= ccr;
                default:
                    return false;
            }
        }

        private readonly IMachine machine;
        private readonly ulong frequency;
        private readonly LimitTimer updates;
        private readonly uint[] registers = new uint[0x400 / 4];
        private double startTicks;
        private uint frozen;
        private long cleared;
        private bool flagged;

        // RM0433, TIM1.
        private const long Cr1 = 0x00;
        private const long Dier = 0x0C;
        private const long Sr = 0x10;
        private const long Egr = 0x14;
        private const long Ccmr1 = 0x18;
        private const long Ccmr2 = 0x1C;
        private const long Ccer = 0x20;
        private const long Cnt = 0x24;
        private const long Psc = 0x28;
        private const long Arr = 0x2C;
        private const long Rcr = 0x30;
        private const long Ccr1 = 0x34;
        private const long Bdtr = 0x44;
        private const long GpioIdr = 0x10;
        private const uint CenBit = 1U;
        private const uint UrsUdis = 0x6U;
        private const uint Dir = 1U << 4;
        private const uint UieBit = 1U;
        private const uint UifBit = 1U;
        private const uint UgBit = 1U;
        private const uint MoeBit = 1U << 15;
    }
}
