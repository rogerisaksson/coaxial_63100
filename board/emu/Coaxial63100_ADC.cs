// Coaxial63100_ADC.cs - The H7's ADC as this firmware drives it, in place of Renode's: 16-bit
// conversions of the front end's pins (ICoaxial63100_AdcInput, registered at a channel), the
// regular sequence on ADSTART, the injected one on JADSTART by software or on TIM1's TRGO2 -
// the plant fires it once a PWM period - its end raising the interrupt as IER enables it.
// Renode 1.17.0's STM32H7_ADC gives nineteen inputs (a conversion on 19 throws), drops ADC2's
// registers at +0x100 and keeps no JEXTSEL or JEXTEN in JSQR, so HAL's InjectedStart refused
// (2026-09-25). A conversion takes no time. A differential input is its pin's volts as the
// front end gives them, 1.65 V + Vdiff/2: the same code the part makes of Vdiff.

using System;
using System.Collections.Generic;
using System.Linq;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Analog
{
    /// <summary>What an ADC input reads: its pin's volts.</summary>
    public interface ICoaxial63100_AdcInput : IPeripheral
    {
        double Volts { get; }
    }

    public class Coaxial63100_ADC : IDoubleWordPeripheral, IKnownSize,
        IPeripheralContainer<ICoaxial63100_AdcInput, NumberRegistrationPoint<int>>
    {
        public Coaxial63100_ADC(IMachine machine, double referenceVoltage = 3.3)
        {
            this.machine = machine;
            reference = referenceVoltage;
            IRQ = new GPIO();
            Reset();
        }

        public GPIO IRQ { get; }

        public long Size => 0x100;

        public void Register(ICoaxial63100_AdcInput input, NumberRegistrationPoint<int> at)
        {
            machine.RegisterAsAChildOf(this, input, at);
            inputs[at.Address] = input;
        }

        public IEnumerable<NumberRegistrationPoint<int>> GetRegistrationPoints(ICoaxial63100_AdcInput input)
        {
            return inputs.Where(pair => pair.Value == input)
                         .Select(pair => new NumberRegistrationPoint<int>(pair.Key)).ToList();
        }

        public IEnumerable<IRegistered<ICoaxial63100_AdcInput, NumberRegistrationPoint<int>>> Children =>
            inputs.Select(pair => Registered.Create(pair.Value, new NumberRegistrationPoint<int>(pair.Key))).ToList();

        public void Unregister(ICoaxial63100_AdcInput input)
        {
            machine.UnregisterAsAChildOf(this, input);
            foreach(var pair in new List<KeyValuePair<int, ICoaxial63100_AdcInput>>(inputs))
            {
                if(pair.Value == input)
                {
                    inputs.Remove(pair.Key);
                }
            }
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[Cr / 4] = DeepPowerDown;
            injectedArmed = false;
            IRQ.Unset();
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset < 0 || offset >= Size)
            {
                return 0;
            }
            var value = registers[offset / 4];
            if(offset == Dr)
            {
                registers[Isr / 4] &= ~EndOfConversion;     // read clears EOC
                Update();
            }
            return value;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
                case Isr:
                    registers[Isr / 4] &= ~value;
                    break;
                case Cr:
                    Control(value);
                    Arming?.Invoke();
                    break;
                case Dr:
                case Jdr1:
                case Jdr1 + 4:
                case Jdr1 + 8:
                case Jdr1 + 12:
                    break;
                default:
                    if(offset >= 0 && offset < Size)
                    {
                        registers[offset / 4] = value;
                    }
                    break;
            }
            Update();
        }

        /// <summary>Whether the injected sequence waits on TIM1's TRGO2.</summary>
        public bool WaitsOnTrgo2
        {
            get
            {
                var jsqr = registers[Jsqr / 4];
                return injectedArmed && ((jsqr >> 7) & 3U) != 0 && ((jsqr >> 2) & 0x1FU) == Tim1Trgo2;
            }
        }

        /// <summary>Told when WaitsOnTrgo2 may have changed: the trigger's source runs only then.</summary>
        public event Action Arming;

        /// <summary>TIM1's TRGO2: the injected sequence, where JSQR waits on it.</summary>
        public void OnTrgo2()
        {
            if(WaitsOnTrgo2)
            {
                Injected();
                Update();
            }
        }

        private void Control(uint value)
        {
            var cr = registers[Cr / 4];
            // What the part keeps: the regulator, deep power-down, boost and the calibration's modes.
            cr = (cr & ~Kept) | (value & Kept);
            if((value & Enable) != 0)
            {
                cr |= Enable;
                registers[Isr / 4] |= Ready;
            }
            if((value & Disable) != 0)
            {
                cr &= ~(Enable | InjectedStart);
                injectedArmed = false;
            }
            if((cr & RegulatorOn) != 0)
            {
                registers[Isr / 4] |= RegulatorReady;
            }
            // Calibration, ADSTP and JADSTP are done as soon as asked.
            if((value & InjectedStop) != 0)
            {
                cr &= ~InjectedStart;
                injectedArmed = false;
            }
            registers[Cr / 4] = cr;
            if((value & RegularStart) != 0 && (cr & Enable) != 0)
            {
                Regular();
            }
            if((value & InjectedStart) != 0 && (cr & Enable) != 0)
            {
                if(((registers[Jsqr / 4] >> 7) & 3U) == 0)
                {
                    Injected();                 // by software: once, and JADSTART clears
                }
                else
                {
                    injectedArmed = true;
                    registers[Cr / 4] |= InjectedStart;
                }
            }
        }

        private void Regular()
        {
            var length = (int)(registers[Sqr1 / 4] & 0xFU) + 1;
            for(var rank = 1; rank <= length; rank++)
            {
                registers[Dr / 4] = Convert(RegularChannel(rank));
                registers[Isr / 4] |= EndOfSampling | EndOfConversion;
            }
            registers[Isr / 4] |= EndOfSequence;
        }

        private void Injected()
        {
            var jsqr = registers[Jsqr / 4];
            var length = (int)(jsqr & 3U) + 1;
            for(var rank = 0; rank < length; rank++)
            {
                registers[Jdr1 / 4 + rank] = Convert((int)((jsqr >> (9 + 6 * rank)) & 0x1FU));
            }
            registers[Isr / 4] |= InjectedEndOfConversion | InjectedEndOfSequence;
        }

        /// <summary>SQ1..SQ16: four ranks in SQR1 from bit 6, five in each of the rest.</summary>
        private int RegularChannel(int rank)
        {
            var register = rank < 5 ? Sqr1 : Sqr1 + 4 * ((rank - 5) / 5 + 1);
            var shift = rank < 5 ? 6 * rank : 6 * ((rank - 5) % 5);
            return (int)((registers[register / 4] >> shift) & 0x1FU);
        }

        /// <summary>The input's code at the resolution RES sets: 16, 14, 12, 10 or 8 bits, in
        /// rev Y's codes or rev V's optimised ones.</summary>
        private uint Convert(int channel)
        {
            ICoaxial63100_AdcInput input;
            var volts = inputs.TryGetValue(channel, out input) ? input.Volts : 0.0;
            var code = Math.Max(0.0, Math.Min(65535.0, Math.Round(volts / reference * 65536.0)));
            var bits = Resolutions[(registers[Cfgr / 4] >> 2) & 7U];
            return (uint)code >> (16 - bits);
        }

        private void Update()
        {
            IRQ.Set((registers[Isr / 4] & registers[Ier / 4] & 0x7FFU) != 0);
        }

        private readonly IMachine machine;
        private readonly double reference;
        private readonly uint[] registers = new uint[0x100 / 4];
        private readonly Dictionary<int, ICoaxial63100_AdcInput> inputs = new Dictionary<int, ICoaxial63100_AdcInput>();
        private bool injectedArmed;

        // RM0433 (rev V): the registers, CR's and ISR's bits, CFGR.RES's codes.
        private const long Isr = 0x00;
        private const long Ier = 0x04;
        private const long Cr = 0x08;
        private const long Cfgr = 0x0C;
        private const long Sqr1 = 0x30;
        private const long Dr = 0x40;
        private const long Jsqr = 0x4C;
        private const long Jdr1 = 0x80;

        private const uint Enable = 1U << 0;
        private const uint Disable = 1U << 1;
        private const uint RegularStart = 1U << 2;
        private const uint InjectedStart = 1U << 3;
        private const uint InjectedStop = 1U << 5;
        private const uint RegulatorOn = 1U << 28;
        private const uint DeepPowerDown = 1U << 29;
        private const uint Kept = (3U << 8) | (1U << 16) | (0x3FU << 22) | RegulatorOn | DeepPowerDown | (1U << 30);

        private const uint Ready = 1U << 0;
        private const uint EndOfSampling = 1U << 1;
        private const uint EndOfConversion = 1U << 2;
        private const uint EndOfSequence = 1U << 3;
        private const uint InjectedEndOfConversion = 1U << 5;
        private const uint InjectedEndOfSequence = 1U << 6;
        private const uint RegulatorReady = 1U << 12;

        /// <summary>JEXTSEL's code for TIM1's TRGO2 (ADC_EXTERNALTRIGINJEC_T1_TRGO2).</summary>
        private const uint Tim1Trgo2 = 8;

        private static readonly int[] Resolutions = { 16, 14, 12, 10, 8, 14, 12, 8 };
    }
}
