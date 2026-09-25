// Coaxial63100_AFE.cs - The board's analog front end in the emulator: the ADC pins' volts
// from physical inputs set on the monitor (`afe DcBusVolts 24`), through the schematic's
// networks and the phase transfer LTspice gives (electronic_simulations/afe,
// host/tools/emu/afe_spice.py). AFE_ON (PB2) powers the 3.3 V reference: off, every pin
// reads exact mid-scale. Renode converts every channel single-ended, so a differential input
// is given as the volts of its code: mid-scale plus half the difference.

using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Utilities.RESD;

namespace Antmicro.Renode.Peripherals.Analog
{
    public class Coaxial63100_AFE : IGPIOReceiver
    {
        public Coaxial63100_AFE(IMachine machine)
        {
            this.machine = machine;
            Reset();
        }

        public void Reset()
        {
            Powered = false;
            PhaseUAmps = PhaseVAmps = PhaseWAmps = 0.0;
            DcBusVolts = GateVolts = 0.0;
            Rail5Volts = 5.0;
            NtcCelsius = DieCelsius = 25.0;
            noise = new Random(NoiseSeed);
            // The die sensor's factory points, where __LL_ADC_CALC_TEMPERATURE reads them.
            machine.SystemBus.WriteWord(TsCal1Address, (ushort)DieCode(30.0));
            machine.SystemBus.WriteWord(TsCal2Address, (ushort)DieCode(110.0));
        }

        public void OnGPIO(int number, bool value)
        {
            Powered = value;
        }

        /// <summary>The volts the ADC converts on `signal`'s pin.</summary>
        public double PinVolts(string signal)
        {
            if(!Powered)
            {
                return MidVolts;
            }
            var volts = Nominal(signal) + Gauss() * NoiseVolts;
            return Math.Max(0.0, Math.Min(ReferenceVolts, volts));
        }

        public bool Powered { get; set; }

        // The physical inputs.
        public double PhaseUAmps { get; set; }
        public double PhaseVAmps { get; set; }
        public double PhaseWAmps { get; set; }
        public double DcBusVolts { get; set; }
        public double NtcCelsius { get; set; }
        public double Rail5Volts { get; set; }
        public double GateVolts { get; set; }
        public double DieCelsius { get; set; }

        // The networks: the schematic's values (board_cal.c's defaults trace them).
        public double ReferenceVolts { get; set; } = 3.3;
        public double BusTopOhms { get; set; } = 49900.0;
        public double BusBottomOhms { get; set; } = 2200.0;
        public double NtcR25Ohms { get; set; } = 10000.0;
        public double NtcBetaKelvin { get; set; } = 3380.0;
        public double NtcFixedOhms { get; set; } = 10000.0;
        public double Rail5TopOhms { get; set; } = 10000.0;
        public double Rail5BottomOhms { get; set; } = 10000.0;
        public double GateTopOhms { get; set; } = 57000.0;
        public double GateBottomOhms { get; set; } = 10000.0;

        // The phase transfer at the ADC's differential input: LTspice's (afe_spice.py).
        public double PhaseVoltsPerAmp { get; set; }
        public double PhaseZeroVolts { get; set; }

        // The die sensor: STM32H753 datasheet typicals, 0.62 V at 30 C, 2.0 mV/K.
        public double DieVoltsAt30 { get; set; } = 0.62;
        public double DieVoltsPerKelvin { get; set; } = 0.002;

        // One sigma at the pin; LTspice's spread at the sample instant (afe_spice.py).
        public double NoiseVolts { get; set; } = 0.0001;
        public int NoiseSeed { get; set; } = 63100;

        private double Nominal(string signal)
        {
            switch(signal)
            {
                case "Phase U": return Differential(PhaseZeroVolts + PhaseVoltsPerAmp * PhaseUAmps);
                case "Phase V": return Differential(PhaseZeroVolts + PhaseVoltsPerAmp * PhaseVAmps);
                case "Phase W": return Differential(PhaseZeroVolts + PhaseVoltsPerAmp * PhaseWAmps);
                case "DC bus":  return DcBusVolts * BusBottomOhms / (BusTopOhms + BusBottomOhms);
                case "NTC":     return ReferenceVolts * NtcFixedOhms / (NtcOhms() + NtcFixedOhms);
                case "+5V":     return Rail5Volts * Rail5BottomOhms / (Rail5TopOhms + Rail5BottomOhms);
                case "Vgate":   return GateVolts * GateBottomOhms / (GateTopOhms + GateBottomOhms);
                case "MCU die": return DieVoltsAt30 + (DieCelsius - 30.0) * DieVoltsPerKelvin;
                default:        return 0.0;          // Clevel, Cinj: not modelled
            }
        }

        private double Differential(double volts)
        {
            return MidVolts + volts / 2.0;
        }

        private double NtcOhms()
        {
            var kelvin = NtcCelsius + KelvinAtZero;
            return NtcR25Ohms * Math.Exp(NtcBetaKelvin * (1.0 / kelvin - 1.0 / (25.0 + KelvinAtZero)));
        }

        private int DieCode(double celsius)
        {
            var volts = DieVoltsAt30 + (celsius - 30.0) * DieVoltsPerKelvin;
            return (int)Math.Round(volts / ReferenceVolts * FullScaleCode);
        }

        private double Gauss()
        {
            var u1 = 1.0 - noise.NextDouble();
            var u2 = noise.NextDouble();
            return Math.Sqrt(-2.0 * Math.Log(u1)) * Math.Cos(2.0 * Math.PI * u2);
        }

        private double MidVolts => ReferenceVolts * 32768.0 / FullScaleCode;

        private Random noise;
        private readonly IMachine machine;

        private const double KelvinAtZero = 273.15;
        private const double FullScaleCode = 65535.0;
        private const ulong TsCal1Address = 0x1FF1E820;
        private const ulong TsCal2Address = 0x1FF1E840;
    }

    /// <summary>One ADC input wired to the front end: registered on its ADC at its channel.</summary>
    public class Coaxial63100_AFEChannel : IRESDSampleSource<VoltageSample>
    {
        public Coaxial63100_AFEChannel(Coaxial63100_AFE afe, string signal)
        {
            this.afe = afe;
            this.signal = signal;
        }

        public void Reset()
        {
        }

        public VoltageSample Sample => new VoltageSample((uint)Math.Round(afe.PinVolts(signal) * 1e6));

        private readonly Coaxial63100_AFE afe;
        private readonly string signal;
    }
}
