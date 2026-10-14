// Coaxial63100_AFE.cs - The board's analog front end in the emulator: world_parts.c in the
// world library, the ADC pins' volts from physical inputs set on the monitor (`afe DcBusVolts
// 24`), through the schematic's networks and the phase transfer LTspice gives
// (electronic_simulations/afe, host/tools/emu/afe_spice.py). AFE_ON (PB2) powers the 3.3 V
// reference: off, every pin reads exact mid-scale. Renode converts every channel single-ended,
// so a differential input is given as the volts of its code: mid-scale plus half the difference.

using System;
using System.Runtime.InteropServices;
using Antmicro.Renode.Core;

namespace Antmicro.Renode.Peripherals.Analog
{
    public class Coaxial63100_AFE : IGPIOReceiver
    {
        public Coaxial63100_AFE(IMachine machine)
        {
            this.machine = machine;
            Reset();
        }

        public event Action<IntPtr, int> Bound;

        public void Bind(IntPtr library, int nodeIndex)
        {
            if(native == IntPtr.Zero)
            {
                native = library;
                afeInit = Export<AfeNode>("emu_afe_init");
                afeReset = Export<AfeNode>("emu_afe_reset");
                afeDraw = Export<AfeDraw>("emu_afe_draw");
                afeSet = Export<AfeSet>("emu_afe_set");
                afeVolts = Export<AfeVolts>("emu_afe_volts");
                afeDieVolts = Export<AfeDieVolts>("emu_afe_die_volts");
            }
            node = nodeIndex;
            bound = true;
            afeInit(node);
            Sync();
            WriteFactoryCalibration();
            Bound?.Invoke(library, node);
        }

        public void Reset()
        {
            powered = false;
            phaseUAmps = phaseVAmps = phaseWAmps = 0.0;
            dcBusVolts = 0.0;
            gateVolts = 0.0;
            clevelVolts = cinjVolts = 0.0;
            rail5Volts = 5.0;
            ntcCelsius = dieCelsius = AngleCelsius = 25.0;
            if(bound)
            {
                afeReset(node);
                Sync();
            }
            WriteFactoryCalibration();
        }

        public void OnGPIO(int number, bool value)
        {
            Powered = value;
        }

        /// <summary>The volts the ADC converts on `pin` (world_pin_t).</summary>
        public double PinVolts(int pin)
        {
            return bound ? afeVolts(node, pin) : referenceVolts * 32768.0 / FullScaleCode;
        }

        public double PinVolts(string signal)
        {
            return PinVolts(PinOf(signal));
        }

        public static int PinOf(string signal)
        {
            switch(signal)
            {
                case "Phase U": return 1;
                case "Phase V": return 2;
                case "Phase W": return 3;
                case "DC bus":  return 4;
                case "NTC":     return 5;
                case "+5V":     return 6;
                case "Vgate":   return 7;
                case "MCU die": return 8;
                case "Clevel":  return 9;
                case "Cinj":    return 10;
                default:        return 0;
            }
        }

        public bool Powered
        {
            get => powered;
            set { powered = value; Set(0, value ? 1.0 : 0.0); }
        }

        // The physical inputs.
        public double PhaseUAmps
        {
            get => phaseUAmps;
            set { phaseUAmps = value; Set(1, value); }
        }
        public double PhaseVAmps
        {
            get => phaseVAmps;
            set { phaseVAmps = value; Set(2, value); }
        }
        public double PhaseWAmps
        {
            get => phaseWAmps;
            set { phaseWAmps = value; Set(3, value); }
        }
        public double DcBusVolts
        {
            get => dcBusVolts;
            set { dcBusVolts = value; Set(4, value); }
        }
        public double NtcCelsius
        {
            get => ntcCelsius;
            set { ntcCelsius = value; Set(5, value); }
        }
        public double DieCelsius
        {
            get => dieCelsius;
            set { dieCelsius = value; Set(6, value); }
        }
        public double Rail5Volts
        {
            get => rail5Volts;
            set { rail5Volts = value; Set(7, value); }
        }
        public double GateVolts
        {
            get => gateVolts;
            set { gateVolts = value; Set(8, value); }
        }
        /// <summary>The A1335's die, C: its TSEN.</summary>
        public double AngleCelsius { get; set; } = 25.0;

        /// <summary>The STO chain's integrator and recovered pilot, V: its model's
        /// (Coaxial63100_STO.cs).</summary>
        public double ClevelVolts
        {
            get => clevelVolts;
            set { clevelVolts = value; Set(9, value); }
        }
        public double CinjVolts
        {
            get => cinjVolts;
            set { cinjVolts = value; Set(10, value); }
        }

        // The networks: the schematic's values (board_cal.c's defaults trace them).
        public double ReferenceVolts
        {
            get => referenceVolts;
            set { referenceVolts = value; Set(11, value); WriteFactoryCalibration(); }
        }
        public double PhaseVoltsPerAmp
        {
            get => phaseVoltsPerAmp;
            set { phaseVoltsPerAmp = value; Set(12, value); }
        }
        public double PhaseZeroVolts
        {
            get => phaseZeroVolts;
            set { phaseZeroVolts = value; Set(13, value); }
        }
        public double PhaseGainSigma
        {
            get => phaseGainSigma;
            set { phaseGainSigma = value; Set(14, value); }
        }
        public double PhaseZeroSigmaVolts
        {
            get => phaseZeroSigmaVolts;
            set { phaseZeroSigmaVolts = value; Set(15, value); }
        }
        public double BusGainSigma
        {
            get => busGainSigma;
            set { busGainSigma = value; Set(16, value); }
        }
        public double BusTopOhms
        {
            get => busTopOhms;
            set { busTopOhms = value; Set(17, value); }
        }
        public double BusBottomOhms
        {
            get => busBottomOhms;
            set { busBottomOhms = value; Set(18, value); }
        }
        public double NtcR25Ohms
        {
            get => ntcR25Ohms;
            set { ntcR25Ohms = value; Set(19, value); }
        }
        public double NtcBetaKelvin
        {
            get => ntcBetaKelvin;
            set { ntcBetaKelvin = value; Set(20, value); }
        }
        public double NtcFixedOhms
        {
            get => ntcFixedOhms;
            set { ntcFixedOhms = value; Set(21, value); }
        }
        public double Rail5TopOhms
        {
            get => rail5TopOhms;
            set { rail5TopOhms = value; Set(22, value); }
        }
        public double Rail5BottomOhms
        {
            get => rail5BottomOhms;
            set { rail5BottomOhms = value; Set(23, value); }
        }
        public double GateTopOhms
        {
            get => gateTopOhms;
            set { gateTopOhms = value; Set(24, value); }
        }
        public double GateBottomOhms
        {
            get => gateBottomOhms;
            set { gateBottomOhms = value; Set(25, value); }
        }
        public double DieVoltsAt30
        {
            get => dieVoltsAt30;
            set { dieVoltsAt30 = value; Set(26, value); WriteFactoryCalibration(); }
        }
        public double DieVoltsPerKelvin
        {
            get => dieVoltsPerKelvin;
            set { dieVoltsPerKelvin = value; Set(27, value); WriteFactoryCalibration(); }
        }
        public double NoiseVolts
        {
            get => noiseVolts;
            set { noiseVolts = value; Set(28, value); }
        }

        /// <summary>Which board this is: its errors are drawn from it, again when it is set -
        /// a limb seeds each node from its UID.</summary>
        public int NoiseSeed
        {
            get => seed;
            set
            {
                seed = value;
                if(bound)
                {
                    afeDraw(node, (uint)seed);
                }
            }
        }

        private void Set(int field, double value)
        {
            if(bound)
            {
                afeSet(node, field, value);
            }
        }

        private void Sync()
        {
            afeSet(node, 0, powered ? 1.0 : 0.0);
            afeSet(node, 1, phaseUAmps);
            afeSet(node, 2, phaseVAmps);
            afeSet(node, 3, phaseWAmps);
            afeSet(node, 4, dcBusVolts);
            afeSet(node, 5, ntcCelsius);
            afeSet(node, 6, dieCelsius);
            afeSet(node, 7, rail5Volts);
            afeSet(node, 8, gateVolts);
            afeSet(node, 9, clevelVolts);
            afeSet(node, 10, cinjVolts);
            afeSet(node, 11, referenceVolts);
            afeSet(node, 12, phaseVoltsPerAmp);
            afeSet(node, 13, phaseZeroVolts);
            afeSet(node, 14, phaseGainSigma);
            afeSet(node, 15, phaseZeroSigmaVolts);
            afeSet(node, 16, busGainSigma);
            afeSet(node, 17, busTopOhms);
            afeSet(node, 18, busBottomOhms);
            afeSet(node, 19, ntcR25Ohms);
            afeSet(node, 20, ntcBetaKelvin);
            afeSet(node, 21, ntcFixedOhms);
            afeSet(node, 22, rail5TopOhms);
            afeSet(node, 23, rail5BottomOhms);
            afeSet(node, 24, gateTopOhms);
            afeSet(node, 25, gateBottomOhms);
            afeSet(node, 26, dieVoltsAt30);
            afeSet(node, 27, dieVoltsPerKelvin);
            afeSet(node, 28, noiseVolts);
            afeDraw(node, (uint)seed);
        }

        private void WriteFactoryCalibration()
        {
            var v30 = bound ? afeDieVolts(node, 30.0) : dieVoltsAt30;
            var v110 = bound ? afeDieVolts(node, 110.0) : (dieVoltsAt30 + 80.0 * dieVoltsPerKelvin);
            machine.SystemBus.WriteWord(TsCal1Address, (ushort)Math.Round(v30 / referenceVolts * FullScaleCode));
            machine.SystemBus.WriteWord(TsCal2Address, (ushort)Math.Round(v110 / referenceVolts * FullScaleCode));
        }

        private static T Export<T>(string name) where T : Delegate
        {
            return Marshal.GetDelegateForFunctionPointer<T>(NativeLibrary.GetExport(native, name));
        }

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void AfeNode(int i);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void AfeDraw(int i, uint seed);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate void AfeSet(int i, int field, double value);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate double AfeVolts(int i, int pin);
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate double AfeDieVolts(int i, double celsius);

        private static IntPtr native;
        private static AfeNode afeInit;
        private static AfeNode afeReset;
        private static AfeDraw afeDraw;
        private static AfeSet afeSet;
        private static AfeVolts afeVolts;
        private static AfeDieVolts afeDieVolts;

        private readonly IMachine machine;
        private int node;
        private bool bound;
        private bool powered;
        private double phaseUAmps;
        private double phaseVAmps;
        private double phaseWAmps;
        private double dcBusVolts;
        private double ntcCelsius = 25.0;
        private double dieCelsius = 25.0;
        private double rail5Volts = 5.0;
        private double gateVolts;
        private double clevelVolts;
        private double cinjVolts;
        private double referenceVolts = 3.3;
        private double phaseVoltsPerAmp = 0.025;
        private double phaseZeroVolts;
        private double phaseGainSigma;
        private double phaseZeroSigmaVolts;
        private double busGainSigma;
        private double busTopOhms = 49900.0;
        private double busBottomOhms = 2200.0;
        private double ntcR25Ohms = 10000.0;
        private double ntcBetaKelvin = 3380.0;
        private double ntcFixedOhms = 10000.0;
        private double rail5TopOhms = 10000.0;
        private double rail5BottomOhms = 10000.0;
        private double gateTopOhms = 57000.0;
        private double gateBottomOhms = 10000.0;
        private double dieVoltsAt30 = 0.62;
        private double dieVoltsPerKelvin = 0.002;
        private double noiseVolts = 0.0001;
        private int seed = 63100;

        private const double FullScaleCode = 65535.0;
        private const ulong TsCal1Address = 0x1FF1E820;
        private const ulong TsCal2Address = 0x1FF1E840;
    }

    /// <summary>One ADC input wired to the front end: registered on its ADC at its channel.</summary>
    public class Coaxial63100_AFEChannel : ICoaxial63100_AdcInput
    {
        public Coaxial63100_AFEChannel(Coaxial63100_AFE afe, string signal)
        {
            this.afe = afe;
            pin = Coaxial63100_AFE.PinOf(signal);
        }

        public void Reset()
        {
        }

        public double Volts => afe.PinVolts(pin);

        private readonly Coaxial63100_AFE afe;
        private readonly int pin;
    }
}
