// STM32H7_ADC20.cs - Renode's STM32H7_ADC with the H7's twenty inputs, INP0..INP19: Renode
// 1.17.0 gives it nineteen, and a conversion on channel 19 (Vgate, PA5) throws inside
// Renode. Copyright (c) 2010-2026 Antmicro, MIT (Renode's licenses/MIT.txt).

using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.DMA;

namespace Antmicro.Renode.Peripherals.Analog
{
    public class STM32H7_ADC20 : STM32_ADC_Common
    {
        public STM32H7_ADC20(IMachine machine, double referenceVoltage, uint externalEventFrequency, int dmaChannel = 0, IDMA dmaPeripheral = null)
            : base(
                machine,
                referenceVoltage,
                externalEventFrequency,
                dmaChannel,
                dmaPeripheral,
                // Base class configuration
                watchdogCount: 3,
                hasCalibration: true,
                channelCount: 20,
                hasPrescaler: true,
                hasVbatPin: true,
                hasChannelSelect: false,
                hasChannelSequence: true,
                hasPowerRegister: false,
                hasOffset: true,
                hasDifferentialMode: true,
                samplingTime: SamplingTime.PerChannel,
                dualMode: true,
                hasLinearityCalibration: true,
                hasChannelInjection: true,
                hasSeparateThresholdRegisters: true,
                resolutionRange: ResolutionRange.Bits8_16,
                hasChannelPreselection: true,
                hasScanDirection: false
            )
        { }
    }
}
