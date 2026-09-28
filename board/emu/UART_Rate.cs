// UART_Rate.cs - The rate a USART runs at on the part: its kernel clock as the firmware set the
// RCC, over BRR as the part reads it. Renode's USART keeps one fixed clock (the app's 118.75 MHz,
// coaxial_63100.repl; the bootloader's is 80) and reads an OVER8 BRR raw: the bootloader's 0x10
// at 14.8 Mbit against the part's 10 (2026-09-28).

using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.UART
{
    public static class UART_Rate
    {
        /// <summary>`uart`'s rate, bits a second; 0 while BRR is unset or its kernel clock is not
        /// PCLK1.</summary>
        public static double Of(IMachine machine, IUART uart)
        {
            var regs = uart as IDoubleWordPeripheral;
            if(regs == null)
            {
                return 0;
            }
            var brr = regs.ReadDoubleWord(Brr) & 0xFFFFu;
            var kernel = Pclk1(machine);
            if(brr == 0 || kernel == 0)
            {
                return 0;
            }
            if(Oversampling(uart) == 16)
            {
                return kernel / brr;
            }
            // RM0433: BRR[2:0] holds USARTDIV[3:1] at OVER8.
            return 2.0 * kernel / ((brr & 0xFFF0u) | ((brr & 0x7u) << 1));
        }

        /// <summary>Samples a bit: 8 with CR1's OVER8 set, else 16.</summary>
        public static int Oversampling(IUART uart)
        {
            var regs = uart as IDoubleWordPeripheral;
            return regs != null && (regs.ReadDoubleWord(Cr1) & Over8) != 0 ? 8 : 16;
        }

        /// <summary>PCLK1, Hz, from the RCC's registers: SYSCLK over D1CPRE, HPRE and D2PPRE1; 0
        /// where USART2/3/5's kernel clock is another than PCLK1.</summary>
        private static double Pclk1(IMachine machine)
        {
            var bus = machine.SystemBus;
            if((bus.ReadDoubleWord(Rcc + 0x54) & 0x7u) != 0)             // D2CCIP2R.USART234578SEL
            {
                return 0;
            }
            var cr = bus.ReadDoubleWord(Rcc + 0x00);
            var hsi = 64e6 / (1u << (int)((cr >> 3) & 0x3u));
            double sysclk;
            switch((bus.ReadDoubleWord(Rcc + 0x10) >> 3) & 0x7u)            // CFGR.SWS
            {
                case 0: sysclk = hsi; break;
                case 1: sysclk = Csi; break;
                case 2: sysclk = Hse; break;
                default: sysclk = Pll1P(bus, hsi); break;
            }
            var d1 = bus.ReadDoubleWord(Rcc + 0x18);
            var d2 = bus.ReadDoubleWord(Rcc + 0x1C);
            return sysclk / Ahb(d1 >> 8) / Ahb(d1) / Apb(d2 >> 4);
        }

        private static double Pll1P(IBusController bus, double hsi)
        {
            var sel = bus.ReadDoubleWord(Rcc + 0x28);                        // PLLCKSELR
            var m = (sel >> 4) & 0x3Fu;
            double source;
            switch(sel & 0x3u)
            {
                case 0: source = hsi; break;
                case 1: source = Csi; break;
                case 2: source = Hse; break;
                default: return 0;
            }
            if(m == 0)
            {
                return 0;
            }
            var div = bus.ReadDoubleWord(Rcc + 0x30);                        // PLL1DIVR
            var frac = (bus.ReadDoubleWord(Rcc + 0x2C) & 0x1u) != 0          // PLLCFGR.PLL1FRACEN
                ? ((bus.ReadDoubleWord(Rcc + 0x34) >> 3) & 0x1FFFu) / 8192.0 : 0.0;
            return source / m * ((div & 0x1FFu) + 1 + frac) / (((div >> 9) & 0x7Fu) + 1);
        }

        // D1CPRE and HPRE: 0xxx /1, 1000 /2 .. 1111 /512, 32 skipped.
        private static double Ahb(uint field)
        {
            var f = (int)(field & 0xFu);
            return f < 8 ? 1 : 1 << (f - 7 + (f >= 12 ? 1 : 0));
        }

        // D2PPRE1: 0xx /1, 100 /2 .. 111 /16.
        private static double Apb(uint field)
        {
            var f = (int)(field & 0x7u);
            return f < 4 ? 1 : 1 << (f - 3);
        }

        private const long Cr1 = 0x00;
        private const long Brr = 0x0C;
        private const uint Over8 = 1u << 15;
        private const ulong Rcc = 0x58024400;
        // The board's crystal (coaxial_63100.repl), and the CSI.
        private const double Hse = 25e6;
        private const double Csi = 4e6;
    }
}
