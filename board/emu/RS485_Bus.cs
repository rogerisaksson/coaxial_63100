// RS485_Bus.cs - A limb's RS485 bus: one medium, its characters in one order - the order they
// went onto it - every tap but the sender's hearing each at its machine's next synced state, in
// that order. Renode's UARTHub stamps a character with the time of the thread that sends it, and
// a machine's timer runs on whichever node's CPU thread completed the quantum: the adapter's
// characters, stamped on two machines' clocks, reached node1 out of order (findings/emulation.md).

using System;
using System.Collections.Generic;
using System.Linq;
using Antmicro.Renode.Core;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.UART
{
    public sealed class RS485_Bus
    {
        /// <summary>The bus a limb's script names, made at the first naming.</summary>
        public static RS485_Bus Named(string name)
        {
            lock(buses)
            {
                if(!buses.TryGetValue(name, out var bus))
                {
                    bus = new RS485_Bus();
                    buses[name] = bus;
                }
                return bus;
            }
        }

        /// <summary>`owner` on the bus, on `machine`: what the others put on it reaches `heard`.</summary>
        public void Attach(IMachine machine, object owner, Action<byte> heard)
        {
            lock(taps)
            {
                taps.Add(new Tap { Machine = machine, Owner = owner, Heard = heard });
            }
        }

        /// <summary>`value` onto the medium behind every character before it: each other tap
        /// hears it at its machine's next synced state behind those, a stamp of no domain being
        /// one Renode's time source takes in the order given.</summary>
        public void Put(object owner, byte value)
        {
            lock(taps)
            {
                foreach(var tap in taps.Where(tap => tap.Owner != owner))
                {
                    tap.Machine.HandleTimeDomainEvent(tap.Heard, value, new TimeStamp());
                }
            }
        }

        private struct Tap
        {
            public IMachine Machine;
            public object Owner;
            public Action<byte> Heard;
        }

        private readonly List<Tap> taps = new List<Tap>();

        private static readonly Dictionary<string, RS485_Bus> buses = new Dictionary<string, RS485_Bus>();
    }
}
