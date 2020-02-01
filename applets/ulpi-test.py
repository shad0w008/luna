#!/usr/bin/env python3
# pylint: disable=maybe-no-member
#
# This file is part of LUNA.
#

import sys
import time

from prompt_toolkit import HTML
from prompt_toolkit import print_formatted_text as pprint

from nmigen import Signal, Elaboratable, Module, Cat, ClockDomain, ClockSignal, ResetInserter
from nmigen.lib.cdc import FFSynchronizer

from luna                             import top_level_cli

from luna.apollo                      import ApolloDebugger, ApolloILAFrontend
from luna.gateware.debug.ila          import SyncSerialILA


from luna.gateware.utils.cdc          import synchronize
from luna.gateware.utils              import rising_edge_detector
from luna.gateware.architecture.car   import LunaECP5DomainGenerator
from luna.gateware.interface.spi      import SPIRegisterInterface, SPIMultiplexer, SPIBus
from luna.gateware.interface.ulpi     import UMTITranslator
from luna.gateware.usb.analyzer       import USBAnalyzer


ANALYZER_RESULT = 2


class ULPIDiagnostic(Elaboratable):
    """ Gateware that evalutes ULPI PHY functionality. """


    def elaborate(self, platform):
        m = Module()

        # Generate our clock domains.
        clocking = LunaECP5DomainGenerator()
        m.submodules.clocking = clocking

        # Grab a reference to our debug-SPI bus.
        board_spi = synchronize(m, platform.request("debug_spi"))

        # Create our SPI-connected registers.
        m.submodules.spi_registers = spi_registers = SPIRegisterInterface(7, 32)
        m.d.comb += spi_registers.spi.connect(board_spi)

        # Create our UMTI translator.
        ulpi = platform.request("target_phy")
        m.submodules.umti = umti = UMTITranslator(ulpi=ulpi)


        # Strap our power controls to be in VBUS passthrough by default,
        # on the target port.
        m.d.comb += [
            platform.request("power_a_port")      .eq(0),
            platform.request("pass_through_vbus") .eq(1),
        ]


        # Hook up our LEDs to status signals.
        m.d.comb += [
            platform.request("led", 2)  .eq(umti.session_valid),
            platform.request("led", 3)  .eq(umti.rx_active),
            platform.request("led", 4)  .eq(umti.rx_error)
        ]

        spi_registers.add_read_only_register(1, read=umti.last_rx_command)

        # Set up our parameters.
        m.d.comb += [

            # Set our mode to non-driving and full speed.
            umti.op_mode     .eq(0b00),
            umti.xcvr_select .eq(0b01),

            # Disable the DP/DM pull resistors.
            umti.dm_pulldown .eq(0),
            umti.dm_pulldown .eq(0),
            umti.term_select .eq(0)
        ]


        # Create a USB analyzer, and connect a register up to its output.
        m.submodules.analyzer = analyzer = USBAnalyzer(umti_interface=umti)
        spi_registers.add_read_only_register(ANALYZER_RESULT, read=analyzer.data_out, read_strobe=analyzer.next)

        m.d.comb += [
            platform.request("led", 0)  .eq(analyzer.capturing),
            platform.request("led", 1)  .eq(analyzer.data_available),
            platform.request("led", 5)  .eq(analyzer.overrun)
        ]

        # Return our elaborated module.
        return m


if __name__ == "__main__":
    top_level_cli(ULPIDiagnostic)
