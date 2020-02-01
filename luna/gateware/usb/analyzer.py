#
# This file is part of LUNA.
#
""" Low-level USB analyzer gateware. """


import unittest

from nmigen            import Signal, Module, Elaboratable, Memory
from nmigen.back.pysim import Simulator

from ..test           import LunaGatewareTestCase, ulpi_domain_test_case, sync_test_case
from ..utils          import rising_edge_detector, falling_edge_detector


class USBAnalyzer(Elaboratable):
    """ Core USB analyzer; backed by a small ringbuffer in FPGA block RAM.

    If you're looking to instantiate a full analyzer, you'll probably want to grab
    one of the DRAM-based ringbuffer variants (which are currently forthcoming).

    I/O port:
        O: data_available -- indicates that new data is available in the analysis stream
        O: data_out[8]    -- the next byte in the captured stream; valid when data_available is asserted
        I: next           -- strobe that indicates when the data_out byte has been accepted; and can be
                             discarded from the local memory
    """

    # Current, we'll provide a packet header of 16 bits.
    HEADER_SIZE_BITS = 16
    HEADER_SIZE_BYTES = HEADER_SIZE_BITS // 8

    # Support a maximum payload size of 1024B, plus a 1-byte PID and a 2-byte CRC16.
    MAX_PACKET_SIZE_BYTES = 1024 + 1 + 2

    def __init__(self, *, umti_interface, mem_depth=16384):
        """
        Parameters:
            umti_interface -- A record or elaboratable that presents a UMTI interface.
        """

        self.umti = umti_interface

        # Internal storage memory.
        self.mem = Memory(width=8, depth=mem_depth, name="analysis_ringbuffer")
        self.mem_size = mem_depth

        #
        # I/O port
        #
        self.data_available = Signal()
        self.data_out       = Signal(8)
        self.next           = Signal()


        self.overrun        = Signal()
        self.capturing      = Signal()


    def elaborate(self, platform):
        m = Module()

        # Memory read and write ports.
        m.submodules.write = mem_write_port = self.mem.write_port()
        m.submodules.read  = mem_read_port  = self.mem.read_port()

        # Store the memory address of our active packet header, which will store
        # packet metadata like the packet size.
        header_location = Signal.like(mem_write_port.addr)
        write_location  = Signal.like(mem_write_port.addr)

        # Read FIFO status.
        read_location   = Signal.like(mem_read_port.addr)
        fifo_count      = Signal.like(mem_read_port.addr, reset=0)
        fifo_new_data   = Signal()

        # Current receive status.
        packet_size     = Signal(16)

        #
        # Read FIFO logic.
        #
        m.d.comb += [

            # We have data ready whenever there's not data in the FIFO.
            self.data_available .eq(fifo_count != 0),

            # Our data_out is always the output of our read port...
            self.data_out       .eq(mem_read_port.data),

            # ... and our read port always reads from our read pointer.
            mem_read_port.addr  .eq(read_location)
        ]

        # Once our consumer has accepted our current data, move to the next address.
        with m.If(self.next & self.data_available):
            m.d.sync += read_location.eq(read_location + 1)


        #
        # FIFO count handling.
        #
        fifo_full = (fifo_count == self.mem_size)

        data_push = self.next & self.data_available
        data_pop  = fifo_new_data & ~fifo_full

        # If we have both a read and a write, don't update the count,
        # as we've both added one and subtracted one.
        with m.If(data_push & data_pop):
            pass

        # Otherwise, add when data's added, and subtract when data's removed.
        with m.Elif(data_push):
            m.d.sync += fifo_count.eq(fifo_count + 1)
        with m.Elif(data_pop):
            m.d.sync += fifo_count.eq(fifo_count - 1)


        #
        # Core analysis FSM.
        #
        with m.FSM() as f:
            m.d.comb += [
                self.overrun   .eq(f.ongoing("OVERRUN")),
                self.capturing .eq(f.ongoing("CAPTURING")),
            ]

            # IDLE: wait for an active receive.
            with m.State("IDLE"):

                # Wait until a transmission is active.
                # TODO: add triggering logic?
                with m.If(self.umti.rx_active):
                    m.next = "CAPTURE"
                    m.d.sync += [
                        header_location  .eq(write_location),
                        write_location   .eq(write_location + self.HEADER_SIZE_BYTES),
                        packet_size      .eq(0),
                    ]


            # Capture data until the packet is complete.
            with m.State("CAPTURE"):

                # Capture data whenever RxValid is asserted.
                m.d.comb += [
                    mem_write_port.addr  .eq(write_location),
                    mem_write_port.data  .eq(self.umti.data_out),
                    mem_write_port.en    .eq(self.umti.rx_valid)
                ]

                # Advance the write pointer each time we receive a bit.
                with m.If(self.umti.rx_valid):
                    m.d.sync += [
                        write_location  .eq(write_location + 1),
                        packet_size     .eq(packet_size + 1)
                    ]

                    m.d.comb += fifo_new_data.eq(1)

                    # If this would be filling up our data memory,
                    # move to the OVERRUN state.
                    with m.If(fifo_count == self.mem_size - 1):
                        m.next = "OVERRUN"

                # If we've stopped receiving, move to the "finalize" state.
                with m.If(~self.umti.rx_active):
                    m.next = "EOP_1"

            # EOP: handle the end of the relevant packet.
            with m.State("EOP_1"):

                # Now that we're done, add the header to the start of our packet.
                # This will take two cycles, currently, as we're using a 2-byte header,
                # but we only have an 8-bit write port.
                m.d.comb += [
                    mem_write_port.addr  .eq(header_location),
                    mem_write_port.data  .eq(packet_size[7:16]),
                    mem_write_port.en    .eq(1)
                ]
                m.next = "EOP_2"


            with m.State("EOP_2"):

                # Add the second byte of our header.
                # Note that, if this is an adjacent read, we should have
                # just captured our packet header _during_ the stop turnaround.
                m.d.comb += [
                    mem_write_port.addr  .eq(header_location),
                    mem_write_port.data  .eq(packet_size[0:8]),
                    mem_write_port.en    .eq(self.umti.rx_valid)
                ]

                # Move to the next state, which will either be another capture,
                # or our idle state, depending on whether we have another rx.
                with m.If(self.umti.rx_active):
                    m.next = "CAPTURE"
                    m.d.sync += [
                        header_location  .eq(write_location),
                        write_location   .eq(write_location + self.HEADER_SIZE_BYTES),
                        packet_size      .eq(0),
                    ]
                with m.Else():
                    m.next = "IDLE"


            # BABBLE -- handles the case in which we've received a packet beyond
            # the allowable size in the USB spec
            with m.State("BABBLE"):

                # Trap here, for now.
                pass


            with m.State("OVERRUN"):
                pass



        return m

