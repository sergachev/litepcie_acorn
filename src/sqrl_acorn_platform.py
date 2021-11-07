#
# This file is part of LiteX-Boards.
#
# Copyright (c) 2020 Florent Kermarrec <florent@enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

# The Acorn (CLE-101, CLE-215(+)) are cryptocurrency mining accelerator cards from SQRL that can be
# repurposed as generic FPGA PCIe development boards:
# - http://www.squirrelsresearch.com/acorn-cle-101
# - http://www.squirrelsresearch.com/acorn-cle-215-plus
# The 101 variant is eguivalent to the LiteFury and 215 variant equivalent to the NiteFury from
# RHSResearchLLC that are documented at: https://github.com/RHSResearchLLC/NiteFury-and-LiteFury.

from litex.build.generic_platform import *
from litex.build.xilinx import XilinxPlatform, VivadoProgrammer
from litex.build.openocd import OpenOCD

# IOs ----------------------------------------------------------------------------------------------

_io = [
    # Clk / Rst.
    ("clk200", 0,
        Subsignal("p", Pins("J19"), IOStandard("DIFF_SSTL15")),
        Subsignal("n", Pins("H19"), IOStandard("DIFF_SSTL15"))
    ),

    # Leds.
    ("user_led", 0, Pins("G3"), IOStandard("LVCMOS33")),
    ("user_led", 1, Pins("H3"), IOStandard("LVCMOS33")),
    ("user_led", 2, Pins("G4"), IOStandard("LVCMOS33")),
    ("user_led", 3, Pins("H4"), IOStandard("LVCMOS33")),

    # SPIFlash.
    ("flash_cs_n", 0, Pins("T19"), IOStandard("LVCMOS33")),
    ("flash", 0,
        Subsignal("mosi", Pins("P22")),
        Subsignal("miso", Pins("R22")),
        Subsignal("wp",   Pins("P21")),
        Subsignal("hold", Pins("R21")),
        IOStandard("LVCMOS33")
    ),

    # PCIe.
    ("pcie_clkreq_n", 0, Pins("G1"), IOStandard("LVCMOS33")),
    ("pcie_x1", 0,
        Subsignal("rst_n", Pins("J1"), IOStandard("LVCMOS33"), Misc("PULLUP=TRUE")),
        Subsignal("clk_p", Pins("F6")),
        Subsignal("clk_n", Pins("E6")),
        Subsignal("rx_p",  Pins("D11")),
        Subsignal("rx_n",  Pins("C11")),
        Subsignal("tx_p",  Pins("D5")),
        Subsignal("tx_n",  Pins("C5")),
    ),

    # DDR3 SDRAM.
    ("ddram", 0,
        Subsignal("a", Pins(
            "M15 L21 M16 L18 K21 M18 M21 N20",
            "M20 N19 J21 M22 K22 N18 N22 J22"),
            IOStandard("SSTL15")),
        Subsignal("ba",    Pins("L19 J20 L20"), IOStandard("SSTL15")),
        Subsignal("ras_n", Pins("H20"),  IOStandard("SSTL15")),
        Subsignal("cas_n", Pins("K18"),  IOStandard("SSTL15")),
        Subsignal("we_n",  Pins("L16"),  IOStandard("SSTL15")),
        Subsignal("dm", Pins("A19 G22"), IOStandard("SSTL15")),
        Subsignal("dq", Pins(
            "D19 B20 E19 A20 F19 C19 F20 C18",
            "E22 G21 D20 E21 C22 D21 B22 D22"),
            IOStandard("SSTL15"),
            Misc("IN_TERM=UNTUNED_SPLIT_50")),
        Subsignal("dqs_p", Pins("F18 B21"),  IOStandard("DIFF_SSTL15")),
        Subsignal("dqs_n", Pins("E18 A21"), IOStandard("DIFF_SSTL15")),
        Subsignal("clk_p", Pins("K17"), IOStandard("DIFF_SSTL15")),
        Subsignal("clk_n", Pins("J17"), IOStandard("DIFF_SSTL15")),
        Subsignal("cke",   Pins("H22"), IOStandard("SSTL15")),
        Subsignal("odt",   Pins("K19"), IOStandard("SSTL15")),
        Subsignal("reset_n", Pins("K16"), IOStandard("LVCMOS15")),
        #Subsignal("cs_n", Pins(""), IOStandard("SSTL15")),
        Misc("SLEW=FAST"),
    ),
]

_serial_io = [
    # Serial adapter on P2.
    ("serial", 0,
        Subsignal("tx", Pins("K2")),
        Subsignal("rx", Pins("J2")),
        Misc("SLEW=FAST"),
        IOStandard("LVCMOS33"),
    ),
]

_sdcard_io = [
    # SPI SDCard adapter on P2.
    # https://spoolqueue.com/new-design/fpga/migen/litex/2020/08/11/acorn-cle-215.html
    ("spisdcard", 0,
        Subsignal("clk",  Pins("J2")),
        Subsignal("mosi", Pins("J5"), Misc("PULLUP True")),
        Subsignal("cs_n", Pins("H5"), Misc("PULLUP True")),
        Subsignal("miso", Pins("K2"), Misc("PULLUP True")),
        Misc("SLEW=FAST"),
        IOStandard("LVCMOS33"),
    ),
]

# Platform -----------------------------------------------------------------------------------------

class Platform(XilinxPlatform):
    default_clk_name   = "clk200"
    default_clk_period = 1e9/200e6

    def __init__(self, variant="cle-215+"):
        device = {
            "cle-101":  "xc7a100t-fgg484-2",
            "cle-215":  "xc7a200t-fbg484-2",
            "cle-215+": "xc7a200t-fbg484-3"
        }[variant]

        XilinxPlatform.__init__(self, device, _io, toolchain="vivado")
        self.add_extension(_serial_io)
        self.add_extension(_sdcard_io)
        self.add_platform_command("set_property INTERNAL_VREF 0.750 [get_iobanks 34]")

        self.toolchain.bitstream_commands = [
            "set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]",
            "set_property BITSTREAM.CONFIG.CONFIGRATE 16 [current_design]",
            "set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]",
            "set_property CFGBVS VCCO [current_design]",
            "set_property CONFIG_VOLTAGE 3.3 [current_design]",
        ]
        self.toolchain.additional_commands = \
            ["write_cfgmem -force -format mcs -interface spix4 -size 16 "
             "-loadbit \"up 0x0 {build_name}.bit\" -file {build_name}.mcs"]

    def create_programmer(self):
        return VivadoProgrammer(flash_part='s25fl256sxxxxxx0-spi-x1_x2_x4')
        # return OpenOCD("openocd_xc7_ft232.cfg", "bscan_spi_xc7a200t.bit")

    def do_finalize(self, fragment):
        XilinxPlatform.do_finalize(self, fragment)
        self.add_period_constraint(self.lookup_request("clk200", loose=True), 1e9/200e6)
