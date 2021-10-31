import argparse
from litex.soc.interconnect.csr import *
from litex.soc.integration.soc_core import *
from litex.soc.integration.builder import *
from litex.soc.cores.clock import *
from litex.soc.cores.led import LedChaser
from litex.soc.cores.icap import ICAP
from litepcie.phy.s7pciephy import S7PCIEPHY
from litepcie.software import generate_litepcie_software
import sqrl_acorn_platform as acorn


class CRG(Module):
    def __init__(self, platform, sys_clk_freq):
        self.rst = Signal()
        self.clock_domains.cd_sys = ClockDomain()
        self.clock_domains.cd_sys4x = ClockDomain(reset_less=True)
        self.clock_domains.cd_sys4x_dqs = ClockDomain(reset_less=True)
        self.clock_domains.cd_idelay = ClockDomain()

        clk200 = platform.request("clk200")
        self.submodules.pll = pll = S7PLL()
        self.comb += pll.reset.eq(self.rst)
        pll.register_clkin(clk200, 200e6)
        pll.create_clkout(self.cd_sys, sys_clk_freq)
        pll.create_clkout(self.cd_sys4x, 4 * sys_clk_freq)
        pll.create_clkout(self.cd_sys4x_dqs, 4 * sys_clk_freq, phase=90)
        pll.create_clkout(self.cd_idelay, 200e6)
        # Ignore sys_clk to pll.clkin path created by SoC's rst.
        platform.add_false_path_constraints(self.cd_sys.clk, pll.clkin)
        self.submodules.idelayctrl = S7IDELAYCTRL(self.cd_idelay)


class BaseSoC(SoCCore):
    def __init__(self, variant="cle-215+", sys_clk_freq=int(100e6), with_led_chaser=True, **kwargs):
        platform = acorn.Platform(variant=variant)

        SoCCore.__init__(self, platform, sys_clk_freq,
                         ident="LiteX SoC on Acorn CLE-101/215(+)",
                         ident_version=True,
                         **kwargs)

        self.submodules.crg = CRG(platform, sys_clk_freq)

        self.submodules.pcie_phy = S7PCIEPHY(platform, platform.request("pcie_x1"),
                                             data_width=64,
                                             bar0_size=0x20000)
        self.add_pcie(phy=self.pcie_phy, ndmas=1)
        # FIXME: Apply it to all targets (integrate it in LitePCIe?).
        platform.add_period_constraint(self.crg.cd_sys.clk, 1e9 / sys_clk_freq)
        platform.toolchain.pre_placement_commands.append(
            "reset_property LOC [get_cells -hierarchical -filter "
            "{{NAME=~pcie_support/*gtp_channel.gtpe2_channel_i}}]")
        platform.toolchain.pre_placement_commands.append(
            "set_property LOC GTPE2_CHANNEL_X0Y5 [get_cells -hierarchical -filter "
            "{{NAME=~pcie_support/*gtp_channel.gtpe2_channel_i}}]")

        platform.toolchain.pre_placement_commands.add(
            "set_clock_groups -group [get_clocks {sys_clk}] -group [get_clocks userclk1] -asynchronous",
            sys_clk=self.crg.cd_sys.clk)
        platform.toolchain.pre_placement_commands.add(
            "set_clock_groups -group [get_clocks {sys_clk}] -group [get_clocks clk_125mhz] -asynchronous",
            sys_clk=self.crg.cd_sys.clk)
        platform.toolchain.pre_placement_commands.add(
            "set_clock_groups -group [get_clocks {sys_clk}] -group [get_clocks clk_250mhz] -asynchronous",
            sys_clk=self.crg.cd_sys.clk)
        platform.toolchain.pre_placement_commands.add(
            "set_clock_groups -group [get_clocks clk_125mhz] -group [get_clocks clk_250mhz] -asynchronous")

        self.submodules.icap = ICAP()
        self.icap.add_reload()
        self.icap.add_timing_constraints(platform, sys_clk_freq, self.crg.cd_sys.clk)

        if with_led_chaser:
            self.submodules.leds = LedChaser(
                pads=platform.request("user_led", 0),
                sys_clk_freq=sys_clk_freq)
        self.comb += platform.request("user_led", 1).eq(~self.pcie_phy._link_status.fields.status)


def main():
    parser = argparse.ArgumentParser(description="LiteX SoC on Acorn CLE-101/215(+)")
    parser.add_argument("--build", action="store_true", help="Build bitstream")
    parser.add_argument("--load", action="store_true", help="Load bitstream")
    parser.add_argument("--flash", action="store_true", help="Flash bitstream")
    parser.add_argument("--variant", default="cle-215+", help="Board variant: cle-215+ (default), cle-215 or cle-101")
    parser.add_argument("--sys-clk-freq", default=100e6, help="System clock frequency (default: 100MHz)")
    parser.add_argument("--driver", action="store_true", help="Generate PCIe driver")
    builder_args(parser)
    soc_core_args(parser)
    args = parser.parse_args()

    soc_argdict = soc_core_argdict(args)
    soc_argdict['cpu_type'] = None
    soc = BaseSoC(
        variant=args.variant,
        sys_clk_freq=int(float(args.sys_clk_freq)),
        **soc_argdict
    )

    builder = Builder(soc, **builder_argdict(args))
    builder.build(run=args.build)

    if args.driver:
        generate_litepcie_software(soc, os.path.join(builder.output_dir, "driver"))

    if args.load:
        prog = soc.platform.create_programmer()
        prog.load_bitstream(os.path.join(builder.gateware_dir, soc.build_name + ".bit"))

    if args.flash:
        prog = soc.platform.create_programmer()
        prog.flash(0, os.path.join(builder.gateware_dir, soc.build_name + ".bin"))


if __name__ == "__main__":
    main()
