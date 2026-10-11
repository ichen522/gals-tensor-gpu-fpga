# opt_pre.tcl -- runs right before opt_design in impl_1
# (STEPS.OPT_DESIGN.TCL.PRE, set by scripts/create_project.tcl). Unchanged from v7.2.
#
# Keeps Vivado's BRAM power optimization away from every BRAM. In the
# first 400 MHz run it added "cooolgate" LUTs that drive each RAMB36's
# REGCE from the reset / refill nets on the far side of the tile -- the
# 55 worst paths of the design (-0.71 ns). The design already gates BRAM
# reads with its own enables, so this optimization saves almost nothing.
set brams [get_cells -quiet -hierarchical -filter {PRIMITIVE_TYPE =~ BMEM.*.*}]
puts "opt_pre: excluding [llength $brams] BRAM cells from BRAM power optimization"
if {[llength $brams] > 0} {
  set_power_opt -exclude_cells $brams
}
