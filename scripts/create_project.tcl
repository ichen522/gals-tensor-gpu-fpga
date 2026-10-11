# create_project.tcl  (10_6) -- make a fresh Vivado project for this folder.
#
# Vivado Tcl Console (or "vivado -mode batch -source ..."):
#
#   source C:/Users/ian06/Desktop/590/10_6/scripts/create_project.tcl
#
# Creates 10_6/vivado/gals_10_6.xpr for the VC707 part, adds rtl/*.v
# (NOT sim/ -- those are simulation stand-ins for the Xilinx primitives),
# the pin / clock / Pblock constraints, the v7.2 implementation settings,
# and stops. Then press Generate Bitstream (or set RUN 1 below).
#
# No IP to generate: the SM MMCMs / PLLs are instantiated directly
# (rtl/sm_clk.v), and clk_mem is the board clock itself.

set RUN 0          ;# 1: also run synthesis + implementation + bitstream
set JOBS 8

set root [file normalize [file join [file dirname [info script]] ..]]
set proj [file join $root vivado]

create_project gals_10_6 $proj -part xc7vx485tffg1761-2 -force

# ---- sources ----
add_files -norecurse [glob [file join $root rtl *.v]]
set_property top vc707_top [current_fileset]
update_compile_order -fileset sources_1

# ---- constraints (order matters: the clock is created in the .xdc) ----
set c1 [file join $root constraints vc707_pins.xdc]
set c2 [file join $root constraints clocks.tcl]
set c3 [file join $root constraints pblocks.tcl]
add_files -fileset constrs_1 -norecurse [list $c1 $c2 $c3]
set_property USED_IN_SYNTHESIS false [get_files $c3]
set_property USED_IN_IMPLEMENTATION true [get_files $c3]

# ---- implementation settings carried over from v7.2 ----
if {[catch {set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]} err]} {
  catch {set_property strategy Performance_Explore [get_runs impl_1]}
}
set_property STEPS.OPT_DESIGN.TCL.PRE [file join $root scripts opt_pre.tcl] [get_runs impl_1]

puts "create_project: $proj/gals_10_6.xpr ready"
puts "  sources:     [llength [get_files -of_objects [get_filesets sources_1]]] files, top = vc707_top (NUM_SM = 24: 4 GPC x 3 TPC x 2 SM)"
puts "  constraints: vc707_pins.xdc, clocks.tcl, pblocks.tcl (implementation only)"
puts "  impl_1:      [get_property strategy [get_runs impl_1]], opt_design pre-hook opt_pre.tcl"

if {$RUN} {
  launch_runs impl_1 -to_step write_bitstream -jobs $JOBS
  wait_on_run impl_1
  puts "create_project: impl_1 [get_property STATUS [get_runs impl_1]]"
} else {
  puts "create_project: now press Generate Bitstream, then source scripts/reports.tcl"
}
