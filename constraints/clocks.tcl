# clocks.tcl  (10_6) -- Tcl constraint file (synthesis + implementation).
#
# Every SM clock comes from its own MMCM or PLL, fed by clk_mem (= sysclk),
# so Vivado would treat them all as RELATED clocks and try to time every
# crossing between them. The crossings are all handled by synchronizers /
# handshakes in the RTL (tog_cdc, stat_cdc, en_gen, sm_clk, the dual-clock
# BRAMs), so: sysclk and each SM clock = its own asynchronous group -- the
# GALS structure. A Tcl file (not .xdc) because the number of SMs depends
# on the build: it simply finds every MMCME2_ADV / PLLE2_ADV and takes the
# clock on CLKOUT1.

# The pins in vc707_pins.xdc only exist on the VC707's package.
set part [get_property PART [current_design]]
if {![string match -nocase "xc7vx485tffg1761*" $part]} {
  common::send_msg_id {10_6-1} {CRITICAL WARNING} "this project's part is $part, the VC707 is xc7vx485tffg1761-2: pins like AV40 will not exist. Fix: set_property part xc7vx485tffg1761-2 \[current_project\] (or build with scripts/create_project.tcl)"
}

set grp [list -group [get_clocks sysclk]]
set n 0
foreach m [lsort -dictionary [get_cells -quiet -hierarchical -filter {REF_NAME == MMCME2_ADV || REF_NAME == PLLE2_ADV}]] {
  set c [get_clocks -quiet -of_objects [get_pins -quiet $m/CLKOUT1]]
  if {[llength $c] > 0} {
    lappend grp -group $c
    incr n
  }
}
set_clock_groups -name async_sms -asynchronous {*}$grp
puts "clocks.tcl: sysclk + $n SM clocks set asynchronous to each other"
