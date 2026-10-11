# temp_log.tcl  (10_6) -- log the FPGA's on-chip temperature and supply
# voltages (XADC / System Monitor, read over JTAG) to a CSV file.
#
# Hardware Manager open, device programmed, then in the Tcl Console:
#
#   source C:/Users/ian06/Desktop/590/10_6/scripts/temp_log.tcl
#
# Change SECONDS / PERIOD_MS / OUT below first if you like. Vivado is busy
# (GUI frozen) while it logs; the console prints a line every 30 s.
#
# Columns: epoch_ms (PC clock, to line up with other logs), t_s (seconds
# since start), temp_C, vccint_V, vccaux_V, vccbram_V.
# The XADC's temperature step is about 0.12 C, so single samples jitter by
# a few tenths of a degree; more samples = smoother average.

set SECONDS   660          ;# 11 min: 1 min idle + 5 min load + 5 min cooling
set PERIOD_MS 100          ;# one sample every 100 ms (10 per second) if JTAG keeps up
set OUT       C:/Users/ian06/Desktop/590/10_6/temp_log_fast.csv

set sm [lindex [get_hw_sysmons] 0]
if {$sm eq ""} { error "temp_log: no System Monitor found -- open the hardware target and program the device first" }

proc tl_val {sm prop} {
  if {[catch {set v [get_property $prop $sm]}]} { return "" }
  if {[regexp {[-0-9.]+} $v num]} { return $num }
  return ""
}

set f  [open $OUT w]
puts $f "epoch_ms,t_s,temp_C,vccint_V,vccaux_V,vccbram_V"
set t0 [clock milliseconds]
set n  0
set next_report 30
puts "temp_log: logging $SECONDS s every $PERIOD_MS ms -> $OUT"
while {1} {
  set now [clock milliseconds]
  set el  [expr {($now - $t0) / 1000.0}]
  if {$el >= $SECONDS} break
  refresh_hw_sysmon $sm
  set tc [tl_val $sm TEMPERATURE]
  puts $f [format "%s,%.3f,%s,%s,%s,%s" $now $el $tc [tl_val $sm VCCINT] [tl_val $sm VCCAUX] [tl_val $sm VCCBRAM]]
  incr n
  if {$el >= $next_report} {
    flush $f
    puts [format "temp_log: %4.0f s, %d samples, %s C" $el $n $tc]
    incr next_report 30
  }
  # wait for the next slot (no wait if the JTAG read itself took longer)
  set wait [expr {$t0 + $n * $PERIOD_MS - [clock milliseconds]}]
  if {$wait > 0} { after $wait }
}
close $f
set dt [expr {([clock milliseconds] - $t0) / 1000.0}]
puts [format "temp_log: done, %d samples in %.0f s (%.1f per second) -> %s" $n $dt [expr {$n / $dt}] $OUT]
