# reports.tcl  (10_6) -- write the reports Claude needs into 10_6/reports/.
#
# With the IMPLEMENTED design open (Open Implemented Design):
#
#   source C:/Users/ian06/Desktop/590/10_6/scripts/reports.tcl
#
# A few minutes; only writes files.

set root [file normalize [file join [file dirname [info script]] ..]]
set out  [file join $root reports]
file mkdir $out
puts "reports: writing to $out ..."

proc try_rpt {label body} {
  puts "  $label"
  if {[catch {uplevel 1 $body} err]} { puts "    (skipped: $err)" }
}

try_rpt "timing summary" {
  report_timing_summary -max_paths 5 -file $out/timing_summary.txt
}
try_rpt "worst 300 paths (one line each)" {
  report_timing -max_paths 300 -nworst 1 -unique_pins -sort_by slack \
                -path_type summary -file $out/worst_paths_summary.txt
}
try_rpt "path characteristics" {
  report_design_analysis -timing -setup -max_paths 200 -file $out/design_analysis_setup.txt
}
try_rpt "worst paths per SM clock" {
  foreach m [get_cells -quiet -hierarchical -filter {REF_NAME == MMCME2_ADV || REF_NAME == PLLE2_ADV}] {
    if {![regexp {SM\[(\d+)\]} $m -> i]} { continue }
    set c [get_clocks -quiet -of_objects [get_pins $m/CLKOUT1]]
    if {[llength $c]} {
      report_timing -from $c -to $c -max_paths 10 -nworst 1 -unique_pins -sort_by slack \
                    -input_pins -file $out/paths_sm$i.txt
    }
  }
}
try_rpt "clk_mem paths" {
  report_timing -from [get_clocks sysclk] -to [get_clocks sysclk] -max_paths 20 -nworst 1 \
                -unique_pins -sort_by slack -input_pins -file $out/paths_clk_mem.txt
}
try_rpt "high-fanout nets" {
  report_high_fanout_nets -timing -load_types -max_nets 40 -file $out/high_fanout.txt
}
try_rpt "utilization (hierarchical)" {
  report_utilization -hierarchical -hierarchical_depth 3 -file $out/util_hier.txt
}
try_rpt "utilization per Pblock" {
  # one Pblock per call (passing all 12 at once was skipped on 2025.2)
  set fh [open $out/util_pblocks.txt w]
  foreach pb [lsort -dictionary [get_pblocks -quiet pb_sm*]] {
    puts $fh "################ $pb ([get_property GRID_RANGES $pb])"
    if {[catch {puts $fh [report_utilization -pblocks $pb -return_string]} err]} {
      puts $fh "(failed: $err)"
    }
  }
  close $fh
}
try_rpt "clock utilization (which clock in which region)" {
  report_clock_utilization -file $out/clock_utilization.txt
}
try_rpt "power (vectorless)" {
  report_power -hierarchical_depth 2 -file $out/power.txt
}
puts "reports: done -- tell Claude the reports are in $out"
