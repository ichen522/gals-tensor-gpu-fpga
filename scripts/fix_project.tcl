# fix_project.tcl  (10_3 / 10_6) -- repair a project made with the New Project wizard
# (for example C:/Users/ian06/project_3) so it builds this folder the same way
# scripts/create_project.tcl would.
#
# With THAT project open in Vivado, in the Tcl Console:
#
#   source C:/Users/ian06/Desktop/590/10_6/scripts/fix_project.tcl
#
# then press Generate Bitstream. Safe to run more than once.
#
# What it fixes:
#   1. part -> xc7vx485tffg1761-2 (VC707). With any other package the pins in
#      vc707_pins.xdc do not exist ("'AV40' is not a valid site") and sysclk
#      lands on a non-clock pin ("Place 30-574", "IO Clock Placer failed").
#   2. sources = every rtl/*.v of this folder (and never sim/sim_prims.v), top = vc707_top.
#   3. constraints = vc707_pins.xdc, clocks.tcl, pblocks.tcl (implementation only),
#      in that order.
#   4. no incremental synthesis from the old (wrong-part) checkpoint.
#   5. impl strategy + opt_design pre-hook, as in create_project.tcl.
#   6. resets synth_1 / impl_1 so nothing stale is reused.

set PART xc7vx485tffg1761-2
set root [file normalize [file join [file dirname [info script]] ..]]

proc fp_norm {p} { return [string tolower [file normalize $p]] }

# the file object(s) in a fileset whose path is $path (case-insensitive: Windows)
proc fp_get {fileset path} {
  set want [fp_norm $path]
  set out {}
  foreach f [get_files -quiet -of_objects [get_filesets $fileset]] {
    if {[fp_norm $f] eq $want} { lappend out $f }
  }
  return $out
}
proc fp_has {fileset path} { return [expr {[llength [fp_get $fileset $path]] > 0}] }

puts "fix_project: project [get_property NAME [current_project]] ([get_property DIRECTORY [current_project]])"

# ---- 1. part ----
set old [get_property PART [current_project]]
set_property part $PART [current_project]
foreach r [get_runs -quiet {synth_1 impl_1}] {
  catch {set_property PART $PART $r}
}
puts "fix_project: part $old -> [get_property PART [current_project]]"

# ---- 2. sources ----
foreach f [lsort [glob [file join $root rtl *.v]]] {
  if {![fp_has sources_1 $f]} {
    add_files -norecurse -fileset sources_1 $f
    puts "fix_project:   + [file tail $f]"
  }
}
foreach f [get_files -quiet -of_objects [get_filesets sources_1]] {
  set t [file tail $f]
  if {$t eq "sim_prims.v" || [string match "tb_*" $t]} {
    remove_files -fileset sources_1 $f
    puts "fix_project:   - $t (simulation only)"
  }
}
set_property top vc707_top [get_filesets sources_1]
update_compile_order -fileset sources_1

# ---- 3. constraints ----
set c1 [file join $root constraints vc707_pins.xdc]
set c2 [file join $root constraints clocks.tcl]
set c3 [file join $root constraints pblocks.tcl]
foreach c [list $c1 $c2 $c3] {
  if {![fp_has constrs_1 $c]} {
    add_files -norecurse -fileset constrs_1 $c
    puts "fix_project:   + [file tail $c]"
  }
}
# order: the clock is created in the .xdc, clocks.tcl uses it, pblocks.tcl last
set f1 [fp_get constrs_1 $c1]
set f2 [fp_get constrs_1 $c2]
set f3 [fp_get constrs_1 $c3]
catch {reorder_files -fileset constrs_1 -front $f1}
catch {reorder_files -fileset constrs_1 -after  $f1 $f2}
catch {reorder_files -fileset constrs_1 -back  $f3}
set_property USED_IN_SYNTHESIS      true  $f1
set_property USED_IN_IMPLEMENTATION true  $f1
set_property USED_IN_SYNTHESIS      true  $f2
set_property USED_IN_IMPLEMENTATION true  $f2
set_property USED_IN_SYNTHESIS      false $f3
set_property USED_IN_IMPLEMENTATION true  $f3
catch {set_property target_constrs_file $c1 [get_filesets constrs_1]}

# ---- 4. no incremental synthesis from the old checkpoint ----
catch {set_property AUTO_INCREMENTAL_CHECKPOINT 0 [get_runs synth_1]}
catch {set_property INCREMENTAL_CHECKPOINT {} [get_runs synth_1]}
catch {
  foreach f [get_files -quiet -of_objects [get_filesets -quiet utils_1] *.dcp] {
    catch {remove_files -fileset utils_1 $f}
  }
}

# ---- 5. implementation settings (same as create_project.tcl) ----
if {[catch {set_property strategy Performance_ExplorePostRoutePhysOpt [get_runs impl_1]} err]} {
  catch {set_property strategy Performance_Explore [get_runs impl_1]}
}
set_property STEPS.OPT_DESIGN.TCL.PRE [file join $root scripts opt_pre.tcl] [get_runs impl_1]

# ---- 6. start clean ----
catch {reset_run impl_1}
catch {reset_run synth_1}

puts "fix_project: done"
puts "  part:        [get_property PART [current_project]]"
puts "  sources:     [llength [get_files -quiet -of_objects [get_filesets sources_1]]] files, top = [get_property TOP [get_filesets sources_1]]"
set cl {}
foreach f [get_files -of_objects [get_filesets constrs_1]] { lappend cl [file tail $f] }
puts "  constraints: $cl"
puts "  impl_1:      [get_property strategy [get_runs impl_1]]"
puts "fix_project: now press Generate Bitstream, then source scripts/reports.tcl"
