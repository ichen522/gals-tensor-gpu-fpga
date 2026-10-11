# pblocks.tcl  (10_6) -- Tcl constraint file, IMPLEMENTATION ONLY
# (scripts/create_project.tcl sets USED_IN_SYNTHESIS false).
#
# The GPU floorplan. The XC7VX485T has 14 clock regions (2 columns x 7
# rows), each with one CMT = one MMCM + one PLL:
#
#        X0     X1
#   Y6  GPC2   GPC3     TPC 0 of each GPC = its corner region,
#   Y5  GPC2   GPC3     TPC 1 next to it, TPC 2 next to that
#   Y4  GPC2   GPC3
#   Y3  front end (host_if, win_ctrl, clk_ctrl, spine registers)
#   Y2  GPC0   GPC1
#   Y1  GPC0   GPC1
#   Y0  GPC0   GPC1
#
#   TPC = one clock region, SM = half of it:
#     SM 6g+2t   (slot 0): lower half, clocked by the region's MMCM
#     SM 6g+2t+1 (slot 1): upper half, clocked by the region's PLL
#
# Each SM Pblock = the SLICE / DSP48 / RAMB18 / RAMB36 ranges of its half
# region (read from the device, so no hand-typed coordinates) + its MMCM /
# PLL site (LOCed). SM[i].u_sm (sm_top: BRAMs, 64 DSPs, readout,
# checksum, CDC) and SM[i].u_clk (sm_clk: MMCM / PLL + clock-enable logic)
# are the members. EXCLUDE_PLACEMENT keeps everything else out.
#
# The SM's regional clock buffers (BUFH clk_free, BUFHCE SM clock) are NOT
# members: no BUFHCE site is in these ranges, and the clock placer puts
# them in the only region they can drive -- the one with their MMCM / PLL
# and all their loads. The HROW (where the BUFHs sit) runs through the
# middle of the region, i.e. exactly between the two SM halves, so each
# SM's clock-enable path is short.
#
# Per half region: X0 column 90 DSP / 35 RAMB36 / ~2,700 slices,
# X1 column 110 DSP / 40 RAMB36; one SM needs 64 DSP, 4 RAMB18 + 2 RAMB36,
# ~3.5k LUT, ~5k FF. Check after placement: scripts/reports.tcl
# (util_pblocks.txt).

set TPC_REGIONS {X0Y0 X0Y1 X0Y2  X1Y0 X1Y1 X1Y2  X0Y6 X0Y5 X0Y4  X1Y6 X1Y5 X1Y4}
set EXCLUDE true

proc pb_cell {name} {
  set c [get_cells -quiet $name]
  if {[llength $c] == 0} {
    set c [get_cells -quiet -hierarchical -filter "NAME == \"$name\""]
  }
  return $c
}

# sites of one type prefix (SLICE, DSP48, RAMB18, RAMB36) in a clock region
proc pb_region_sites {region prefix} {
  set s {}
  catch {set s [get_sites -quiet -of_objects [get_clock_regions $region] -filter "NAME =~ ${prefix}_X*"]}
  if {[llength $s] == 0} {
    set s [get_sites -quiet -filter "CLOCK_REGION == $region && NAME =~ ${prefix}_X*"]
  }
  return $s
}

# site ranges of the lower (half 0) or upper (half 1) half of a region
proc pb_half_ranges {region half} {
  set out {}
  foreach pre {SLICE DSP48 RAMB18 RAMB36} {
    set xs {}
    set ys {}
    foreach st [pb_region_sites $region $pre] {
      if {[regexp "^${pre}_X(\\d+)Y(\\d+)\$" $st -> x y]} { lappend xs $x; lappend ys $y }
    }
    if {[llength $ys] == 0} {
      puts "pblocks.tcl: WARNING no $pre sites found in $region"
      continue
    }
    set xs [lsort -integer -unique $xs]
    set ys [lsort -integer -unique $ys]
    set h  [expr {[llength $ys] / 2}]
    if {$half == 0} {
      set y0 [lindex $ys 0];  set y1 [lindex $ys [expr {$h - 1}]]
    } else {
      set y0 [lindex $ys $h]; set y1 [lindex $ys end]
    }
    lappend out "${pre}_X[lindex $xs 0]Y${y0}:${pre}_X[lindex $xs end]Y${y1}"
  }
  return $out
}

set n 0
for {set i 0} {$i < 2 * [llength $TPC_REGIONS]} {incr i} {
  set sm [pb_cell "SM\[$i\].u_sm"]
  if {[llength $sm] == 0} { break }
  set ck   [pb_cell "SM\[$i\].u_clk"]
  set gpc  [expr {$i / 6}]
  set tpc  [expr {($i % 6) / 2}]
  set slot [expr {$i % 2}]
  set r    [lindex $TPC_REGIONS [expr {3 * $gpc + $tpc}]]
  regexp {X(\d+)Y(\d+)} $r -> cx cy

  set pb [get_pblocks -quiet pb_sm$i]
  if {[llength $pb] == 0} { set pb [create_pblock pb_sm$i] }
  set rngs [pb_half_ranges $r $slot]
  foreach rng $rngs { resize_pblock $pb -add $rng }

  # the SM's clock generator: the region's MMCM (slot 0) or PLL (slot 1)
  set kind [expr {$slot == 0 ? "MMCME2_ADV" : "PLLE2_ADV"}]
  set site [get_sites -quiet "${kind}_X${cx}Y${cy}"]
  set prim {}
  foreach c [get_cells -quiet -hierarchical -filter "REF_NAME == $kind"] {
    if {[string first "SM\[$i\].u_clk/" $c] == 0} { lappend prim $c }
  }
  if {[llength $site] > 0 && [llength $prim] == 1} {
    resize_pblock $pb -add "${kind}_X${cx}Y${cy}:${kind}_X${cx}Y${cy}"
    set_property LOC $site $prim
  } else {
    puts "pblocks.tcl: WARNING SM $i: site ${kind}_X${cx}Y${cy} ([llength $site]) / cell ([llength $prim]) not found, left to the placer"
  }

  add_cells_to_pblock $pb $sm
  if {[llength $ck] > 0} { add_cells_to_pblock $pb $ck }
  set_property EXCLUDE_PLACEMENT $EXCLUDE $pb
  puts [format "pblocks.tcl: SM %2d = GPC %d TPC %d slot %d -> %s %s half, %s  (%s)" \
          $i $gpc $tpc $slot $r [expr {$slot ? "upper" : "lower"}] $kind [join $rngs { }]]
  incr n
}
puts "pblocks.tcl: $n SM Pblocks"
