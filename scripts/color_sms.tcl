# color_sms.tcl  (10_6) -- paint the GPU hierarchy in the Device view.
#
# Open Implemented Design, open the Device window, then in the Tcl Console:
#
#   source C:/Users/ian06/Desktop/590/10_6/scripts/color_sms.tcl
#
# One color family per GPC (GPC 0 red, 1 green, 2 blue, 3 yellow/orange);
# inside a GPC the 3 TPCs get dark / middle / light shades and the two SMs
# of a TPC two neighbouring shades. So you see 4 corners (GPCs), each made
# of 3 stacked clock regions (TPCs), each split in two halves (SMs).
# Front end (UART / host_if, win_ctrl, clk_ctrl) = gray, spine registers =
# white. The Console prints the legend.  Undo:  unhighlight_objects
#
# Takes about a minute (it walks every cell once per SM).

# rgb of SM 6*g + 2*t + s = PALETTE[g][t][s]
set PALETTE {
  { {{200   0   0} {255  80  80}}  {{150   0  60} {235  70 140}}  {{255 130  90} {255 190 170}} }
  { {{  0 140   0} { 90 220  90}}  {{  0 110  80} { 60 200 150}}  {{150 220  50} {205 255 150}} }
  { {{  0  60 220} { 90 150 255}}  {{ 70   0 200} {160 120 255}}  {{  0 170 230} {160 225 255}} }
  { {{220 130   0} {255 200  60}}  {{190  90   0} {255 160  90}}  {{220 220   0} {255 255 150}} }
}
set GPC_NAME {red green blue yellow}

# leaf cells whose full name matches one of the glob patterns
# ('?' stands in for '[' and ']' so SM?1?. never matches SM[10].)
proc cs_leaf {patterns} {
  set f {}
  foreach p $patterns { lappend f "NAME =~ \"$p\"" }
  return [get_cells -quiet -hierarchical -filter "IS_PRIMITIVE && ([join $f { || }])"]
}

catch {unhighlight_objects}

set n 0
puts "color_sms:  SM  GPC TPC slot  clock  color             cells  Pblock"
for {set i 0} {$i < 24} {incr i} {
  set cells [cs_leaf [list "SM?${i}?.u_sm/*" "SM?${i}?.u_clk/*" "SM?${i}?.u_buf*"]]
  if {[llength $cells] == 0} { break }
  set g [expr {$i / 6}]; set t [expr {($i % 6) / 2}]; set s [expr {$i % 2}]
  set rgb [lindex $PALETTE $g $t $s]
  highlight_objects -rgb $rgb $cells
  set rng "-"
  set pb [get_pblocks -quiet pb_sm$i]
  if {[llength $pb] > 0} { catch {set rng [lindex [get_property GRID_RANGES $pb] 0]} }
  puts [format "color_sms: %3d   %d   %d   %d    %-5s  %-6s %-11s %6d  %s" \
          $i $g $t $s [expr {$s ? "PLL" : "MMCM"}] [lindex $GPC_NAME $g] "{$rgb}" [llength $cells] $rng]
  incr n
}

set fe [cs_leaf {"u_host_if/*" "u_win_ctrl/*" "u_clk_ctrl/*" "u_reset*"}]
if {[llength $fe] > 0} {
  highlight_objects -rgb {150 150 150} $fe
  puts [format "color_sms:  front end (host_if, win_ctrl, clk_ctrl)  gray   %6d" [llength $fe]]
}
set sp [cs_leaf {"SM?*?.sp_*"}]
if {[llength $sp] > 0} {
  highlight_objects -rgb {255 255 255} $sp
  puts [format "color_sms:  spine registers                         white  %6d" [llength $sp]]
}
puts "color_sms: $n SMs colored. Undo: unhighlight_objects"
