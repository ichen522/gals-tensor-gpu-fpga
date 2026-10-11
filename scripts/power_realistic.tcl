# power_realistic.tcl  (10_6) -- VCCINT current with realistic switching.
#
# With the IMPLEMENTED design open:
#
#   source C:/Users/ian06/Desktop/590/10_6/scripts/power_realistic.tcl
#
# Sets every signal net inside the selected SMs to a 50% toggle rate
# (random 16-bit data: about half the bits flip every k-step; this is the
# worst case "SM at 100% duty") and reports power. The design is not
# changed; the activity is reset afterwards.
#
# IMPORTANT: Vivado times the SMs at the COMPILED clock (DIV_MIN = 3,
# 400 MHz), but they boot at 200 MHz. Dynamic power scales with f, so at
# 200 MHz:  I(200) ~ I_static + I_dyn / 2.
#
# SMS: how many SMs to set active, from SM 0 up (the rest stay vectorless
# = idle). 6 = one GPC, 24 = the whole GPU.
# VC707 limit: VCCINT 20 A (UCD9248 cuts the rail above that).
#
# !! When you close the implemented design afterwards and Vivado asks to
# !! save the constraints, answer DON'T SAVE: otherwise it writes the
# !! set_switching_activity of every SM net into vc707_pins.xdc (38 MB).

set SMS 24

set root [file normalize [file join [file dirname [info script]] ..]]
set out  [file join $root reports]
file mkdir $out

set nets {}
for {set i 0} {$i < $SMS} {incr i} {
  # '?' stands in for the brackets: SM?1?. does not match SM[10].
  set n [get_nets -quiet -hierarchical -filter "TYPE == SIGNAL && NAME =~ \"SM?${i}?.u_sm/*\""]
  set nets [concat $nets $n]
}
puts "power_realistic: 50% toggle on [llength $nets] nets of $SMS SM(s) (takes a few minutes) ..."
set_switching_activity -toggle_rate 50 -static_probability 0.5 $nets
set f [file join $out power_toggle50_${SMS}sms.txt]
report_power -hierarchical_depth 2 -file $f
catch {reset_switching_activity -all}
puts "power_realistic: report in $f"
puts "  read the Vccint row of '1.2 Power Supply Summary' (Total / Dynamic / Static A)."
puts "  that is at 400 MHz; at 200 MHz: about Static + Dynamic / 2.  VC707 limit: 20 A."
puts "power_realistic: when you close this design, do NOT save the constraints (Don't Save)."
