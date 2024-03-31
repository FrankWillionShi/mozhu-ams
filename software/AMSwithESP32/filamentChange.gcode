M620 S[next_extruder]A
M204 S9000

G2 Z{max_layer_z + 0.4} I0.86 J0.86 P1 F10000 ; spiral lift
G1 Z{max_layer_z + 3.0} F1200 ; vertical lift

; cut filament and move back to toilet
G1 X65 Y250 F21000
G1 X250
G1 Y0
G1 X15
G1 Y-5 F1000
G1 X5
G1 X15
G1 X64 F21000
G1 Y265
M400

; call extern AMS to work
M73 P{101+next_extruder}
M73 P{101+next_extruder}
M73 P{101+next_extruder}
M400 S1
M73 P{101+next_extruder}
M73 P{101+next_extruder}
M73 P{101+next_extruder}
M400 S1
M73 P{101+next_extruder}
M73 P{101+next_extruder}
M73 P{101+next_extruder}
M400 S1


; return filament
G1 E10
G1 E-10 F100
G1 E-60 F100
M400

;;

; FLUSH_START
; always use highest temperature to flush
M400
{if filament_type[next_extruder] == "PETG"}
M109 S250
{else}
M109 S[nozzle_temperature_range_high]
{endif}

G92 E0
G1 E10 F200
M400
G1 E50 F200
M400

;turn on fans
M106 P1 S255
M400 S3

G1 X70 F9000
G1 X76 F15000
G1 X65 F15000
G1 X76 F15000
G1 X65 F15000; shake to put down garbage
G1 X70 F6000
G1 X95 F15000
G1 X70 F15000
G1 X100 F15000
G1 X70 F15000
G1 X100 F15000
G1 X70 F15000
G1 X100 F15000
G1 X70 F15000
G1 X100 F15000
G1 X128 F15000; wipe and shake

;turn off fans
M106 P1 S0

;go to a safe place
G1 Y250
G1 X5

;ams end

{if layer_z <= (initial_layer_print_height + 0.001)}
M204 S[initial_layer_acceleration]
{else}
M204 S[default_acceleration]
{endif}

M621 S[next_extruder]A