;===== machine: P1S ========================
;===== date: 20240330 =====================
;===== author: mozhu ======================
; set temperature
{if old_filament_temp > 142 && next_extruder < 255}
M104 S[old_filament_temp]

; cut filament and move back to toilet
G1 X65 Y255 F21000
G1 X255
G1 Y0
G1 X30
G1 X0 F1000
G1 X64
G1 Y265 F21000

; return filament
G1 E5
G1 E-15

; call extern AMS to work
M73 P{101+next_extruder}
M73 P{101+next_extruder}
M73 P{101+next_extruder}

; wait for extern AMS work
M400 U1

; FLUSH_START
; always use highest temperature to flush
M400
{if filament_type[next_extruder] == "PETG"}
M109 S220
{else}
M109 S[nozzle_temperature_range_high]
{endif}
G1 E30

G1 X70 F9000
G1 X76 F15000
G1 X65 F15000
G1 X76 F15000
G1 X65 F15000; shake to put down garbage
G1 X80 F6000
G1 X95 F15000
G1 X80 F15000
G1 X165 F15000; 
G1 X80 F15000
G1 X165 F15000; 
G1 X80 F15000
G1 X165 F15000; wipe and shake
M400
M106 P1 S0