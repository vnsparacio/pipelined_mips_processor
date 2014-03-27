
# PlanAhead Launch Script for Post PAR Floorplanning, created by Project Navigator

create_project -name mips -dir "/afs/ir.stanford.edu/users/s/p/sparacio/Desktop/lab3/synth/planAhead_run_1" -part xc5vlx110tff1136-1
set srcset [get_property srcset [current_run -impl]]
set_property design_mode GateLvl $srcset
set_property edif_top_file "/afs/ir.stanford.edu/users/s/p/sparacio/Desktop/lab3/synth/mips_top.ngc" [ get_property srcset [ current_run ] ]
add_files -norecurse { {/afs/ir.stanford.edu/users/s/p/sparacio/Desktop/lab3/synth} }
set_param project.paUcfFile  "mips_top.ucf"
add_files "mips_top.ucf" -fileset [get_property constrset [current_run]]
open_netlist_design
read_xdl -file "/afs/ir.stanford.edu/users/s/p/sparacio/Desktop/lab3/synth/mips_top.xdl"
if {[catch {read_twx -name results_1 -file "/afs/ir.stanford.edu/users/s/p/sparacio/Desktop/lab3/synth/mips_top.twx"} eInfo]} {
   puts "WARNING: there was a problem importing \"/afs/ir.stanford.edu/users/s/p/sparacio/Desktop/lab3/synth/mips_top.twx\": $eInfo"
}
