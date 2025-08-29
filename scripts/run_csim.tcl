# --- Project Setup ---
open_project -reset "subiso_csim_proj"

# Add the kernel source files
add_files "./source/logger.cpp"
add_files "./source/cmdlineparser.cpp"
add_files "./source/subgraphIsomorphism.cpp" -cflags "-std=c++17"
add_files -tb "./source/tb_csim.cpp"
add_files -tb "./dataset_example"
add_files -tb "./scripts"
set_top subgraphIsomorphism

# --- Solution Setup ---
open_solution -reset "csim_solution" -flow_target vivado
list_part
set_part {xcu250-figd2104-2l-e}
create_clock -period 3.33ns -name default

# --- Run C Simulation ---
puts "--- Running C Simulation ---"
csim_design

exit