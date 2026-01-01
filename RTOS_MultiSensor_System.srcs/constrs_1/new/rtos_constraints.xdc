# Timing Constraints for RTOS Multi-Sensor Control System
# This file defines the timing requirements for the design

# Define primary clock - 100 MHz (10ns period)
create_clock -period 10.000 -name sys_clk -waveform {0.000 5.000} [get_ports clk]

# Set input delay constraints for sensor data
# Assuming sensors provide data 2ns after clock edge
set_input_delay -clock [get_clocks sys_clk] -min 1.000 [get_ports {temp_sensor_data[*]}]
set_input_delay -clock [get_clocks sys_clk] -max 3.000 [get_ports {temp_sensor_data[*]}]

set_input_delay -clock [get_clocks sys_clk] -min 1.000 [get_ports {pressure_sensor_data[*]}]
set_input_delay -clock [get_clocks sys_clk] -max 3.000 [get_ports {pressure_sensor_data[*]}]

set_input_delay -clock [get_clocks sys_clk] -min 1.000 [get_ports {proximity_sensor_data[*]}]
set_input_delay -clock [get_clocks sys_clk] -max 3.000 [get_ports {proximity_sensor_data[*]}]

set_input_delay -clock [get_clocks sys_clk] -min 1.000 [get_ports sensor_valid]
set_input_delay -clock [get_clocks sys_clk] -max 3.000 [get_ports sensor_valid]

# Set output delay constraints for control signals
# Assuming external devices need data 2ns before next clock edge
set_output_delay -clock [get_clocks sys_clk] -min 1.000 [get_ports cooling_fan_ctrl]
set_output_delay -clock [get_clocks sys_clk] -max 7.000 [get_ports cooling_fan_ctrl]

set_output_delay -clock [get_clocks sys_clk] -min 1.000 [get_ports pressure_valve_ctrl]
set_output_delay -clock [get_clocks sys_clk] -max 7.000 [get_ports pressure_valve_ctrl]

set_output_delay -clock [get_clocks sys_clk] -min 1.000 [get_ports alarm_signal]
set_output_delay -clock [get_clocks sys_clk] -max 7.000 [get_ports alarm_signal]

set_output_delay -clock [get_clocks sys_clk] -min 1.000 [get_ports {system_status[*]}]
set_output_delay -clock [get_clocks sys_clk] -max 7.000 [get_ports {system_status[*]}]

# Reset signal timing
set_input_delay -clock [get_clocks sys_clk] -min 0.000 [get_ports rst_n]
set_input_delay -clock [get_clocks sys_clk] -max 2.000 [get_ports rst_n]

# Set false paths for reset (asynchronous reset)
set_false_path -from [get_ports rst_n] -to [all_registers]

# Set maximum delay for combinational paths
set_max_delay 8.000 -from [all_inputs] -to [all_outputs]

# Group related paths for better timing analysis
group_path -name input_to_reg -from [all_inputs] -to [all_registers]
group_path -name reg_to_output -from [all_registers] -to [all_outputs]
group_path -name reg_to_reg -from [all_registers] -to [all_registers]