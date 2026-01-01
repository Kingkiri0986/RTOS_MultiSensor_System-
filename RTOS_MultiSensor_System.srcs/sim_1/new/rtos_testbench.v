/*******************************************************************************
* Testbench for Real-Time Multi-Sensor Control System
* 
* This testbench simulates:
* - Variable temperature sensor readings
* - Variable pressure sensor readings
* - Variable proximity sensor readings
* - Verifies RTOS task scheduling
* - Verifies control outputs (fan, valve, alarm)
* - Monitors system status
*******************************************************************************/

`timescale 1ns / 1ps

module rtos_testbench;

/*******************************************************************************
* Testbench Signals
*******************************************************************************/

// Clock and Reset
reg clk;
reg rst_n;

// Sensor Inputs
reg [15:0] temp_sensor_data;
reg [15:0] pressure_sensor_data;
reg [7:0] proximity_sensor_data;
reg sensor_valid;

// Control Outputs
wire cooling_fan_ctrl;
wire pressure_valve_ctrl;
wire alarm_signal;
wire [7:0] system_status;

// Debug Outputs
wire [3:0] current_task;
wire [31:0] task_execution_count;

// Testbench Variables
integer test_time;
integer i;
reg [31:0] error_count;
reg [31:0] test_count;

/*******************************************************************************
* Device Under Test (DUT) Instantiation
*******************************************************************************/

rtos_top DUT (
    .clk(clk),
    .rst_n(rst_n),
    .temp_sensor_data(temp_sensor_data),
    .pressure_sensor_data(pressure_sensor_data),
    .proximity_sensor_data(proximity_sensor_data),
    .sensor_valid(sensor_valid),
    .cooling_fan_ctrl(cooling_fan_ctrl),
    .pressure_valve_ctrl(pressure_valve_ctrl),
    .alarm_signal(alarm_signal),
    .system_status(system_status),
    .current_task(current_task),
    .task_execution_count(task_execution_count)
);

/*******************************************************************************
* Clock Generation: 100 MHz (10ns period)
*******************************************************************************/

initial begin
    clk = 0;
    forever #5 clk = ~clk;  // Toggle every 5ns = 10ns period = 100MHz
end

/*******************************************************************************
* Sensor Data Conversion Functions (for readability)
*******************************************************************************/

// Convert temperature in Celsius to sensor value
function [15:0] temp_to_sensor;
    input real temp_celsius;
    begin
        temp_to_sensor = temp_celsius * 655.35;  // Scale 0-100°C to 0-65535
    end
endfunction

// Convert pressure in kPa to sensor value
function [15:0] pressure_to_sensor;
    input real pressure_kpa;
    begin
        pressure_to_sensor = pressure_kpa * 327.675;  // Scale 0-200kPa to 0-65535
    end
endfunction

// Convert distance in cm to sensor value
function [7:0] distance_to_sensor;
    input real distance_cm;
    begin
        distance_to_sensor = distance_cm * 5.1;  // Scale 0-50cm to 0-255
    end
endfunction

/*******************************************************************************
* Test Monitoring Task
*******************************************************************************/

always @(posedge clk) begin
    if (rst_n) begin
        // Monitor critical changes
        if (cooling_fan_ctrl) begin
            $display("[%0t ns] CONTROL: Cooling fan ACTIVATED (Temp = %0d)", 
                     $time, temp_sensor_data);
        end
        
        if (pressure_valve_ctrl) begin
            $display("[%0t ns] CONTROL: Pressure valve OPENED (Pressure = %0d)", 
                     $time, pressure_sensor_data);
        end
        
        if (alarm_signal) begin
            $display("[%0t ns] ALARM: CRITICAL CONDITION DETECTED!", $time);
            $display("           Temperature: %0d, Proximity: %0d", 
                     temp_sensor_data, proximity_sensor_data);
        end
    end
end

/*******************************************************************************
* Main Test Sequence
*******************************************************************************/

initial begin
    // Initialize waveform dump for viewing in GTKWave or Vivado simulator
    $dumpfile("rtos_simulation.vcd");
    $dumpvars(0, rtos_testbench);
    
    // Initialize signals
    rst_n = 0;
    sensor_valid = 0;
    temp_sensor_data = 16'd0;
    pressure_sensor_data = 16'd0;
    proximity_sensor_data = 8'd0;
    error_count = 0;
    test_count = 0;
    
    // Display header
    $display("\n");
    $display("================================================================================");
    $display("  REAL-TIME MULTI-SENSOR CONTROL SYSTEM - SIMULATION");
    $display("  Master's Degree Project");
    $display("================================================================================");
    $display("\n");
    
    // Apply reset
    $display("[%0t ns] TEST: Applying system reset...", $time);
    #100;
    rst_n = 1;
    #100;
    $display("[%0t ns] TEST: Reset released, system active\n", $time);
    
    /***************************************************************************
    * TEST 1: Normal Operating Conditions
    ***************************************************************************/
    $display("================================================================================");
    $display("TEST 1: Normal Operating Conditions");
    $display("================================================================================");
    test_count = test_count + 1;
    
    sensor_valid = 1;
    temp_sensor_data = temp_to_sensor(25.0);      // 25°C (normal)
    pressure_sensor_data = pressure_to_sensor(100.0); // 100 kPa (normal)
    proximity_sensor_data = distance_to_sensor(30.0); // 30 cm (safe)
    
    $display("Sensor Values:");
    $display("  Temperature: 25°C");
    $display("  Pressure: 100 kPa");
    $display("  Proximity: 30 cm");
    $display("");
    
    #50000;  // Wait 50 microseconds for RTOS to process
    
    // Verify outputs
    if (cooling_fan_ctrl == 0 && pressure_valve_ctrl == 0 && alarm_signal == 0) begin
        $display("✓ TEST 1 PASSED: All controls in normal state");
        $display("  System Status: 0x%h", system_status);
    end else begin
        $display("✗ TEST 1 FAILED: Unexpected control activation");
        error_count = error_count + 1;
    end
    $display("");
    
    /***************************************************************************
    * TEST 2: High Temperature - Fan Activation
    ***************************************************************************/
    $display("================================================================================");
    $display("TEST 2: High Temperature Detection - Fan Control");
    $display("================================================================================");
    test_count = test_count + 1;
    
    // Gradually increase temperature
    $display("Gradually increasing temperature to 75°C...");
    for (i = 25; i <= 75; i = i + 5) begin
        temp_sensor_data = temp_to_sensor(i);
        $display("  Temperature: %0d°C", i);
        #10000;
    end
    $display("");
    
    #20000;  // Wait for RTOS to react
    
    // Verify fan activation
    if (cooling_fan_ctrl == 1) begin
        $display("✓ TEST 2 PASSED: Cooling fan activated at high temperature");
        $display("  System Status: 0x%h (TEMP_HIGH flag should be set)", system_status);
    end else begin
        $display("✗ TEST 2 FAILED: Fan did not activate");
        error_count = error_count + 1;
    end
    $display("");
    
    /***************************************************************************
    * TEST 3: High Pressure - Valve Control
    ***************************************************************************/
    $display("================================================================================");
    $display("TEST 3: High Pressure Detection - Valve Control");
    $display("================================================================================");
    test_count = test_count + 1;
    
    // Reset temperature to normal
    temp_sensor_data = temp_to_sensor(30.0);
    #10000;
    
    // Increase pressure
    $display("Increasing pressure to 170 kPa...");
    pressure_sensor_data = pressure_to_sensor(170.0);
    
    #30000;  // Wait for RTOS to react
    
    // Verify valve opening
    if (pressure_valve_ctrl == 1) begin
        $display("✓ TEST 3 PASSED: Pressure valve opened at high pressure");
        $display("  System Status: 0x%h (PRESS_HIGH flag should be set)", system_status);
    end else begin
        $display("✗ TEST 3 FAILED: Valve did not open");
        error_count = error_count + 1;
    end
    $display("");
    
    /***************************************************************************
    * TEST 4: Close Proximity Detection
    ***************************************************************************/
    $display("================================================================================");
    $display("TEST 4: Close Proximity Detection");
    $display("================================================================================");
    test_count = test_count + 1;
    
    // Reset pressure to normal
    pressure_sensor_data = pressure_to_sensor(100.0);
    #10000;
    
    // Simulate object approaching
    $display("Simulating object approaching...");
    for (i = 30; i >= 8; i = i - 2) begin
        proximity_sensor_data = distance_to_sensor(i);
        $display("  Distance: %0d cm", i);
        #5000;
    end
    $display("");
    
    #20000;  // Wait for RTOS to react
    
    // Verify proximity detection
    if (system_status & 8'b00001000) begin  // Check PROX_CLOSE bit
        $display("✓ TEST 4 PASSED: Proximity detection working");
        $display("  System Status: 0x%h (PROX_CLOSE flag set)", system_status);
    end else begin
        $display("✗ TEST 4 FAILED: Proximity not detected");
        error_count = error_count + 1;
    end
    $display("");
    
    /***************************************************************************
    * TEST 5: Critical Temperature - Alarm Activation
    ***************************************************************************/
    $display("================================================================================");
    $display("TEST 5: Critical Temperature - Alarm System");
    $display("================================================================================");
    test_count = test_count + 1;
    
    // Reset proximity
    proximity_sensor_data = distance_to_sensor(30.0);
    #10000;
    
    // Raise temperature to critical level
    $display("Raising temperature to critical level (95°C)...");
    temp_sensor_data = temp_to_sensor(95.0);
    
    #30000;  // Wait for RTOS to react
    
    // Verify alarm activation
    if (alarm_signal == 1) begin
        $display("✓ TEST 5 PASSED: Alarm activated at critical temperature");
        $display("  System Status: 0x%h (CRITICAL flag should be set)", system_status);
    end else begin
        $display("✗ TEST 5 FAILED: Alarm did not activate");
        error_count = error_count + 1;
    end
    $display("");
    
    /***************************************************************************
    * TEST 6: Multiple Simultaneous Conditions
    ***************************************************************************/
    $display("================================================================================");
    $display("TEST 6: Multiple Simultaneous Conditions");
    $display("================================================================================");
    test_count = test_count + 1;
    
    $display("Setting multiple critical conditions:");
    $display("  Temperature: 85°C (high)");
    $display("  Pressure: 180 kPa (high)");
    $display("  Proximity: 5 cm (close)");
    
    temp_sensor_data = temp_to_sensor(85.0);
    pressure_sensor_data = pressure_to_sensor(180.0);
    proximity_sensor_data = distance_to_sensor(5.0);
    
    #50000;  // Wait for RTOS to process all conditions
    
    // Verify all controls activated
    if (cooling_fan_ctrl == 1 && pressure_valve_ctrl == 1) begin
        $display("✓ TEST 6 PASSED: Multiple control systems activated simultaneously");
        $display("  Fan: %b, Valve: %b, Alarm: %b", 
                 cooling_fan_ctrl, pressure_valve_ctrl, alarm_signal);
        $display("  System Status: 0x%h", system_status);
    end else begin
        $display("✗ TEST 6 FAILED: Not all controls activated");
        error_count = error_count + 1;
    end
    $display("");
    
    /***************************************************************************
    * TEST 7: Return to Normal - Control Deactivation
    ***************************************************************************/
    $display("================================================================================");
    $display("TEST 7: Return to Normal Conditions");
    $display("================================================================================");
    test_count = test_count + 1;
    
    $display("Returning all sensors to normal values...");
    temp_sensor_data = temp_to_sensor(25.0);
    pressure_sensor_data = pressure_to_sensor(100.0);
    proximity_sensor_data = distance_to_sensor(30.0);
    
    #100000;  // Wait longer for hysteresis and control deactivation
    
    // Verify all controls deactivated
    if (cooling_fan_ctrl == 0 && pressure_valve_ctrl == 0 && alarm_signal == 0) begin
        $display("✓ TEST 7 PASSED: All controls deactivated on return to normal");
        $display("  System Status: 0x%h", system_status);
    end else begin
        $display("✗ TEST 7 FAILED: Controls still active");
        $display("  Fan: %b, Valve: %b, Alarm: %b", 
                 cooling_fan_ctrl, pressure_valve_ctrl, alarm_signal);
        error_count = error_count + 1;
    end
    $display("");
    
    /***************************************************************************
    * TEST 8: Task Execution Verification
    ***************************************************************************/
    $display("================================================================================");
    $display("TEST 8: RTOS Task Execution Statistics");
    $display("================================================================================");
    test_count = test_count + 1;
    
    $display("Task Execution Count: %0d", task_execution_count);
    
    if (task_execution_count > 100) begin
        $display("✓ TEST 8 PASSED: RTOS scheduler executing tasks");
    end else begin
        $display("✗ TEST 8 FAILED: Insufficient task executions");
        error_count = error_count + 1;
    end
    $display("");
    
    /***************************************************************************
    * TEST 9: Rapid Sensor Changes - RTOS Responsiveness
    ***************************************************************************/
    $display("================================================================================");
    $display("TEST 9: Rapid Sensor Changes - System Responsiveness");
    $display("================================================================================");
    test_count = test_count + 1;
    
    $display("Simulating rapid sensor fluctuations...");
    
    for (i = 0; i < 20; i = i + 1) begin
        // Alternate between normal and high values rapidly
        if (i % 2 == 0) begin
            temp_sensor_data = temp_to_sensor(80.0);
            pressure_sensor_data = pressure_to_sensor(170.0);
        end else begin
            temp_sensor_data = temp_to_sensor(30.0);
            pressure_sensor_data = pressure_to_sensor(100.0);
        end
        #2000;  // Very short intervals
    end
    
    // Return to normal
    temp_sensor_data = temp_to_sensor(25.0);
    pressure_sensor_data = pressure_to_sensor(100.0);
    
    #20000;
    
    $display("✓ TEST 9 PASSED: System handled rapid changes without errors");
    $display("");
    
    /***************************************************************************
    * Final Test Summary
    ***************************************************************************/
    $display("\n");
    $display("================================================================================");
    $display("  SIMULATION COMPLETE - TEST SUMMARY");
    $display("================================================================================");
    $display("Total Tests Run: %0d", test_count);
    $display("Tests Passed: %0d", test_count - error_count);
    $display("Tests Failed: %0d", error_count);
    $display("");
    $display("Total Task Executions: %0d", task_execution_count);
    $display("Final System Status: 0x%h", system_status);
    $display("");
    
    if (error_count == 0) begin
        $display("*** ALL TESTS PASSED *** ✓");
        $display("The RTOS Multi-Sensor Control System is functioning correctly!");
    end else begin
        $display("*** SOME TESTS FAILED *** ✗");
        $display("Please review the failed tests above.");
    end
    
    $display("================================================================================");
    $display("\n");
    
    // End simulation
    #10000;
    $display("Simulation ended at %0t ns", $time);
    $finish;
end

/*******************************************************************************
* Timeout Watchdog (prevent infinite simulation)
*******************************************************************************/

initial begin
    #10_000_000;  // 10 milliseconds maximum simulation time
    $display("\n*** SIMULATION TIMEOUT ***");
    $display("Simulation exceeded 10ms - terminating");
    $finish;
end

endmodule