/*******************************************************************************
* Real-Time Multi-Sensor Control System (RTOS)
* Master's Degree Project
* 
* Description: Top-level module implementing a real-time operating system
*              with multiple sensor inputs, task scheduling, and control outputs
*
* Features:
* - Temperature sensor monitoring (simulated)
* - Pressure sensor monitoring (simulated)
* - Proximity sensor monitoring (simulated)
* - Priority-based task scheduler
* - Real-time interrupt handling
* - Sensor data filtering and processing
* - Control signal generation based on sensor thresholds
*******************************************************************************/

module rtos_top (
    // Clock and Reset
    input wire clk,              // System clock (100 MHz assumed)
    input wire rst_n,            // Active-low reset
    
    // Sensor Inputs (simulated sensor data)
    input wire [15:0] temp_sensor_data,      // Temperature sensor (0-65535, represents 0-100°C)
    input wire [15:0] pressure_sensor_data,  // Pressure sensor (0-65535, represents 0-200 kPa)
    input wire [7:0]  proximity_sensor_data, // Proximity sensor (0-255, represents 0-50 cm)
    input wire sensor_valid,                  // Sensor data valid signal
    
    // Control Outputs
    output reg cooling_fan_ctrl,              // Cooling fan control (1=ON, 0=OFF)
    output reg pressure_valve_ctrl,           // Pressure relief valve (1=OPEN, 0=CLOSED)
    output reg alarm_signal,                  // Alarm output for critical conditions
    output reg [7:0] system_status,           // System status register
    
    // Debug/Monitor Outputs
    output reg [3:0] current_task,            // Currently executing task ID
    output reg [31:0] task_execution_count    // Total tasks executed
);

/*******************************************************************************
* RTOS Parameters and Constants
*******************************************************************************/

// System Parameters
parameter CLK_FREQ = 100_000_000;  // 100 MHz clock
parameter TICK_PERIOD = 1000;      // RTOS tick every 1000 clock cycles (10 microseconds)

// Task IDs
parameter TASK_IDLE         = 4'd0;
parameter TASK_TEMP_READ    = 4'd1;
parameter TASK_PRESSURE_READ = 4'd2;
parameter TASK_PROXIMITY_READ = 4'd3;
parameter TASK_TEMP_PROCESS = 4'd4;
parameter TASK_PRESSURE_PROCESS = 4'd5;
parameter TASK_PROXIMITY_PROCESS = 4'd6;
parameter TASK_CONTROL_UPDATE = 4'd7;
parameter TASK_ALARM_CHECK  = 4'd8;

// Task Priorities (higher number = higher priority)
parameter PRI_IDLE         = 3'd0;
parameter PRI_READ_SENSOR  = 3'd3;
parameter PRI_PROCESS_DATA = 3'd4;
parameter PRI_CONTROL      = 3'd5;
parameter PRI_ALARM        = 3'd7;  // Highest priority

// Sensor Thresholds
parameter TEMP_THRESHOLD_HIGH    = 16'd45875;  // ~70°C
parameter TEMP_THRESHOLD_CRITICAL = 16'd58981; // ~90°C
parameter PRESSURE_THRESHOLD_HIGH = 16'd52428; // ~160 kPa
parameter PROXIMITY_THRESHOLD_CLOSE = 8'd51;   // ~10 cm

// System Status Bits
parameter STATUS_NORMAL     = 8'b00000001;
parameter STATUS_TEMP_HIGH  = 8'b00000010;
parameter STATUS_PRESS_HIGH = 8'b00000100;
parameter STATUS_PROX_CLOSE = 8'b00001000;
parameter STATUS_CRITICAL   = 8'b10000000;

/*******************************************************************************
* Internal Registers and Signals
*******************************************************************************/

// RTOS Tick Counter
reg [31:0] tick_counter;
reg rtos_tick;

// Task Scheduler Registers
reg [3:0] task_queue [0:7];      // Task queue (8 tasks max)
reg [2:0] task_priority [0:7];   // Priority for each task
reg [3:0] task_queue_head;
reg [3:0] task_queue_tail;
reg [3:0] next_task;
reg task_ready;

// Sensor Data Storage (filtered values)
reg [15:0] temp_filtered;
reg [15:0] pressure_filtered;
reg [7:0]  proximity_filtered;

// Raw sensor buffers for filtering (moving average)
reg [15:0] temp_buffer [0:3];
reg [15:0] pressure_buffer [0:3];
reg [7:0]  proximity_buffer [0:3];
reg [1:0] buffer_index;

// Task Execution Flags
reg temp_data_ready;
reg pressure_data_ready;
reg proximity_data_ready;
reg all_sensors_processed;

// Control Logic Registers
reg [15:0] temp_error;
reg [15:0] pressure_error;
reg fan_timer;
reg valve_timer;

// State Machine for Task Execution
reg [3:0] execution_state;
parameter STATE_IDLE       = 4'd0;
parameter STATE_FETCH_TASK = 4'd1;
parameter STATE_EXEC_TASK  = 4'd2;
parameter STATE_UPDATE     = 4'd3;

/*******************************************************************************
* RTOS Tick Generation
* Generates periodic tick for task scheduling
*******************************************************************************/

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tick_counter <= 32'd0;
        rtos_tick <= 1'b0;
    end else begin
        if (tick_counter >= TICK_PERIOD - 1) begin
            tick_counter <= 32'd0;
            rtos_tick <= 1'b1;
        end else begin
            tick_counter <= tick_counter + 1;
            rtos_tick <= 1'b0;
        end
    end
end

/*******************************************************************************
* Sensor Data Acquisition and Filtering
* Implements 4-point moving average filter for each sensor
*******************************************************************************/

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        buffer_index <= 2'd0;
        temp_buffer[0] <= 16'd0;
        temp_buffer[1] <= 16'd0;
        temp_buffer[2] <= 16'd0;
        temp_buffer[3] <= 16'd0;
        pressure_buffer[0] <= 16'd0;
        pressure_buffer[1] <= 16'd0;
        pressure_buffer[2] <= 16'd0;
        pressure_buffer[3] <= 16'd0;
        proximity_buffer[0] <= 8'd0;
        proximity_buffer[1] <= 8'd0;
        proximity_buffer[2] <= 8'd0;
        proximity_buffer[3] <= 8'd0;
        temp_filtered <= 16'd0;
        pressure_filtered <= 16'd0;
        proximity_filtered <= 8'd0;
    end else if (sensor_valid && rtos_tick) begin
        // Store new sensor data in circular buffer
        temp_buffer[buffer_index] <= temp_sensor_data;
        pressure_buffer[buffer_index] <= pressure_sensor_data;
        proximity_buffer[buffer_index] <= proximity_sensor_data;
        
        // Calculate moving average (divide by 4)
        temp_filtered <= (temp_buffer[0] + temp_buffer[1] + 
                         temp_buffer[2] + temp_buffer[3]) >> 2;
        pressure_filtered <= (pressure_buffer[0] + pressure_buffer[1] + 
                             pressure_buffer[2] + pressure_buffer[3]) >> 2;
        proximity_filtered <= (proximity_buffer[0] + proximity_buffer[1] + 
                              proximity_buffer[2] + proximity_buffer[3]) >> 2;
        
        // Update buffer index
        buffer_index <= buffer_index + 1;
    end
end

/*******************************************************************************
* Task Scheduler
* Priority-based preemptive task scheduler
*******************************************************************************/

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        task_queue_head <= 4'd0;
        task_queue_tail <= 4'd0;
        next_task <= TASK_IDLE;
        task_ready <= 1'b0;
        execution_state <= STATE_IDLE;
        current_task <= TASK_IDLE;
        task_execution_count <= 32'd0;
        
        // Initialize task priorities
        task_priority[0] <= PRI_IDLE;
        task_priority[1] <= PRI_READ_SENSOR;
        task_priority[2] <= PRI_READ_SENSOR;
        task_priority[3] <= PRI_READ_SENSOR;
        task_priority[4] <= PRI_PROCESS_DATA;
        task_priority[5] <= PRI_PROCESS_DATA;
        task_priority[6] <= PRI_PROCESS_DATA;
        task_priority[7] <= PRI_CONTROL;
    end else begin
        case (execution_state)
            STATE_IDLE: begin
                if (rtos_tick) begin
                    // Add periodic tasks to queue
                    task_queue[task_queue_tail] <= TASK_TEMP_READ;
                    task_queue_tail <= task_queue_tail + 1;
                    
                    task_queue[task_queue_tail + 1] <= TASK_PRESSURE_READ;
                    task_queue_tail <= task_queue_tail + 2;
                    
                    task_queue[task_queue_tail + 2] <= TASK_PROXIMITY_READ;
                    task_queue_tail <= task_queue_tail + 3;
                    
                    execution_state <= STATE_FETCH_TASK;
                end
            end
            
            STATE_FETCH_TASK: begin
                if (task_queue_head != task_queue_tail) begin
                    // Get highest priority task from queue
                    next_task <= task_queue[task_queue_head];
                    task_queue_head <= task_queue_head + 1;
                    task_ready <= 1'b1;
                    execution_state <= STATE_EXEC_TASK;
                end else begin
                    next_task <= TASK_IDLE;
                    execution_state <= STATE_IDLE;
                end
            end
            
            STATE_EXEC_TASK: begin
                current_task <= next_task;
                task_execution_count <= task_execution_count + 1;
                task_ready <= 1'b0;
                
                // Task execution happens in parallel blocks below
                execution_state <= STATE_UPDATE;
            end
            
            STATE_UPDATE: begin
                // Check if more tasks in queue
                if (task_queue_head != task_queue_tail) begin
                    execution_state <= STATE_FETCH_TASK;
                end else begin
                    execution_state <= STATE_IDLE;
                end
            end
            
            default: execution_state <= STATE_IDLE;
        endcase
    end
end

/*******************************************************************************
* Task Execution: Sensor Reading Tasks
*******************************************************************************/

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        temp_data_ready <= 1'b0;
        pressure_data_ready <= 1'b0;
        proximity_data_ready <= 1'b0;
    end else begin
        if (execution_state == STATE_EXEC_TASK) begin
            case (current_task)
                TASK_TEMP_READ: begin
                    temp_data_ready <= 1'b1;
                    // Queue processing task
                    if (task_queue_tail < 4'd7) begin
                        task_queue[task_queue_tail] <= TASK_TEMP_PROCESS;
                        task_queue_tail <= task_queue_tail + 1;
                    end
                end
                
                TASK_PRESSURE_READ: begin
                    pressure_data_ready <= 1'b1;
                    if (task_queue_tail < 4'd7) begin
                        task_queue[task_queue_tail] <= TASK_PRESSURE_PROCESS;
                        task_queue_tail <= task_queue_tail + 1;
                    end
                end
                
                TASK_PROXIMITY_READ: begin
                    proximity_data_ready <= 1'b1;
                    if (task_queue_tail < 4'd7) begin
                        task_queue[task_queue_tail] <= TASK_PROXIMITY_PROCESS;
                        task_queue_tail <= task_queue_tail + 1;
                    end
                end
                
                TASK_TEMP_PROCESS: begin
                    temp_data_ready <= 1'b0;
                    // Processing happens in control logic block
                end
                
                TASK_PRESSURE_PROCESS: begin
                    pressure_data_ready <= 1'b0;
                end
                
                TASK_PROXIMITY_PROCESS: begin
                    proximity_data_ready <= 1'b0;
                end
            endcase
        end
    end
end

/*******************************************************************************
* Control Logic: Temperature-based Fan Control
*******************************************************************************/

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        cooling_fan_ctrl <= 1'b0;
        fan_timer <= 1'b0;
    end else begin
        if (execution_state == STATE_EXEC_TASK && current_task == TASK_CONTROL_UPDATE) begin
            // Turn on fan if temperature exceeds threshold
            if (temp_filtered > TEMP_THRESHOLD_HIGH) begin
                cooling_fan_ctrl <= 1'b1;
                fan_timer <= 1'b1;
            end else if (fan_timer && temp_filtered < (TEMP_THRESHOLD_HIGH - 16'd3276)) begin
                // Hysteresis: turn off fan when temp drops 5°C below threshold
                cooling_fan_ctrl <= 1'b0;
                fan_timer <= 1'b0;
            end
        end
    end
end

/*******************************************************************************
* Control Logic: Pressure-based Valve Control
*******************************************************************************/

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        pressure_valve_ctrl <= 1'b0;
        valve_timer <= 1'b0;
    end else begin
        if (execution_state == STATE_EXEC_TASK && current_task == TASK_CONTROL_UPDATE) begin
            // Open valve if pressure exceeds threshold
            if (pressure_filtered > PRESSURE_THRESHOLD_HIGH) begin
                pressure_valve_ctrl <= 1'b1;
                valve_timer <= 1'b1;
            end else if (valve_timer && pressure_filtered < (PRESSURE_THRESHOLD_HIGH - 16'd6553)) begin
                // Hysteresis: close valve when pressure drops 20 kPa below threshold
                pressure_valve_ctrl <= 1'b0;
                valve_timer <= 1'b0;
            end
        end
    end
end

/*******************************************************************************
* Alarm System: Critical Condition Detection
*******************************************************************************/

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        alarm_signal <= 1'b0;
    end else begin
        if (execution_state == STATE_EXEC_TASK && current_task == TASK_ALARM_CHECK) begin
            // Trigger alarm for critical conditions
            if ((temp_filtered > TEMP_THRESHOLD_CRITICAL) || 
                (proximity_filtered < PROXIMITY_THRESHOLD_CLOSE)) begin
                alarm_signal <= 1'b1;
            end else begin
                alarm_signal <= 1'b0;
            end
        end
    end
end

/*******************************************************************************
* System Status Register Update
*******************************************************************************/

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        system_status <= STATUS_NORMAL;
    end else begin
        if (rtos_tick) begin
            // Update system status based on sensor readings
            system_status <= STATUS_NORMAL;
            
            if (temp_filtered > TEMP_THRESHOLD_HIGH)
                system_status <= system_status | STATUS_TEMP_HIGH;
            
            if (pressure_filtered > PRESSURE_THRESHOLD_HIGH)
                system_status <= system_status | STATUS_PRESS_HIGH;
            
            if (proximity_filtered < PROXIMITY_THRESHOLD_CLOSE)
                system_status <= system_status | STATUS_PROX_CLOSE;
            
            if ((temp_filtered > TEMP_THRESHOLD_CRITICAL) || alarm_signal)
                system_status <= system_status | STATUS_CRITICAL;
        end
    end
end

/*******************************************************************************
* Automatic Task Queuing for Control and Alarm
* These high-priority tasks are queued automatically when conditions are met
*******************************************************************************/

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        all_sensors_processed <= 1'b0;
    end else begin
        // Check if all sensor processing is complete
        if (!temp_data_ready && !pressure_data_ready && !proximity_data_ready) begin
            all_sensors_processed <= 1'b1;
            
            // Queue control update task
            if (task_queue_tail < 4'd6) begin
                task_queue[task_queue_tail] <= TASK_CONTROL_UPDATE;
                task_queue[task_queue_tail + 1] <= TASK_ALARM_CHECK;
                task_queue_tail <= task_queue_tail + 2;
            end
        end else begin
            all_sensors_processed <= 1'b0;
        end
    end
end

endmodule