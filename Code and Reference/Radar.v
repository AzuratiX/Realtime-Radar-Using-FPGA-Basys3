`timescale 1ns / 1ps
module Servo_Radar_VGA (
    input clk_100MHz,
    input start,              // Start (reset) button
    input play_pause,         // Play/Pause button
    input echo,               // Echo from HC-SR04
    output reg trigger,       // Trigger to HC-SR04
    output reg servo,         // PWM to servo
    output reg [2:0] led,     // LEDs for distance indication
    output [3:0] vga_red,
    output [3:0] vga_green,
    output [3:0] vga_blue,
    output hsync,
    output vsync
);

    // -------------------------
    // Clock Divider for 25MHz VGA Clock
    // -------------------------
    reg [1:0] clk_div = 0;
    reg pix_clk = 0;
    always @(posedge clk_100MHz) begin
        clk_div <= clk_div + 1;
        if (clk_div == 1) begin
            clk_div <= 0;
            pix_clk <= ~pix_clk;
        end
    end

    // -------------------------
    // Servo and Ultrasonic Logic
    // -------------------------
    reg [20:0] counter;     // 0 to 2,000,000 for 20ms at 100MHz
    reg [20:0] pulse_width; // 100,000 to 200,000 cycles (1ms to 2ms)
    reg direction;
    reg paused;
    reg last_play_pause;

    // Ultrasonic states and regs
    reg [2:0] sensor_state;
    reg [15:0] trigger_count; 
    reg [31:0] echo_count;    
    reg [31:0] distance_cm;  

    localparam S_IDLE          = 3'd0;
    localparam S_TRIGGER       = 3'd1;
    localparam S_WAIT_ECHO_HIGH= 3'd2;
    localparam S_WAIT_ECHO_LOW = 3'd3;
    localparam S_DONE          = 3'd4;

    initial begin
        counter = 0;
        pulse_width = 100000; // start at 1ms
        direction = 0;
        paused = 0;
        last_play_pause = 0;
        sensor_state = S_IDLE;
        trigger = 0;
        trigger_count = 0;
        echo_count = 0;
        distance_cm = 0;
        led = 3'b000;
    end

    always @(posedge clk_100MHz) begin
        // Play/Pause toggle detection
        if (play_pause && !last_play_pause) begin
            paused <= ~paused;
        end
        last_play_pause <= play_pause;

        // Start button (reset functionality)
        if (start) begin
            counter <= 0;
            pulse_width <= 100000;
            direction <= 0;
            paused <= 0;
            sensor_state <= S_IDLE;
            trigger <= 0;
            echo_count <= 0;
            distance_cm <= 0;
            led <= 3'b000;
        end else begin
            if (!paused) begin
                if (counter < 2000000) begin
                    counter <= counter + 1;
                end else begin
                    counter <= 0;
                    // Move servo angle
                    if (direction == 0) begin
                        pulse_width <= pulse_width + 1000; 
                        if (pulse_width >= 200000) direction <= 1;
                    end else begin
                        pulse_width <= pulse_width - 1000;
                        if (pulse_width <= 100000) direction <= 0;
                    end
                    // Trigger ultrasonic measurement each cycle
                    sensor_state <= S_TRIGGER;
                    trigger_count <= 0;
                    echo_count <= 0;
                    trigger <= 1;
                end
            end else begin
                // Paused: hold angle and don't trigger sensor repeatedly
                if (counter < 2000000) begin
                    counter <= counter + 1;
                end else begin
                    counter <= 0;
                    sensor_state <= S_IDLE;
                end
            end

            // Servo PWM output
            servo <= (counter < pulse_width) ? 1 : 0;

            // Ultrasonic Sensor FSM
            case(sensor_state)
                S_IDLE: begin end
                S_TRIGGER: begin
                    if (trigger_count < 1000) begin // 10us at 100MHz
                        trigger_count <= trigger_count + 1;
                        trigger <= 1;
                    end else begin
                        trigger <= 0;
                        sensor_state <= S_WAIT_ECHO_HIGH;
                    end
                end
                S_WAIT_ECHO_HIGH: begin
                    if (echo == 1) begin
                        echo_count <= 0;
                        sensor_state <= S_WAIT_ECHO_LOW;
                    end
                end
                S_WAIT_ECHO_LOW: begin
                    if (echo == 1) begin
                        echo_count <= echo_count + 1;
                    end else begin
                        sensor_state <= S_DONE;
                    end
                end
                S_DONE: begin
                    // distance in cm approx.
                    distance_cm <= echo_count / 5800;
                    sensor_state <= S_IDLE;
                end
            endcase

            

            // LED logic based on distance
            if (distance_cm < 30) begin
                led <= 3'b001;
            end else if (distance_cm < 40) begin
                led <= 3'b010;
            end else if (distance_cm < 50) begin
                led <= 3'b100;
            end else begin
                led <= 3'b000;
            end
        end
    end

    // -------------------------
    // VGA Controller
    // -------------------------
    localparam H_VISIBLE = 640;
    localparam H_FRONT   = 16;
    localparam H_SYNC    = 96;
    localparam H_BACK    = 48;
    localparam H_TOTAL   = H_VISIBLE + H_FRONT + H_SYNC + H_BACK; // 800

    localparam V_VISIBLE = 480;
    localparam V_FRONT   = 10;
    localparam V_SYNC    = 2;
    localparam V_BACK    = 33;
    localparam V_TOTAL   = V_VISIBLE + V_FRONT + V_SYNC + V_BACK; // 525

    reg [9:0] hcount = 0;
    reg [9:0] vcount = 0;

    always @(posedge pix_clk) begin
        if (hcount < H_TOTAL - 1) begin
            hcount <= hcount + 1;
        end else begin
            hcount <= 0;
            if (vcount < V_TOTAL - 1) vcount <= vcount + 1;
            else vcount <= 0;
        end
    end

    reg hsync_r, vsync_r;
    always @(posedge pix_clk) begin
        hsync_r <= ~((hcount >= (H_VISIBLE + H_FRONT)) && (hcount < (H_VISIBLE + H_FRONT + H_SYNC)));
        vsync_r <= ~((vcount >= (V_VISIBLE + V_FRONT)) && (vcount < (V_VISIBLE + V_FRONT + V_SYNC)));
    end

    assign hsync = hsync_r;
    assign vsync = vsync_r;

    wire visible_area = (hcount < H_VISIBLE && vcount < V_VISIBLE);

    // -------------------------
    // Beam Angle Calculation
    // -------------------------
    reg [10:0] beam_angle = 0; // 0 to 359 degrees
    reg [31:0] angle_counter = 0;
    localparam ANGLE_INCREMENT_INTERVAL = 290000; // Adjust for sweep speed
    reg direction_r = 0; // 0 = increasing angle, 1 = decreasing angle
    always @(posedge pix_clk) begin
        if (!paused) begin
            if (angle_counter < ANGLE_INCREMENT_INTERVAL) begin
                angle_counter <= angle_counter + 1;
            end else begin
                angle_counter <= 0;
                if (direction_r == 0) begin
                    beam_angle <= (beam_angle < 180) ? beam_angle + 1 : beam_angle;
                    if (beam_angle == 180) direction_r <= 1; // Change direction
                end else begin
                    beam_angle <= (beam_angle > 0) ? beam_angle - 1 : beam_angle;
                    if (beam_angle == 0) direction_r <= 0; // Change direction
                end
            end
        end else begin
            angle_counter <= 0;
        end
    end


    // -------------------------
    // Sine and Cosine Lookup Table
    // -------------------------
    reg signed [15:0] sine_table [0:359];
    reg signed [15:0] cosine_table [0:359];
    integer i;
    real sine_val_real, cosine_val_real;
    initial begin
        for (i = 0; i < 360; i = i + 1) begin
            sine_val_real = $sin(i * 3.141592653589793 / 180.0);
            cosine_val_real = $cos(i * 3.141592653589793 / 180.0);
            sine_table[i] = $rtoi(sine_val_real * 256.0); // Higher precision
            cosine_table[i] = $rtoi(cosine_val_real * 256.0); // Higher precision
        end
    end

    wire signed [15:0] sine_val = sine_table[beam_angle];
    wire signed [15:0] cosine_val = cosine_table[beam_angle];

    // -------------------------
    // Beam Endpoint Calculation
    // -------------------------
    localparam CX = 320;
    localparam CY = 240;
    
    localparam RADAR_RADIUS = 200;
    wire [9:0] beam_x = CX + (cosine_val * RADAR_RADIUS) / 256;
    wire [9:0] beam_y = CY - (sine_val * RADAR_RADIUS) / 256;

    wire [9:0] obj_radius_pixels = (distance_cm > (RADAR_RADIUS / 2)) ? 0 : (distance_cm * 2);
    wire [9:0] obj_x = CX + (cosine_val * obj_radius_pixels) / 256;
    wire [9:0] obj_y = CY - (sine_val * obj_radius_pixels) / 256;

    // -------------------------
    // Radar Beam and Object Display Logic
    // -------------------------
    localparam BEAM_THICKNESS = 2;
    reg [3:0] red_reg;
    reg [3:0] green_reg;
    reg [3:0] blue_reg;

    integer radius_step;
    integer dist_sq;

    reg signed [31:0] cross;
    reg signed [31:0] cross_abs;
    reg signed [31:0] threshold;
    
    always @(*) begin
        if (!visible_area) begin
            red_reg = 0;
            green_reg = 0;
            blue_reg = 0;
        end else begin
            red_reg = 0;
            green_reg = 0;
            blue_reg = 0;

            dist_sq = (hcount - CX)*(hcount - CX) + (vcount - CY)*(vcount - CY);
            for (radius_step = 50; radius_step <= RADAR_RADIUS; radius_step = radius_step + 50) begin
                if (dist_sq > (radius_step - 2)*(radius_step - 2) && dist_sq < (radius_step + 2)*(radius_step + 2)) begin
                    green_reg = 4'hF;
                end
            end

            cross = ((beam_x - CX) * (vcount - CY)) - ((beam_y - CY) * (hcount - CX));
            cross_abs = (cross < 0) ? -cross : cross;
            threshold = BEAM_THICKNESS * RADAR_RADIUS / 2;

            if (cross_abs < threshold) begin
                red_reg = 4'hF;
                green_reg = 4'hF;
                blue_reg = 4'hF;
            end

            if (distance_cm > 0 && distance_cm < (RADAR_RADIUS / 2)) begin
                if ((hcount > obj_x - 3 && hcount < obj_x + 3) && (vcount > obj_y - 3 && vcount < obj_y + 3)) begin
                    red_reg = 4'hF;
                    green_reg = 0;
                    blue_reg = 0;
                end
            end
        end
    end

    assign vga_red = red_reg;
    assign vga_green = green_reg;
    assign vga_blue = blue_reg;

endmodule