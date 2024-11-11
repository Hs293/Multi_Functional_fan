module led_dht11_pwm_top(
    input clk,reset_p,
    inout dht11_data,
    output [7:0] seg_7,
    output [3:0] com,
    output led_r,
    output led_g,
    output led_b,
    output [4:0] led_select_mode
);

parameter   IDLE                      = 5'b00001;
parameter   COLD                      = 5'b00010;
parameter   GOOD                      = 5'b00100;
parameter   HOT                       = 5'b01000;
parameter   VERY_HOT                  = 5'b10000;

            reg     [4 : 0]     select_mode;
            reg     [4 : 0]     next_mode;
            reg     [7 : 0]     duty_r,duty_g,duty_b;
            wire    [15 : 0]    value_a;
            

            
dht11_fan_top(
clk, reset_p, dht11_data, value_a
);

pwm_128step pwm_led_r(
.clk(clk),
.reset_p(reset_p),
.duty(duty_r),
.pwm(led_r)
);

pwm_128step pwm_led_g(
.clk(clk),
.reset_p(reset_p),
.duty(duty_g),
.pwm(led_g)
);

pwm_128step pwm_led_b(
.clk(clk),
.reset_p(reset_p),
.duty(duty_b),
.pwm(led_b)
);

    always @(negedge clk or posedge reset_p) begin
        if (reset_p) begin
            select_mode <= IDLE;
        end else begin
            select_mode <= next_mode;
        end
    end

    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            next_mode <= IDLE;
            duty_b <= 0;
            duty_r <= 0;
            duty_g <= 0;
        end
        else begin
            case (select_mode)
                IDLE: begin
                    duty_b <= 0;
                    duty_r <= 0;
                    duty_g <= 0;
                    if (value_a < 22) begin
                    next_mode <= COLD;
                    end
                    else if (value_a >= 22 && value_a < 24) begin                     
                    next_mode <= GOOD;
                    end
                    else if (value_a >= 24 && value_a < 26) begin
                    next_mode <= HOT;
                    end
                    else if (value_a >= 26) begin
                    next_mode <= VERY_HOT;
                    end
                end
                COLD: begin
                    if (value_a >= 22) next_mode <= IDLE; //18
                    else begin                     
                    next_mode <= COLD;
                    duty_b <= 255;
                    duty_r <= 0;
                    duty_g <= 0;
                    end
                end
                GOOD: begin
                    if (value_a < 22 || value_a > 23) next_mode <= IDLE; //25
                    else begin                     
                    next_mode <= GOOD;
                    duty_b <= 0;
                    duty_r <= 0;
                    duty_g <= 255;
                    end
                end
                HOT: begin
                    if (value_a < 23 || value_a > 25) next_mode <= IDLE;//31
                    else begin                     
                    next_mode <= HOT;
                    duty_b <= 0;
                    duty_r <= 255;
                    duty_g <= 255;
                    end
                end    
                VERY_HOT: begin
                    if (value_a < 25) next_mode <= IDLE; //31
                    else begin         
                    next_mode <= VERY_HOT;
                    duty_b <= 0;
                    duty_r <= 255;
                    duty_g <= 0;
                    end
                end
            endcase
        end
    end
    
    assign led_select_mode = select_mode;
    wire [15:0] adc_value_bcd;
    bin_to_dec(
            .bin({4'b0, value_a[7:0]}),
            .bcd(adc_value_bcd)  );


 fnd_4digit_cntr fnd_on ( .clk(clk), .reset_p(reset_p), .com(com) , .value(adc_value_bcd), .seg_7(seg_7) );

endmodule

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
module dht11_fan_top(
    input   clk, 
    input   reset_p,
    inout   dht11_data,
    output  [15:0]value
);
    wire [15:0] led;
    wire [7:0] humidity, temperature;
    dht11_ctrl dht11(
    clk, reset_p,
    dht11_data, humidity, temperature, led);
    
   wire [15:0] bcd_humi, bcd_tmpr;
   
   assign value = {8'b0, temperature};
    
endmodule

/////////////////////////////////////////////////////////////////////////////////////////////////////////
module dht11_ctrl(
    input clk, reset_p,
    inout dht11_data,
    output reg [7:0] humidity, temperature,
    output [15:0] led
    );

    parameter S_IDLE        = 6'b00_0001;
    parameter S_LOW_18MS    = 6'b00_0010;
    parameter S_HIGH_20US   = 6'b00_0100;
    parameter S_LOW_80US    = 6'b00_1000;
    parameter S_HIGH_80US   = 6'b01_0000;
    parameter S_READ_DATA   = 6'b10_0000;
    
    parameter S_WAIT_PEDGE = 2'b01;
    parameter S_WAIT_NEDGE = 2'b10;
    
    reg [21:0] count_usec;
    wire clk_usec;
    reg count_usec_e;
    wire dht_nedge, dht_pedge;
    reg [5:0] state, next_state;
    reg [1:0] read_state;
    reg [39:0] temp_data;
    reg [5:0] data_count;
    reg dht11_buffer;
    
    
    clock_div_100 us_clk(.clk(clk), .reset_p(reset_p), .clk_div_100(clk_usec));
    
    edge_detector_p ed(
        .clk(clk), .reset_p(reset_p), .cp(dht11_data), 
        .n_edge(dht_nedge), .p_edge(dht_pedge));
    
    // count_usec
    always @(negedge clk or posedge reset_p)begin 
        if(reset_p)count_usec = 0;
        else if(clk_usec && count_usec_e)count_usec = count_usec + 1;
        else if(count_usec_e == 0)count_usec = 0;
    end
    
    // state
    always @(negedge clk or posedge reset_p)begin
        if(reset_p)state = S_IDLE;
        else state = next_state;
    end
    
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            count_usec_e = 0;
            next_state = S_IDLE;
            read_state = S_WAIT_PEDGE;
            data_count = 0;
            dht11_buffer = 'bz;
        end
        else begin
            case(state)
                S_IDLE: begin
                    if(count_usec < 22'd3_000_000)begin //3_000_000
                        count_usec_e = 1;
                        dht11_buffer = 'bz;
                    end
                    else begin
                        next_state = S_LOW_18MS;
                        count_usec_e = 0;
                    end
                end
                S_LOW_18MS:begin
                    if(count_usec < 22'd18_000)begin
                        dht11_buffer = 0;
                        count_usec_e = 1;
                    end
                    else begin
                        next_state = S_HIGH_20US;
                        count_usec_e = 0;
                        dht11_buffer = 'bz;
                    end
                end
                S_HIGH_20US:begin
                    count_usec_e = 1;
                    if(count_usec > 22'd100_000)begin
                        next_state = S_IDLE;
                        count_usec_e = 0;
                    end
                        if(dht_nedge)begin
                            next_state = S_LOW_80US;
                            count_usec_e = 0;
                        end
                    
                end
                S_LOW_80US:begin
                count_usec_e = 1;
                    if(count_usec > 22'd100_000)begin
                        next_state = S_IDLE;
                        count_usec_e = 0;
                    end
                    if(dht_pedge)begin
                        next_state = S_HIGH_80US;
                    end
                end
                S_HIGH_80US:begin
                    if(dht_nedge)begin
                        next_state = S_READ_DATA;
                    end
                end
                S_READ_DATA:begin
                    case(read_state)
                        S_WAIT_PEDGE:begin
                            if(dht_pedge)read_state = S_WAIT_NEDGE;
                            count_usec_e = 0;
                        end
                        S_WAIT_NEDGE:begin
                            if(dht_nedge)begin
                                if(count_usec < 45)begin
                                    temp_data = {temp_data[38:0], 1'b0};
                                end
                                else begin
                                    temp_data = {temp_data[38:0], 1'b1};
                                end
                                data_count = data_count + 1;
                                read_state = S_WAIT_PEDGE;
                            end
                            else count_usec_e = 1;
                            if(count_usec > 22'd700_000)begin
                                next_state = S_IDLE;
                                count_usec_e = 0;
                                data_count = 0;
                                read_state = S_WAIT_PEDGE;
                            end
                        end
                    endcase
                    if(data_count >= 40)begin
                        data_count = 0;
                        next_state = S_IDLE;
                        if((temp_data[39:32] + temp_data[31:24] +temp_data[23:16]+temp_data[15:8]) == temp_data[7:0])begin
                        humidity = temp_data[39:32];
                        temperature = temp_data[23:16];
                    end
                end
             end
                default:next_state = S_IDLE;
            endcase
        end
    end

    assign led[5:0] = state;
    assign dht11_data = dht11_buffer;
    
endmodule

module pwm_128step(
    input                       clk,
    input                       reset_p,
    input       [6 : 0]         duty,
    output                     reg pwm
    );
    
    parameter                   sys_clk_freg = 100_000_000;
    parameter                   pwm_freg = 100;
    parameter                   duty_step = 128;
    parameter                   temp = sys_clk_freg / (pwm_freg * duty_step);
    parameter                   temp_half = temp / 2;
    
    integer                    cnt;
    reg                         pwm_fregX128;
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            pwm_fregX128 = 0;
            cnt = 0;
        end
        else begin
            if(cnt >= (temp - 1)) cnt = 0;
            else cnt = cnt + 1;
            
            if(cnt < temp_half) pwm_fregX128 = 0;
            else pwm_fregX128 = 1;
        end
     end
    
    wire pwm_fregX128_nedge;
    edge_detector_n ed(
        .clk(clk), .reset_p(reset_p), .cp(pwm_fregX128),
        .n_edge(pwm_fregX128_nedge));
    reg        [6 : 0]          cnt_duty;
    always @(posedge clk or posedge reset_p)begin
        if(reset_p)begin
            cnt_duty = 0;
            pwm = 0;
        end
    else if(pwm_fregX128_nedge)begin
            cnt_duty = cnt_duty + 1;
            if(cnt_duty < duty)pwm = 1;
            else pwm = 0;
        end
    end
endmodule
