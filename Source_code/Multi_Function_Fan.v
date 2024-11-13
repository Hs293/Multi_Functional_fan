module pan_top_project_detector_V2(
    input clk, reset_p,
    input [3:0] btn,
    inout dht11_data,
    input vauxn6, vauxp6,
    input [3:0] pulse_out,
    output [3:0] TTL,
    output [11:0] led,
    output motor_pwm,
    output servo_pwm,
    output [3:0] com,
    output [7:0] seg_7,
    output led_r,
    output led_g,
    output led_b);

    wire motor_stop_f;
    


    pan motor_pwm_ctnr(  .clk(clk), .reset_p(reset_p), .btn(btn[0]), .motor_stop_f(motor_stop_f),
                        .motor_stop( (motor_stop_t || motor_stop_r) || motor_off_s), .led(led[5:3]),  .motor_pwm(motor_pwm) );


    led_pro led_pwm_ctnr( .clk(clk), .reset_p(reset_p), .btn(btn[1]), .pwm_led(led[6]));


    timer_set_value cook_timer( .clk(clk), .reset_p(reset_p), .btn(btn[2]), .vauxn6(vauxn6), .vauxp6(vauxp6), .motor_stop(motor_stop_t), 
                                .motor_stop_f(motor_stop_f), .motor_off_s(motor_off_s), .motor_stop_r(motor_stop_r),  .led(led[2:0]), .com(com), .seg_7(seg_7));
                                                             

    HC_SR04_test_project motor_off_so( .clk(clk), .reset_p(reset_p), .pulse_out(pulse_out[3]), .TTL(TTL[3]), .motor_off(motor_off_s));
    
    servo_motor_detector_v2 ultra( .clk(clk), .reset_p(reset_p), .btn(btn[3]),.pulse_out(pulse_out[2:0]), .TTL(TTL[2:0]), .servo_pwm(servo_pwm), .led(led[11:7]) );
    
    led_dht11_pwm_top( .clk(clk), .reset_p(reset_p), .dht11_data(dht11_data), .led_r(led_r), .led_g(led_g), .led_b(led_b));
endmodule

module pan( 
    input clk, reset_p,
    input btn,
    input motor_stop,
    output reg motor_stop_f,
    output [2:0] led,
    output motor_pwm );
    
      
    parameter S_IDLE_MOTER           = 4'b0001;
    parameter S_MODE_1               = 4'b0010;
    parameter S_MODE_2               = 4'b0100;
    parameter S_MODE_3               = 4'b1000;
    
    wire  btn_mode_moter;
    button_cntr     btn_moter(.clk(clk), .reset_p(reset_p), .btn(btn), .btn_pedge(btn_mode_moter)); //moter 밝기 제어
    
                                                                                               // 3번 버튼 >> timer 시간 제어

    integer duty_moter;
    
    reg [3:0] state_moter, next_state_moter;  
    
    
    
    
    
    always @ (posedge clk, posedge reset_p) begin
        if(reset_p) begin
            state_moter = S_IDLE_MOTER;
        end
        else begin
            state_moter = next_state_moter;
        end
    end 
    
    assign led = state_moter[3:1];


    always @(posedge clk or posedge reset_p) begin  //moter state
        if (reset_p) begin
            next_state_moter <= S_IDLE_MOTER;
            duty_moter <= 0;

        end
        else begin
            case (state_moter)
                S_IDLE_MOTER: begin
                    if (btn_mode_moter) begin
                        next_state_moter <= S_MODE_1;
                        motor_stop_f <= 0;
                    end
                    else begin
                        next_state_moter <= S_IDLE_MOTER;
                        duty_moter <= 0;

                    end
                end
                S_MODE_1: begin
                    if (btn_mode_moter) begin
                        next_state_moter <= S_MODE_2;
                    end
                    else begin
                        if(motor_stop) begin
                            next_state_moter <= S_IDLE_MOTER;
                            motor_stop_f <= 1;
                        end
                        else begin
                            next_state_moter <= S_MODE_1;
                            duty_moter <= 38;
                        end
                    end
                end
                S_MODE_2: begin
                    if (btn_mode_moter) begin
                        next_state_moter <= S_MODE_3;
                    end
                    else begin
                        if(motor_stop) begin
                            next_state_moter <= S_IDLE_MOTER;
                            motor_stop_f <= 1;
                        end
                        else begin
                            next_state_moter <= S_MODE_2;
                            duty_moter <= 76;
                        end
                    end
                end
                S_MODE_3: begin
                    if (btn_mode_moter)begin
                        next_state_moter <= S_IDLE_MOTER;
                    end
                    else begin
                        if(motor_stop) begin
                            next_state_moter <= S_IDLE_MOTER;
                            motor_stop_f <= 1;
                        end
                        else begin
                            next_state_moter <= S_MODE_3;
                            duty_moter <= 115;
                        end
                    end
                end
                default : begin
                    next_state_moter = S_IDLE_MOTER;
                end
            endcase
        end
    end
 
    

    pwm_128step_freq #(.pwm_freq(100), .duty_steps(128)) pwm_motor_spd(.clk(clk), .reset_p(reset_p), .duty(duty_moter),.pwm(motor_pwm));
       
      
     
endmodule


module button_cntr(
    input btn, clk, reset_p,
    output btn_pedge, btn_nedge
);

wire  btn_clk;

reg [16:0] clk_div;

reg debounced_btn;

always @ (posedge clk) clk_div = clk_div  + 1; 

edge_detector_n  ed1 (.clk(clk), .reset_p(reset_p), .cp(clk_div[16]), .p_edge(btn_clk));


always @(posedge clk, posedge reset_p) begin
    if(reset_p) debounced_btn = 0;
    else if (btn_clk) debounced_btn = btn;
end



 edge_detector_n  ed0 (.clk(clk), .reset_p(reset_p), .cp(debounced_btn), .p_edge(btn_pedge), .n_edge(btn_nedge)); 
     
endmodule


module edge_detector_n (clk, reset_p, cp, p_edge, n_edge);
input clk, reset_p, cp;
output   p_edge, n_edge;

 reg ff_cur, ff_old;       


    always @ (negedge clk, posedge reset_p) begin
            if(reset_p) begin
                ff_cur <= 0;
                ff_old <= 0;
            end
            else begin
                ff_cur <= cp;                   
                ff_old <= ff_cur;
            end

    end

    assign p_edge = ({ff_cur, ff_old} == 2'b10) ? 1: 0;
    
    assign n_edge = ({ff_cur, ff_old} == 2'b01) ? 1: 0;


endmodule



module pwm_128step_freq   //pwm_freq =  50 - servo moter, 100 - motor, 10000 - led
    #(parameter sys_clk_freq = 100_000_000,
    parameter pwm_freq = 50, 
    parameter duty_steps = 256, 
    parameter pwm_clk = sys_clk_freq / pwm_freq /duty_steps,
    parameter pwm_clk_half = pwm_clk /2)
(
    input clk, reset_p,
    input [31:0] duty,
    output reg pwm);
    
    
    
    integer cnt;
    
    reg pwm_freqx128;
    
    always @ (posedge clk, posedge reset_p) begin
        if (reset_p) begin
            pwm_freqx128 = 0;
            cnt = 0;
        end
        else begin
            if(cnt >= (pwm_clk - 1)) begin
                cnt = 0;
            end
            else  begin
                cnt = cnt + 1 ;
            end
            if (cnt < pwm_clk_half)pwm_freqx128 = 0;
            else pwm_freqx128 = 1;
        end
    end
    
    wire pwm_freq_nedge;
   edge_detector_n  ed0 (.clk(clk), .reset_p(reset_p), .cp(pwm_freqx128), .n_edge(pwm_freq_nedge));  
   
    
    integer count_duty;
    
    
    always @ (posedge clk, posedge reset_p) begin
        if (reset_p) begin
            count_duty = 0;
            pwm = 0;
        end
        else if(pwm_freq_nedge) begin
            if (count_duty >=duty_steps-1) begin
                count_duty = 0;
            end
            else count_duty = count_duty+1;
            if(count_duty < duty) pwm =1;
            else pwm = 0;
        end
    end
    
    

endmodule


module led_pro(
    input clk, reset_p,
    input btn,
    output pwm_led);

    parameter S_IDLE_LED             = 4'b0001;   // off state
    parameter S_LED_1st              = 4'b0010;   
    parameter S_LED_2nd              = 4'b0100;   
    parameter S_LED_3rd              = 4'b1000;   

    wire btn_mode_led;

    button_cntr     btn_led(.clk(clk), .reset_p(reset_p), .btn(btn), .btn_pedge(btn_mode_led)); 

    integer duty_led;   //30% >>38.4 60% >>76.8 90% >> 115.2

    reg [3:0] state_led, next_state_led;

    always @ (posedge clk, posedge reset_p) begin
        if(reset_p) begin
            state_led = S_IDLE_LED;
        end
        else begin
            state_led = next_state_led;
        end
    end 

       
    always @ (posedge clk, posedge reset_p) begin   //led state
        if (reset_p) begin
            next_state_led = S_IDLE_LED;
            duty_led = 0;
        end
        else begin
            case(state_led)
                S_IDLE_LED : begin
                    if(btn_mode_led) begin
                        next_state_led = S_LED_1st;
                    end
                    else begin
                        next_state_led = S_IDLE_LED;
                        duty_led = 0;
                    end 
                end
                S_LED_1st : begin
                    if(btn_mode_led) begin
                        next_state_led = S_LED_2nd;
                    end
                    else begin
                        duty_led = 10;
                        next_state_led = S_LED_1st;
                    end 
                end
                S_LED_2nd: begin
                    if(btn_mode_led) begin
                        next_state_led = S_LED_3rd;               
                    end
                    else begin
                        duty_led = 60;
                        next_state_led = S_LED_2nd;
                    end                     
                end
                S_LED_3rd: begin
                    if(btn_mode_led) begin
                        next_state_led = S_IDLE_LED;                
                    end
                    else begin
                        duty_led = 110;
                        next_state_led = S_LED_3rd;
                    end                     
                end                
                default : begin
                    next_state_led = S_IDLE_LED;
                end
            endcase
        end
    end

    pwm_128step_freq #(.pwm_freq(10000)) led(.clk(clk), .reset_p(reset_p), .duty(duty_led) ,.pwm(pwm_led));

endmodule

  
    module timer_set_value (
        input clk, reset_p,
        input btn,
        input motor_stop_f, motor_off_s,
        input vauxn6, vauxp6,
        output reg motor_stop, motor_stop_r,
        output [2:0] led,
        output [3:0] com,
        output [7:0] seg_7);
        
        parameter STOP = 4'b0001;
        parameter TIME1 = 4'b0010;
        parameter TIME3 = 4'b0100;
        parameter TIME5 = 4'b1000;
        
        reg [3:0] state, state_next;
        
        always @ (negedge clk, posedge reset_p) begin
            if (reset_p) begin
                state = STOP;
            end
            else begin
                state = state_next;
            end
        end

        wire btn_pedge;
        
        reg [15:0] value;
        
        reg start;
        
        wire [15:0] cur_time, cur_time_r;
        
        reg stop_start;
        
        wire btn_long;
        
        reg btn_long_reg;
        
        btn_long(.clk(clk), .reset_p(reset_p), .btn(btn), .long_key(btn_long), .short_key(short_press));
        
        wire  mode;
        
       T_flip_flop_p t_start ( .clk(clk), .reset_p(reset_p), .t(btn_long), .q(mode));             

        
        

        always @ (posedge clk, posedge reset_p) begin
            if(reset_p) begin
                state_next = STOP;
                value = 0;
                motor_stop =0;
                stop_start = 0;
            end
            else begin
                case (state)
                    STOP : begin
                        if (!mode && short_press) begin
                            state_next = TIME1;
                            stop_start = 0;
                        end
                        else begin
                            value = 16'b0000_0001_0000_0000;
                            stop_start = 1;
                            if (motor_stop_f) begin
                                motor_stop = 0;
                            end
                        end
                    end
                    TIME1 : begin
                        value = 16'b0000_0011_0000_0000;
                        if (!mode && short_press) begin
                            state_next = TIME3;
                        end
                        else if (cur_time == 0) begin
                            state_next = STOP;
                            motor_stop = 1;
                        end
                        else if (mode) begin
                            state_next = STOP;
                        end
                        else if (motor_off_s) begin
                            state_next = STOP;
                        end
                    end
                    TIME3 : begin
                        value = 16'b0000_0101_0000_0000;
                        if (!mode && short_press) begin

                            state_next = TIME5;
                         end
                         else if (cur_time == 0) begin
                            state_next = STOP;
                            motor_stop = 1;
                        end
                        else if (mode) begin
                            state_next = STOP;
                        end
                        else if (motor_off_s) begin
                            state_next = STOP;
                        end
                    end
                    TIME5 : begin
                        value = 16'b0000_0000_0000_0001;
                        if(!mode && short_press) begin
                            state_next = STOP;
                        end
                        else if (cur_time == 0) begin                  
                            state_next = STOP;
                            motor_stop = 1;
                        end
                        else if (mode) begin
                            state_next = STOP;
                        end
                        else if (motor_off_s) begin
                            state_next = STOP;
                        end
                    end

                endcase

            end

        end
            
        assign led = state[3:1];
        
        wire w_us_clk, w_ms_clk, w_s_clk, w_m_clk;
        
        wire clk_stop;
        
        assign  clk_stop = stop_start ? 1 : reset_p;
        
        clock_div_100 i_us_clk( .clk(clk), .reset_p(clk_stop), .cp_div_100(w_us_clk));
    
        clock_div_1000 i_ms_clk(.clk(clk), .reset_p(clk_stop), .clk_source(w_us_clk), .cp_div_1000_nedge(w_ms_clk));
    
        clock_div_1000 i_s_clk (.clk(clk), .reset_p(clk_stop), .clk_source(w_ms_clk), .cp_div_1000_nedge(w_s_clk));
        
        clock_div_60 i_m_clk (.clk(clk), .reset_p(clk_stop), .clk_source(w_s_clk), .cp_div_60_nedge(w_m_clk) );
        
        wire [3:0] cur_sec10, cur_sec1, cur_min10, cur_min1;
        
        wire [3:0] cur_sec10_r, cur_sec1_r, cur_min10_r, cur_min1_r;
        
        wire [15:0] adc_value;

        adc_top_potentiometer value_test( .clk(clk), .reset_p(reset_p),
                                     .vauxn6(vauxn6), .vauxp6(vauxp6), .adc_value_bcd(adc_value));
        

         wire fnd_stop;
         
        assign fnd_stop = mode ? reset_p : 1;
        
        reg fnd_mode; 
        
        wire [15:0] fnd_v;
        

        
        always @ (posedge clk, posedge reset_p) begin
            if (reset_p) begin
                fnd_mode = 0;
                motor_stop_r = 0;
            end
            else if (motor_stop_f) begin
                 motor_stop_r = 0;
            end
            else if (mode && short_press) begin
                fnd_mode = ~fnd_mode;
            end
            else if (cur_time_r == 0 && fnd_mode) begin
                fnd_mode = 0;
                motor_stop_r = 1;
            end
            else if (!mode) begin
                fnd_mode = 0;
            end
            else if (motor_off_s) begin
                fnd_mode = 0;
            end
        end
        
        wire w_us_clk_r, w_ms_clk_r, w_s_clk_r, w_m_clk_r;
        
        wire dec_clk_r, dec_clk0_r, dec_clk, dec_clk0;
        
        wire stop_reset_p;
        
        assign stop_reset_p = mode ? reset_p : 1;
        
        clock_div_100 i_us_clk_r( .clk(clk), .reset_p(stop_reset_p), .cp_div_100(w_us_clk_r));
    
        clock_div_1000 i_ms_clk_r(.clk(clk), .reset_p(stop_reset_p), .clk_source(w_us_clk_r), .cp_div_1000_nedge(w_ms_clk_r));
    
        clock_div_1000 i_s_clk_r (.clk(clk), .reset_p(stop_reset_p), .clk_source(w_ms_clk_r), .cp_div_1000_nedge(w_s_clk_r));
        
        clock_div_60 i_m_clk_r (.clk(clk), .reset_p(stop_reset_p), .clk_source(w_s_clk_r), .cp_div_60_nedge(w_m_clk_r) );
        
        loadable_down_counter_bcd_60 set_time0(  .clk(clk), .reset_p(clk_stop), 
                                                                .clk_time(w_s_clk), .load_enable(short_press),
                                                                .load_bcd10(value[7:4]), .load_bcd1(value[3:0]),
                                                                .bcd10(cur_sec10), .bcd1(cur_sec1),  .dec_clk(dec_clk) );
    
        loadable_down_counter_bcd_60 set_time1(  .clk(clk), .reset_p(clk_stop), 
                                                                .clk_time(dec_clk), .load_enable( short_press),
                                                                .load_bcd10(value[15:12]), .load_bcd1(value[11:8]),
                                                                .bcd10(cur_min10), .bcd1(cur_min1),  .dec_clk(dec_clk0) );

        loadable_down_counter_bcd_60 set_time_r0(  .clk(clk), .reset_p(stop_reset_p), 
                                                                .clk_time(w_s_clk_r), .load_enable(short_press),
                                                                .load_bcd10(adc_value[7:4]), .load_bcd1(adc_value[3:0]),
                                                                .bcd10(cur_sec10_r), .bcd1(cur_sec1_r),  .dec_clk(dec_clk_r) );
    
        loadable_down_counter_bcd_60 set_time_r1(  .clk(clk), .reset_p(stop_reset_p), 
                                                                .clk_time(dec_clk_r), .load_enable(short_press),
                                                                .load_bcd10(adc_value[15:12]), .load_bcd1(adc_value[11:8]),
                                                                .bcd10(cur_min10_r), .bcd1(cur_min1_r),  .dec_clk(dec_clk0_r) );

        
        assign cur_time = {cur_min10,cur_min1,cur_sec10,cur_sec1};
        
        assign cur_time_r = {cur_min10_r,cur_min1_r,cur_sec10_r,cur_sec1_r};
    
      assign fnd_v = fnd_mode ? cur_time_r : mode ? adc_value : 0;
      
      fnd_4digit_cntr fnd_on ( .clk(clk), .reset_p(fnd_stop), .com(com) , .value(fnd_v), .seg_7(seg_7) );
    
    
    endmodule

    
module btn_long(
    input clk, reset_p,
    input btn,
    output  long_key,
    output short_key);
    
    parameter LONG_PRESS_COUNT = 26'd50000000;
    
    reg [26:0] count;
    reg long_key_reg;
    
    wire w_short_key;
    
    button_cntr_long long( .btn(btn), .clk(clk), .reset_p(reset_p), .btn_long(btn_pedge));
    
    edge_detector_n  ed0 (.clk(clk), .reset_p(reset_p), .cp(btn), .n_edge(w_short_key));  
   
    edge_detector_n  ed1 (.clk(clk), .reset_p(reset_p), .cp(long_key_reg), .p_edge(long_key));  
   
    
    always @ (posedge clk, posedge reset_p) begin
        if(reset_p) begin
            long_key_reg = 0;
        end
        else begin
            if (btn_pedge) begin
                count = count +1;
                if (count >= LONG_PRESS_COUNT) begin
                    long_key_reg = 1;
                    count = 0;
                end
            end
            else begin
                long_key_reg = 0;
                count = 0;
            end
        end
    end

    assign short_key = ~long_key_reg & w_short_key;

endmodule

   
module button_cntr_long(
    input btn, clk, reset_p,
    output btn_long
);

wire  btn_clk;

reg [16:0] clk_div;

reg debounced_btn;

always @ (posedge clk) clk_div = clk_div  + 1; 

edge_detector_n  ed1 (.clk(clk), .reset_p(reset_p), .cp(clk_div[16]), .p_edge(btn_clk));


always @(posedge clk, posedge reset_p) begin
    if(reset_p) debounced_btn = 0;
    else if (btn_clk) debounced_btn = btn;
end

assign btn_long = debounced_btn;
  
endmodule


module T_flip_flop_p(                                 
    input clk, reset_p,                               
    input t,                                          
    output reg q);                                    
    
    reg clk_en;
    
    always @(posedge clk, posedge  reset_p)begin      
        if(reset_p)q <= 0;                            
        else if(clk_en) q <= ~q;                                                                                                    
    end 
    
    always @(*) begin
        clk_en = t;
    end                                              
   
endmodule    


module clock_div_100(
    input clk, reset_p,
    output clk_div_100,
    output cp_div_100
    );

    reg [6:0] count_sysclk;
    
    
    always @ (negedge clk, posedge reset_p) begin
        if (reset_p) count_sysclk = 0;
        else begin
            if (count_sysclk >= 99) begin 
                count_sysclk = 0;
                end
            else begin
                count_sysclk = count_sysclk +1;
            end
        end
    end
    
    assign cp_div_100 = (count_sysclk < 50) ? 0 : 1;
    
 edge_detector_n  edge_usec (.clk(clk), .reset_p(reset_p), .cp(cp_div_100), .n_edge(clk_div_100));
    

endmodule

module clock_div_1000(
    input clk, reset_p,
    input clk_source,
    output cp_div_1000_nedge
    );
    
    wire nedge_source;

 edge_detector_n  edge_usec0 (.clk(clk), .reset_p(reset_p), .cp(clk_source), .n_edge(nedge_source));


    reg [9:0] count_clk_source;
    
    
    always @ (negedge clk, posedge reset_p) begin
        if (reset_p) count_clk_source = 0;
        else if (nedge_source)begin
            if (count_clk_source >= 999) begin 
                count_clk_source = 0;
                end
            else begin
                count_clk_source = count_clk_source +1;
            end
        end
    end
    
    wire cp_div_1000;
    
    assign cp_div_1000 = (count_clk_source < 500) ? 0 : 1;
    
 edge_detector_n  edge_usec1 (.clk(clk), .reset_p(reset_p), .cp(cp_div_1000), .n_edge(cp_div_1000_nedge));
 
endmodule


module clock_div_60(
    input clk, reset_p,
    input clk_source,
    output cp_div_60_nedge
    );
    
    wire nedge_source, cp_div_60;

 edge_detector_n  edge_usec0 (.clk(clk), .reset_p(reset_p), .cp(clk_source), .n_edge(nedge_source));


    reg [5:0] count_clk_source;
    
    
    always @ (negedge clk, posedge reset_p) begin
        if (reset_p) count_clk_source = 0;
        else if (nedge_source)begin
            if (count_clk_source >= 59) begin 
                count_clk_source = 0;
                end
            else begin
                count_clk_source = count_clk_source +1;
            end
        end
    end
    
    assign cp_div_60 = (count_clk_source < 59) ? 0 : 1;
    
    edge_detector_n  edge_usec1 (.clk(clk), .reset_p(reset_p), .cp(cp_div_60), .n_edge(cp_div_60_nedge));

endmodule


module adc_top_potentiometer(
    input clk, reset_p,
    input vauxn6, vauxp6,
    output [15:0] adc_value_bcd); 

    wire [11:0] adc_value_output;

    adc_potentiometer_avr_top test( clk, reset_p,
                                vauxn6, vauxp6,
                                adc_value_output);

    reg [11:0] hundred_value;
    
    always @(posedge clk or posedge reset_p) begin
        if (reset_p) begin
            hundred_value <= 12'd0;
        end 
        else if (adc_value_output >= 59 && adc_value_output < 100) begin
                hundred_value <= 9'd100;
        end 
        else if (adc_value_output >= 159 && adc_value_output < 200) begin
                hundred_value <= 9'd200;
        end 
        else if (adc_value_output >= 12'd259 && adc_value_output < 12'd300) begin
                hundred_value <= 9'd300;
        end
        else if (adc_value_output >= 12'd359 && adc_value_output < 12'd400) begin
                hundred_value <= 9'd400;
        end 
        else if (adc_value_output >= 12'd459) begin
                hundred_value <= 12'd500;
        end
        else hundred_value <= adc_value_output;
        end

    reg [11:0] adc_value;
    
    always @ (posedge clk, posedge reset_p) begin
        if(reset_p) begin
            adc_value <= 0;
        end
        else begin
            adc_value <= hundred_value;
        end
    end
    
 
    
    
    bin_to_dec(
            .bin(adc_value),
            .bcd(adc_value_bcd)  );
   
    
    
    
    
endmodule


module bin_to_dec(
        input [11:0] bin,
        output reg [15:0] bcd
    );

    reg [3:0] i;
   
    always @(bin) begin
        bcd = 0;
        for (i=0;i<12;i=i+1)begin
            bcd = {bcd[14:0], bin[11-i]};
            if(i < 11 && bcd[3:0] > 4) bcd[3:0] = bcd[3:0] + 3;
            if(i < 11 && bcd[7:4] > 4) bcd[7:4] = bcd[7:4] + 3;
            if(i < 11 && bcd[11:8] > 4) bcd[11:8] = bcd[11:8] + 3;
            if(i < 11 && bcd[15:12] > 4) bcd[15:12] = bcd[15:12] + 3;
        end
    end
endmodule


    module adc_potentiometer_avr_top(
    input clk, reset_p,
    input vauxn6, vauxp6,
    output [11:0] adc_avr_12
    );

    wire [15:0] do_out;
    wire eoc_out;
    wire [4:0] channel_out;
    xadc_wiz_0   adc_ch6(
          .daddr_in({2'b0,channel_out}),            // Address bus for the dynamic reconfiguration port
          .dclk_in(clk),                            // Clock input for the dynamic reconfiguration port
          .den_in(eoc_out),                         // Enable Signal for the dynamic reconfiguration port
          .reset_in(reset_p),                       // Reset signal for the System Monitor control logic
          .vauxp6(vauxp6),                          // Auxiliary channel 6
          .vauxn6(vauxn6),
          .channel_out(channel_out),                // Channel Selection Outputs
          .do_out(do_out),                          // Output data bus for dynamic reconfiguration port
          .eoc_out(eoc_out)                         // End of Conversion Signal
          );
    
    wire eoc_out_pedge, eoc_out_nedge;
    edge_detector_n  ed1 (.clk(clk), .reset_p(reset_p), .cp(eoc_out), .p_edge(eoc_out_pedge), .n_edge(eoc_out_nedge));
    
    reg [11:0] adc_value;
    reg [15:0] avr_adc_value;
    reg [4:0] count;
    reg [15:0] adc_avr;
    
    always @ (posedge clk, posedge reset_p) begin
        if(reset_p) begin
            adc_value <= 0;
            count <= 0;
            adc_avr <= 0;
            avr_adc_value <= 0;
        end
        else if (count >= 16) begin
                adc_avr <= avr_adc_value;
                count <= 0;
                avr_adc_value <= 0;
        end
        else if (eoc_out_pedge) begin
            adc_value <= do_out[15:4];
            avr_adc_value <= avr_adc_value + adc_value;
            count <= count +1;
            
        end      
    end


assign adc_avr_12 = adc_avr[15:4];

endmodule 


module loadable_down_counter_bcd_60(
    input clk, reset_p,
    input   clk_time,
    input load_enable,
    input [3:0] load_bcd10, load_bcd1,
    output  reg [3:0] bcd10, bcd1,
    output reg dec_clk );
    
    wire counter_clk_n_edge;


    always @ (posedge clk, posedge reset_p) begin
        
        if(reset_p) begin 
            bcd10 = 0;
            bcd1 = 1;
            dec_clk = 0;
        end
        else  begin
                if(load_enable) begin
                    bcd10 = load_bcd10 ;
                    bcd1 =  load_bcd1;
                end
                else if (clk_time) begin
                    if (bcd1 == 0) begin
                        bcd1 = 9;
                        if(bcd10 == 0) begin
                                dec_clk = 1;
                                 bcd10 = 5;
                                
                         end
                        else bcd10 = bcd10 -1;
                    end
                   else bcd1 = bcd1 - 1;
              end
              else dec_clk = 0;
         end
    end


endmodule


module fnd_4digit_cntr( clk, reset_p, com , value, seg_7 );
input clk, reset_p ;
input [15:0] value;
output [3:0] com;
output [7:0] seg_7;

reg [3:0] hex_value;

    ring_counter_bcd com_0 (.clk(clk), .reset_p(reset_p), . q(com));    
    
    decoder_7seg_behavioral  seg0(  .hex_value(hex_value),  .seg_7(seg_7));    

    always @ (posedge clk) begin                           
               case(com)
                        4'b1110: hex_value = value[3:0];
                        4'b1101: hex_value = value[7:4];
                        4'b1011: hex_value = value[11:8];
                        4'b0111: hex_value = value[15:12];
               endcase
    end


endmodule



module ring_counter_bcd (
    clk, reset_p,  q
    );
input clk, reset_p;
output  reg [3:0] q;

    reg [16:0] clk_div;
    always @(posedge clk) clk_div = clk_div +1;                         // 이런식으로 분주를하네  클럭을 이용하여 카운터를 생성하니까 그 카운터의 2번쨰 비트는 4배 분주가된다 
   wire  clk_div_16_p;
   
    edge_detector_n de_clk (.clk(clk), .reset_p(reset_p), .cp(clk_div[16]), .p_edge(clk_div_16_p));

always @(posedge clk , posedge reset_p)begin                        // always  문에 clk아닌 다른 것이 들어가면 안된다 따라서 에지 디텍터를 이용 
    if(reset_p) q = 4'b1110;                                        // 리셋 을 주지않아도 FPGA에서 리셋된 값이 들어가는것 파워리셋이 존제하여 파워가 들어올때 리셋을 진행함 이는  클럭을 가지고 리셋을 1로 변경하는 것이아닌 자체 리셋이다. (플립 플롭의 프리셋과 리셋 을 이용함) 
    else if (clk_div_16_p)  
               if (q == 4'b0111) q = 4'b1110;
               else q = {q[2:0], 1'b1};
    end 
    
endmodule


module decoder_7seg_behavioral(                               
    input [3:0] hex_value,                        
    output reg [7:0] seg_7
    );                     

    always @(hex_value) begin                    
        case(hex_value)                           
                               //abcd_efgp
            4'b0000 : seg_7 = 8'b0000_0011;   
            4'b0001 : seg_7 = 8'b1001_1111;   
            4'b0010 : seg_7 = 8'b0010_0101;   
            4'b0011 : seg_7 = 8'b0000_1101;   
            4'b0100 : seg_7 = 8'b1001_1001;   
            4'b0101 : seg_7 = 8'b0100_1001;   
            4'b0110 : seg_7 = 8'b0100_0001;   
            4'b0111 : seg_7 = 8'b0001_1111;   
            4'b1000 : seg_7 = 8'b0000_0001;   
            4'b1001 : seg_7 = 8'b0001_1001;   
            4'b1010 : seg_7 = 8'b0001_0001;   
            4'b1011 : seg_7 = 8'b1100_0001;   
            4'b1100 : seg_7 = 8'b0110_0011;   
            4'b1101 : seg_7 = 8'b1000_0101;   
            4'b1110 : seg_7 = 8'b0110_0001;   
            4'b1111 : seg_7 = 8'b0111_0001;   
        endcase                                                       
    end

endmodule


module HC_SR04_test_project(
    input clk, reset_p,
    input pulse_out,
    output TTL,
    output reg motor_off);

    wire [11:0] distance;

    HC_SR04  dd( .clk(clk), .reset_p(reset_p), .pulse_out(pulse_out), .TTL(TTL), .distance(distance));

    always @ (posedge clk, posedge reset_p) begin
        if(reset_p) begin
            motor_off = 0;
        end
        else if (distance <= 3) begin
            motor_off = 1;
        end
        else motor_off = 0;
    end

endmodule


module HC_SR04(
    input clk, reset_p,
    input pulse_out,
    output reg TTL,
    output reg [11:0] distance,
    output [3:0] d);
    

    
    
    parameter S_IDLE = 3'b001;
    parameter S_TRIGGER = 3'b010;
    parameter S_ECHO = 3'b100;
    
    wire pulse_pedge, pulse_nedge, cm_count_nedge;
    
    edge_detector_n  clk_source (.clk(clk), .reset_p(reset_p), .cp(pulse_out), .p_edge(pulse_pedge),.n_edge(pulse_nedge));

    
    reg [19:0] count_sec;
    
    reg [2:0] state, state_next;
    assign d = state;
    reg count_sec_e;
    
    clock_div_100 us_clk (.clk(clk),  .reset_p(reset_p),  .clk_div_100(clk_us) );
    
    always @ (negedge clk, posedge reset_p ) begin
        if (reset_p) begin
            count_sec = 0;
        end
        else if (clk_us && count_sec_e) begin
            count_sec = count_sec +1;
         end
         else if (count_sec_e == 0) count_sec = 0;
    end 

    always @ (negedge clk, posedge reset_p) begin
        if (reset_p) begin
            state = S_IDLE;
        end
         else begin
            state = state_next;
         end
    end
    
    reg clk_count_start;
    

    
    reg clk_e;
    
    reg [8:0] cm_count;
    
    clock_div_58 div_58( .clk(clk), .reset_p(reset_p), .clk_source(clk_us), .clk_e(clk_e), .cp_div_58_nedge(cm_count_nedge)  );
       
    always @ (negedge clk, posedge reset_p ) begin
        if (reset_p) begin
            cm_count = 0;
        end
        else if (cm_count_nedge) begin
            cm_count = cm_count +1;
         end
         else if (clk_e == 0) cm_count = 0;
    end 
     
        
    always @ (posedge clk, posedge reset_p) begin
        if (reset_p) begin
            state_next = S_IDLE;
            TTL = 0;
            count_sec_e = 0;
            clk_e = 0;
            clk_count_start = 0;
        end
        else begin
            case(state)
                S_IDLE : begin
                        if (count_sec > 20'd600000) begin
                                state_next = S_TRIGGER;
                                count_sec_e = 0;
                        end
                        else  begin
                            count_sec_e = 1;
                        end
                end
                S_TRIGGER : begin
                    count_sec_e = 1;
                    TTL = 1;
                    if (count_sec > 15'd11) begin
                        TTL = 0;
                        state_next = S_ECHO;
                        count_sec_e = 0;
                        
                    end
                end
                S_ECHO : begin 
                            if (pulse_pedge) begin
                                count_sec_e = 1;
                                clk_e = 1;
                                clk_count_start = 1;
                            end
                            else if (pulse_nedge) begin
                                distance = cm_count;
                                count_sec_e = 0;
                                clk_e = 0;
                                clk_count_start = 0;
                                state_next = S_IDLE; 
                            end
                        end
                default : state_next = S_IDLE;
            endcase
        end
    end
    
    

endmodule


module clock_div_58(
    input clk, reset_p,
    input clk_source,
    input clk_e,
    output cp_div_58_nedge
    );
    
    reg [9:0] count_clk_source;
    
    
    always @ (negedge clk, posedge reset_p) begin
        if (reset_p) count_clk_source = 0;
        else if (clk_source && clk_e)begin
            if (count_clk_source >= 58) begin 
                count_clk_source = 0;
                end
            else begin
                count_clk_source = count_clk_source +1;
            end
        end
    end
    
    wire cp_div_58;
    
    assign cp_div_58 = (count_clk_source < 29) ? 0 : 1;
    
 edge_detector_n  edge_usec1 (.clk(clk), .reset_p(reset_p), .cp(cp_div_58), .n_edge(cp_div_58_nedge));
 
endmodule

 
 module servo_motor_detector_v2(
    input clk, reset_p,
    input [3:0] pulse_out,
    input btn,
    output [3:0] TTL,
    output [3:0] com,
    output [7:0] seg_7,
    output servo_pwm,
    output [4:0] led
 );
 
    wire [7:0] cnt;
    
    wire down_up, stop;
    wire mode;
    
    
    assign led[0] = mode;
    
    wire long_key, start_stop;
    
    assign start_stop = mode?  reset_p : 1; 
    
    btn_long btn_long0( .clk(clk), .reset_p(reset_p), .btn(btn),  .long_key(long_key));
    
    T_flip_flop_p t_start ( .clk(clk), .reset_p(reset_p), .t(long_key), .q(mode));             

    
    ultrasonic_wave_detector_test0_v2 detector0(.clk(clk), .reset_p(start_stop),  .cnt(cnt), 
                                             .down_up(down_up), .stop(stop),  .pulse_out(pulse_out), .TTL(TTL), .led(led[4:1]));
 
    top_servo_project_v2  detector1( .clk(clk), .reset_p(start_stop), .down_up(down_up), .stop(stop),  .cnt(cnt), .com(com), .seg_7(seg_7), .servo_pwm(servo_pwm));
 
 endmodule

 
    module ultrasonic_wave_detector_test0_v2(
     input clk, reset_p,
     input [2:0] pulse_out,
     input [7:0] cnt,
     output reg down_up, stop, 
     output [2:0] TTL,
     output [3:0] led);
     
    parameter S_IDLE =  4'b0001;
    parameter S_LEFT =  4'b0010;
    parameter S_MID  =  4'b0100;
    parameter S_RIGHT = 4'b1000; 
    
    reg [3:0] state, state_next;
    assign led = state;
    wire [11:0] distance_left_r, distance_mid_r, distance_right_r;
  
    HC_SR04  detector0_left( .clk(clk), .reset_p(reset_p), .pulse_out(pulse_out[0]), .TTL(TTL[0]), .distance(distance_left_r));
    HC_SR04  detector1_mid( .clk(clk), .reset_p(reset_p), .pulse_out(pulse_out[1]), .TTL(TTL[1]), .distance(distance_mid_r));
    HC_SR04  detector2_right( .clk(clk), .reset_p(reset_p), .pulse_out(pulse_out[2]), .TTL(TTL[2]), .distance(distance_right_r));
  
 
     
    
                            
    always @ (posedge clk, posedge reset_p) begin
        if (reset_p) begin
            state = S_IDLE;
        end
        else begin
            state = state_next;
        end
    end
    
    reg distance_left,distance_mid, distance_right ;
    
    reg [11:0] distance_value_high, distance_value_low, distance_value_middle ;
    


always @ (posedge clk or posedge reset_p) begin
    if (reset_p) begin
        distance_value_low = 12'd0;
        distance_left = 0;
        distance_mid = 0;
        distance_right = 0;
    end
    else begin
        if ((distance_left_r <= distance_mid_r) && (distance_left_r <= distance_right_r)) begin
            distance_value_low = distance_left_r;
            if (distance_value_low <= 50) begin
                distance_left = 1;
                distance_mid = 0;
                distance_right = 0;
            end
            else begin
                distance_left = 0;
                distance_mid = 0;
                distance_right = 0;
            end
        end
        else if ((distance_mid_r <= distance_left_r) && (distance_mid_r <= distance_right_r)) begin
            distance_value_low = distance_mid_r;
            if (distance_value_low <= 50) begin
                distance_left = 0;
                distance_mid = 1;
                distance_right = 0;
            end
            else begin
                distance_left = 0;
                distance_mid = 0;
                distance_right = 0;
            end
        end
        else if ((distance_right_r <= distance_left_r) && (distance_right_r <= distance_mid_r))begin
            distance_value_low = distance_right_r;
            if (distance_value_low <= 50) begin
                distance_left = 0;
                distance_mid = 0;
                distance_right = 1;
            end
            else begin
                distance_left = 0;
                distance_mid = 0;
                distance_right = 0;
            end
        end
    end
end
    
    
    always @ (posedge clk, posedge reset_p) begin
        if (reset_p) begin
            state_next = S_IDLE;
            
        end
        else begin
            case (state)
                S_IDLE : begin
                    if (distance_left) begin
                        state_next = S_LEFT;
                    end
                    else if (distance_mid) begin
                        state_next = S_MID;
                    end
                    else if (distance_right) begin
                        state_next = S_RIGHT;
                    end
                end
                S_LEFT : begin
                    down_up = 1;
                    stop = 0;
                    if (cnt <=24) begin
                        stop = 1;
                        if (!distance_right) begin
                            stop = 0;
                            state_next = S_IDLE;
                        end
                    end
                     if (!distance_left) begin
                        stop = 0;
                        state_next = S_IDLE;
                    end
                end
                S_MID : begin
                    if (distance_mid) begin
                        stop = 1;
                    end
                    else begin
                        stop = 0;
                        state_next = S_IDLE;
                    end
                end
                S_RIGHT : begin
                    down_up = 0;
                    stop = 0;
                    if (cnt >= 128) begin
                        stop = 1;
                        if (!distance_right) begin
                            stop = 0;
                            state_next = S_IDLE;
                        end
                    end
                    else if (!distance_right) begin
                        stop = 0;
                        state_next = S_IDLE;
                    end
                end
            endcase
        end
    end
    
    
    
                                     
  
  endmodule 

  
  module top_servo_project_v2(
    input clk, reset_p,
    input down_up, stop,
    output reg [7:0] cnt,
    output [3:0] com,
    output [7:0] seg_7,
    output servo_pwm);
    
    integer clk_div;
    
    always @ (posedge clk) clk_div = clk_div+1;
    
    
    wire clk_div_nedge_20;
    
    edge_detector_n  ed1 (.clk(clk), .reset_p(reset_p), .cp(clk_div[20]), .p_edge(clk_div_nedge_20));
    
    
    
    
    
    always @ (posedge clk, posedge reset_p ) begin
        if (reset_p) begin
           cnt = 50;
        end
        else if (clk_div_nedge_20) begin
            if (down_up)begin
                if (stop) begin
                    cnt = cnt;
                end
                else if (cnt <= 24) begin
                    cnt = 24;
                end
                else cnt = cnt -1;
            end
            else if (!down_up) begin
                if (stop) begin
                    cnt = cnt;
                end
                else if (cnt>= 128) begin
                    cnt = 128;
                end
                else cnt = cnt +1;
            end
        end
    end
    
    wire long;
    
     
    wire [15:0] duty_bcd;
    bin_to_dec(
            .bin(cnt ),
            .bcd(duty_bcd)  );
    
   
    fnd_4digit_cntr fnd_on ( .clk(clk), .reset_p(reset_p), .com(com) , .value(duty_bcd), .seg_7(seg_7) );

    pwm_128step_freq  #(.pwm_freq(50), .duty_steps(1024)) pwm_motor(.clk(clk), .reset_p(reset_p),.duty(cnt),.pwm(servo_pwm));


endmodule


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
