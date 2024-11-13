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
