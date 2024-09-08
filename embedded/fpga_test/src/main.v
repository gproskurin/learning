module top(input wire clk, output wire led1, output wire led2);

// led1
my_pwm #(.W(27)) pwm_led1(clk, 27'd100000000/13, 27'd10000000/17, led1);


// led2
reg [7:0] clk_div_256 = 0;
wire clk_390khz = ~clk_div_256[7]; // 100mhz / 256

always @ (posedge clk)
begin
	// counter for led2
	clk_div_256 <= clk_div_256 + 1;
end

my_pwm #(.W(19)) pwm_led2(clk_390khz, 100000000/256, 20000, led2);

endmodule

