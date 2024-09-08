module my_pwm
	#(parameter W = 27) // up to 100mhz by default
(
	input wire pwm_clk,
	input [W-1:0] pwm_total,
	input [W-1:0] pwm_on,
	output wire out
);

	reg [31:0] cnt = 32'b0;
	reg out_r = (cnt < pwm_on) ? 1 : 0;
	assign out = out_r;

	always @ (posedge pwm_clk)
	begin
		cnt <= (cnt < pwm_total) ? (cnt + 1) : 0;
	end

endmodule
