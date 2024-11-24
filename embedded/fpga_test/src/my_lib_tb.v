`timescale 1ns / 10ps
module my_pwm_test;

reg clk = 0;
always #5 clk = !clk;

reg pwm;

my_pwm mp(clk, 5, 0, pwm);

wire clk_4;
my_clk_div #(.DIV(4)) clkdiv(clk, clk_4);

reg r_iter_out[2:0];

initial begin
	$dumpfile("dump.vcd");
	$dumpvars(0, my_pwm_test);
	#300
	$finish;
end

endmodule

