`timescale 1ns / 10ps
module my_pwm_test;

reg clk = 0;
always #5 clk = !clk;

reg pwm;

my_pwm mp(clk, 5, 0, pwm);

reg r_iter_out[2:0];

my_iterate_n #(.N(3), .CLK_ON(3), .CLK_DEAD(2)) iter_n(clk, r_iter_out);


initial begin
	$dumpfile("dump.vcd");
	$dumpvars(0, my_pwm_test);
	#300
	$finish;
end

endmodule

