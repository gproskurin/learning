`default_nettype none

module top(
	input wire clk,
	output wire led1,
	output wire led2,

	output wire seg74_a,
	output wire seg74_b,
	output wire seg74_c,
	output wire seg74_d,
	output wire seg74_e,
	output wire seg74_f,
	output wire seg74_g,
	output wire seg74_gnd0,
	output wire seg74_gnd1,
	output wire seg74_gnd2,
	output wire seg74_gnd3,

	output wire seg72_a,
	output wire seg72_b,
	output wire seg72_c,
	output wire seg72_d,
	output wire seg72_e,
	output wire seg72_f,
	output wire seg72_g,
	output wire seg72_dp,
	output wire seg72_gnd0,
	output wire seg72_gnd1
);

localparam CONST_CLK = 100000000;

// 128Hz clock
wire clk_128hz;
my_clk_div #(.W($clog2(CONST_CLK/128))) clk2(clk, CONST_CLK/128, clk_128hz);

reg [6:0] cnt_128hz;
always @ (posedge clk_128hz)
begin
	cnt_128hz <= cnt_128hz + 1;
end

// 1Hz clock
wire clk_1hz;
assign clk_1hz = ~cnt_128hz[6];

// led2
assign led2 = clk_1hz;

// led1
reg [3:0] pwm1_width = 0;
reg dec = 0;
my_pwm pwm1(clk, 15, pwm1_width, led1);
always @ (posedge cnt_128hz[5])
begin
	if (pwm1_width == 14) begin
		dec <= 1;
	end else if (pwm1_width == 1) begin
		dec <= 0;
	end
	pwm1_width <= dec ? pwm1_width - 1 : pwm1_width + 1;
end

// 7seg2
reg [7:0] r_seg72_num;
reg r_seg72_en = 1;
always@(posedge cnt_128hz[4])
begin
	r_seg72_num <= r_seg72_num + 1;
end

wire clk_s72;
my_clk_div #(.W($clog2(CONST_CLK/20000))) clks72(clk, CONST_CLK/20000, clk_s72);

reg r_s72_en;
my_pwm pwm_s72_en(clk, 5'd31, 5'd28, r_s72_en);
my_seg7_2 s720(
	clk_s72,
	r_s72_en,
	r_seg72_num,
	{seg72_a, seg72_b, seg72_c, seg72_d, seg72_e, seg72_f, seg72_g},
	{seg72_gnd0, seg72_gnd1}
);


// 7seg-4
reg [15:0] r_num74 = 0;
always@(posedge cnt_128hz[0])
begin
	r_num74 <= r_num74 + 1;
end

my_seg7_n #(.N(4)) s74(
	clk,
	1'b1,
	r_num74,
	{seg74_a, seg74_b, seg74_c, seg74_d, seg74_e, seg74_f, seg74_g},
	{seg74_gnd3, seg74_gnd2, seg74_gnd1, seg74_gnd0}
);


endmodule

