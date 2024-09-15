module top(
	input wire clk,
	output wire led1,
	output wire led2,

	output wire seg7_a,
	output wire seg7_b,
	output wire seg7_c,
	output wire seg7_d,
	output wire seg7_e,
	output wire seg7_f,
	output wire seg7_g,

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
always @ (posedge cnt_128hz[4])
begin
	if (pwm1_width == 14) begin
		dec <= 1;
	end else if (pwm1_width == 1) begin
		dec <= 0;
	end
	pwm1_width <= dec ? pwm1_width - 1 : pwm1_width + 1;
end

// 7seg-1
reg r_seg7en;
wire seg7en = r_seg7en;
my_seg7_1 my7_1(
	clk,
	r_seg7en,
	pwm1_width,
	{seg7_a, seg7_b, seg7_c, seg7_d, seg7_e, seg7_f, seg7_g}
);

my_pwm pwm7seg1(clk, 15, 3, r_seg7en);

// 7seg2
reg [7:0] r_seg72_num;
reg r_seg72_en = 1;
always@(posedge cnt_128hz[4])
begin
	r_seg72_num <= r_seg72_num + 1;
end

wire clk_s72;
my_clk_div #(.W($clog2(CONST_CLK/10000))) clks72(clk, CONST_CLK/10000, clk_s72);

reg r_s72_en;
my_pwm pwm_s72_en(clk, 31, 28, r_s72_en);
my_seg7_2 s720(
	clk_s72,
	r_s72_en,
	r_seg72_num,
	{seg72_a, seg72_b, seg72_c, seg72_d, seg72_e, seg72_f, seg72_g},
	{seg72_gnd0, seg72_gnd1}
);


endmodule

