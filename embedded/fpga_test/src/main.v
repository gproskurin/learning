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



reg [15:0] r_rdata;
reg [7:0] r_raddr;

reg [7:0] r_waddr;
reg [15:0] r_wdata;
reg [15:0] wmask = 0;


// memory
SB_RAM40_4K
	#(
		.INIT_0(256'H7127678656750404303022221111000020232022202120201013101210111010),
		.INIT_1(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f),
		.INIT_2(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f),
		.INIT_3(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f),
		.INIT_4(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f),
		.INIT_5(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f),
		.INIT_6(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f),
		.INIT_7(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f),
		.INIT_8(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f),
		.INIT_9(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f),
		.INIT_A(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f),
		.INIT_B(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f),
		.INIT_C(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f),
		.INIT_D(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f),
		.INIT_E(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f),
		.INIT_F(256'h000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f)
	)
	mem(
		.RDATA(r_rdata),
		.RADDR(r_raddr),
		.RCLK(clk),
		.RCLKE(1),
		.RE(1),
		.WADDR(r_waddr),
		.WCLK(0),
		.WCLKE(0),
		.WDATA(r_wdata),
		.WE(0),
		.MASK(0)
	);

localparam RAM_CNT = 100000000/2 - 1;
reg [31:0] clk_cnt;
always@(posedge clk)
begin
	if (clk_cnt == RAM_CNT) begin
		clk_cnt <= 0;
		r_raddr <= r_raddr + 1;
	end else begin
		clk_cnt <= clk_cnt + 1;
	end
end


// 7seg2
reg r_seg72_en = 1;
wire clk_s72;
my_clk_div #(.W($clog2(CONST_CLK/20000))) clks72(clk, CONST_CLK/20000, clk_s72);

reg r_s72_en;
my_pwm pwm_s72_en(clk, 5'd31, 5'd28, r_s72_en);
my_seg7_2 s720(
	clk_s72,
	r_s72_en,
	r_raddr,
	{seg72_a, seg72_b, seg72_c, seg72_d, seg72_e, seg72_f, seg72_g},
	{seg72_gnd0, seg72_gnd1}
);


// 7seg-4
my_seg7_n #(.N(4)) s74(
	clk,
	1'b1,
	r_rdata,
	{seg74_a, seg74_b, seg74_c, seg74_d, seg74_e, seg74_f, seg74_g},
	{seg74_gnd3, seg74_gnd2, seg74_gnd1, seg74_gnd0}
);


endmodule

