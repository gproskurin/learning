`default_nettype none

module ws2812_data #(parameter W_ADDR = 6, W_DATA = 24) (
	input wire clk,
	input wire start,
	output wire done,
	input wire [W_ADDR-1:0] iaddr,
	output wire [W_DATA-1:0] odata
);
reg start_old;
reg done_next;
reg [W_DATA-1:0] r_odata;
wire [2:0] row, col;
assign row = iaddr[W_ADDR-1:3];
assign col = iaddr[2:0];
wire [3:0] row1, col1;
wire [3:0] sum;
assign sum = row1 ^ col1;
assign row1 = {1'b0, row};
assign col1 = {1'b0, col};
reg prepared;
assign odata = r_odata;

localparam PERIOD = 30000000;
reg [$clog2(PERIOD)-1:0] r_cnt;

localparam C_WHITE = 24'h0F0F0F;
localparam C_BLACK = 24'h010101;
localparam C_RED = 24'h010F01;
localparam C_ORANGE = 24'h0F1F01;
localparam C_YELLOW = 24'h0F0F01;
localparam C_GREEN = 24'h0F0101;
localparam C_LIGHTBLUE = 24'h01010F;
localparam C_BLUE = 24'h01012F;
localparam C_MAGENTA = 24'h01803F;

reg [23:0] c1 = C_RED;
reg [23:0] c2 = C_ORANGE;
reg [23:0] c3 = C_YELLOW;
reg [23:0] c4 = C_GREEN;
reg [23:0] c5 = C_LIGHTBLUE;
reg [23:0] c6 = C_BLUE;
reg [23:0] c7 = C_MAGENTA;
reg [23:0] c8 = C_WHITE;
always@(posedge clk)
begin
	if (r_cnt == PERIOD-1) begin
		r_cnt <= 0;
		c1 <= c2;
		c2 <= c3;
		c3 <= c4;
		c4 <= c5;
		c5 <= c6;
		c6 <= c7;
		c7 <= c8;
		c8 <= c1;
	end else begin
		r_cnt <= r_cnt + 1;
	end
	done <= done_next;
	if (start && !start_old) begin
		if (!prepared) begin
			prepared <= 1;
		end
		case (sum[2:0])
			0: r_odata <= c8;
			1: r_odata <= c1;
			2: r_odata <= c2;
			3: r_odata <= c3;
			4: r_odata <= c4;
			5: r_odata <= c5;
			6: r_odata <= c6;
			7: r_odata <= c7;
		endcase
		done_next <= 1;
	end else begin
		done_next <= 0;
		prepared <= 0;
	end
	start_old <= start;
end

endmodule


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
	output wire seg72_gnd1,

	output wire [17:0] sram_a,
	inout wire [15:0] sram_d,
	output wire sram_cs,
	output wire sram_oe,
	output wire sram_we,

	output wire keypad_led_d1,
	output wire keypad_led_d2,
	output wire keypad_led_d3,
	output wire keypad_led_d4,
	output wire keypad_led_d5,
	output wire keypad_led_d6,
	output wire keypad_led_d7,
	output wire keypad_led_d8,

	input wire keypad6_1,
	input wire keypad6_2,
	input wire keypad6_3,
	input wire keypad6_4,
	input wire keypad6_5,
	input wire keypad6_6,

	output wire ws2812_ctrl
);

localparam CONST_CLK = 100000000;

// 128Hz clock
wire clk_128hz;
my_clk_div #(.DIV(CONST_CLK/128)) clk2(clk, clk_128hz);

reg [6:0] cnt_128hz;
always @ (posedge clk_128hz)
begin
	cnt_128hz <= cnt_128hz + 1;
end

// 1Hz clock
wire clk_1hz;
assign clk_1hz = ~cnt_128hz[6];

// high speed counter
reg [15:0] cnt_clk;
always@(posedge clk)
begin
	cnt_clk <= cnt_clk + 1;
end

// led2
//assign led2 = clk_1hz;

// led1
reg [3:0] pwm1_width = 0;
reg dec = 0;
//my_pwm pwm1(clk, 15, pwm1_width, led1);
always @ (posedge cnt_128hz[2])
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


reg [15:0] st_dbgout16;
reg [7:0] st_dbgout8;
wire sram_test_err;
reg [7:0] r_sram_test_err_cnt;
wire tmp1;
sram_test st(
	//.clk(cnt_clk[9]),
	//.clk(cnt_128hz[0]),
	//.clk(clk_1hz),
	.clk(clk),
	.sram_addr(sram_a),
	.sram_data(sram_d),
	.sram_cs(sram_cs),
	.sram_oe(sram_oe),
	.sram_we(sram_we),
	//.dbgout16(st_dbgout16),
	//.dbgout8(st_dbgout8),
	//.dbgout1(led2),
	.dbgout1(tmp1),
	.out_err(sram_test_err)
);
always@(posedge sram_test_err)
begin
	r_sram_test_err_cnt <= r_sram_test_err_cnt + 1;
end

// 7seg2
my_seg7_n #(.N(2)) s720(
	clk,
	1'b1,
	st_dbgout8,
	{seg72_a, seg72_b, seg72_c, seg72_d, seg72_e, seg72_f, seg72_g},
	{seg72_gnd1, seg72_gnd0}
);


// 7seg-4
my_seg7_n #(.N(4)) s74(
	clk,
	1'b1,
	st_dbgout16,
	{seg74_a, seg74_b, seg74_c, seg74_d, seg74_e, seg74_f, seg74_g},
	{seg74_gnd3, seg74_gnd2, seg74_gnd1, seg74_gnd0}
);


//my_keypad_led kpl0(r_sram_test_err_cnt[0], keypad_led_d8);
//my_keypad_led kpl1(r_sram_test_err_cnt[1], keypad_led_d7);
//my_keypad_led kpl2(r_sram_test_err_cnt[2], keypad_led_d6);
//my_keypad_led kpl3(r_sram_test_err_cnt[3], keypad_led_d5);
//my_keypad_led kpl4(r_sram_test_err_cnt[4], keypad_led_d4);
//my_keypad_led kpl5(r_sram_test_err_cnt[5], keypad_led_d3);
//my_keypad_led kpl6(r_sram_test_err_cnt[6], keypad_led_d2);
//my_keypad_led kpl7(r_sram_test_err_cnt[7], keypad_led_d1);

//my_keypad_led kp6_1(~keypad6_1, keypad_led_d1);

//always@(posedge cnt_128hz[5])
//begin
//	keypad_leds <= keypad_leds + 1;
//end

wire btn1 = ~keypad6_1;

wire btn1_debounced;
my_debounce #(.DELAY(100000000/12)) k6_1(.clk(clk), .in(btn1), .out(btn1_debounced));


//always@(posedge btn1) keypad_leds_right <= keypad_leds_right + 1;

//always@(posedge btn1_debounced) keypad_leds_left <= keypad_leds_left + 1;


// WS2812

wire [23:0] ws_data_color;
wire [5:0] ws_data_addr;
wire ws_data_start;
//wire ws_data_done;
ws2812_data ws_data(.clk(clk), .start(ws_data_start), /*.done(ws_data_done),*/ .iaddr(ws_data_addr), .odata(ws_data_color));

wire clk_ws;
//assign clk_ws = cnt_128hz[4];
//assign clk_ws = clk;
my_clk_div #(.DIV(10)) clkws(clk, clk_ws);
my_ws2812 #(.CLK_SCALE(10)) ws2812(
	.clk(clk_ws),
	//.clk_ws(clk),
	.ctrl(ws2812_ctrl),
	.leddata_addr(ws_data_addr),
	.leddata_start(ws_data_start),
	//.leddata_done(ws_data_done),
	.leddata_color(ws_data_color),
	.dbgout8(st_dbgout8),
	.dbgout16(st_dbgout16),
	.dbg_start(keypad_led_d4),
	.dbg_done(keypad_led_d5)
);
//my_ws2812 ws2812(.clk_ws(clk_1hz), .ctrl(led2), .dbg({keypad_led_d4,keypad_led_d5,keypad_led_d6,keypad_led_d7,keypad_led_d8}));
//assign led2 = ws2812_ctrl;

//assign keypad_led_d1 = ~ws_data_start;
//assign keypad_led_d2 = ~ws_data_done;
assign keypad_led_d8 = ~clk_ws;

assign led2 = ws2812_ctrl;

endmodule

