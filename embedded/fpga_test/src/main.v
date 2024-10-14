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
	output wire seg72_gnd1,

	output wire sram_a0,
	output wire sram_a1,
	output wire sram_a2,
	output wire sram_a3,
	output wire sram_a4,
	output wire sram_a5,
	output wire sram_a6,
	output wire sram_a7,
	output wire sram_a8,
	output wire sram_a9,
	output wire sram_a10,
	output wire sram_a11,
	output wire sram_a12,
	output wire sram_a13,
	output wire sram_a14,
	output wire sram_a15,
	output wire sram_a16,
	output wire sram_a17,
	inout wire sram_d0,
	inout wire sram_d1,
	inout wire sram_d2,
	inout wire sram_d3,
	inout wire sram_d4,
	inout wire sram_d5,
	inout wire sram_d6,
	inout wire sram_d7,
	inout wire sram_d8,
	inout wire sram_d9,
	inout wire sram_d10,
	inout wire sram_d11,
	inout wire sram_d12,
	inout wire sram_d13,
	inout wire sram_d14,
	inout wire sram_d15,
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
	input wire keypad6_6
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


// SRAM
reg r_sram_oe;
assign sram_oe = r_sram_oe;

reg r_sram_we;
assign sram_we = r_sram_we;

reg r_sram_cs;
assign sram_cs = r_sram_cs;

wire [15:0] sram_data = {
	sram_d15, sram_d14, sram_d13, sram_d12,
	sram_d11, sram_d10, sram_d9, sram_d8,
	sram_d7, sram_d6, sram_d5, sram_d4,
	sram_d3, sram_d2, sram_d1, sram_d0
};
reg [15:0] r_sram_data_out;
assign sram_data = r_sram_oe ? r_sram_data_out : 16'bZ;

wire [17:0] sram_addr = {
	sram_a17, sram_a16,
	sram_a15, sram_a14,
	sram_a13, sram_a12,
	sram_a11, sram_a10,
	sram_a9, sram_a8,
	sram_a7, sram_a6,
	sram_a5, sram_a4,
	sram_a3, sram_a2,
	sram_a1, sram_a0
};
reg [17:0] r_sram_addr;
assign sram_addr = r_sram_addr;


reg r_write_done = 0;
localparam addr1 = 18'b000101010001011101;
localparam addr2 = 18'b111010110010110100;

reg [31:0] r_cnt_sram;
localparam CNT_READ1 = 100000000*3;
localparam CNT_READ2 = 100000000*6;
reg [15:0] r_sram_read_data;
always@(posedge clk)
begin
	if (r_write_done == 0) begin
		case (r_cnt_sram)
			0: begin
				r_sram_cs <= 1;
				r_sram_oe <= 1;
				r_sram_we <= 1;
			end
			1: r_sram_cs <= 0;

			// write1
			2: begin
				r_sram_addr <= addr1;
				r_sram_data_out <= 16'Hdead;
			end
			3: r_sram_we <= 0;
			4: r_sram_we <= 1;

			// write2
			5: begin
				r_sram_addr <= addr2;
				r_sram_data_out <= 16'Hbeef;
			end
			6: r_sram_we <= 0;
			7: r_sram_we <= 1;

			100: begin
				r_sram_addr <= 0;
				r_sram_data_out <= 0;
				r_sram_read_data <= 0;
				r_write_done <= 1;
			end
		endcase
	end else begin
		case (r_cnt_sram)
			CNT_READ1: begin
				r_sram_cs <= 1;
				r_sram_oe <= 1;
				r_sram_we <= 1;
			end
			CNT_READ1+1: r_sram_cs <= 0;

			// read1
			CNT_READ1+2: r_sram_addr <= addr1;
			CNT_READ1+3: r_sram_oe <= 0;
			CNT_READ1+4: r_sram_read_data <= sram_data;
			CNT_READ1+5: r_sram_oe <= 1;

			// read2
			CNT_READ2+2: r_sram_addr <= addr2;
			CNT_READ2+3: r_sram_oe <= 0;
			CNT_READ2+4: r_sram_read_data <= sram_data;
			CNT_READ2+5: r_sram_oe <= 1;
		endcase
	end
	r_cnt_sram <= (r_cnt_sram > CNT_READ2+1000) ? 0 : r_cnt_sram + 1;
end


// 7seg2
my_seg7_n #(.N(2)) s720(
	clk,
	1'b1,
	r_sram_addr[7:0],
	{seg72_a, seg72_b, seg72_c, seg72_d, seg72_e, seg72_f, seg72_g},
	{seg72_gnd1, seg72_gnd0}
);


// 7seg-4
my_seg7_n #(.N(4)) s74(
	clk,
	1'b1,
	r_sram_read_data,
	{seg74_a, seg74_b, seg74_c, seg74_d, seg74_e, seg74_f, seg74_g},
	{seg74_gnd3, seg74_gnd2, seg74_gnd1, seg74_gnd0}
);


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


endmodule

