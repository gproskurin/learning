`default_nettype none

module my_pwm
(
	input wire pwm_clk,
	input [3:0] pwm_total,
	input [3:0] pwm_on,
	output reg out
);
	reg [3:0] cnt = 0;
	assign out = (cnt <= pwm_on) ? 1 : 0;

	always @ (posedge pwm_clk)
	begin
		cnt <= (cnt < pwm_total) ? (cnt + 1) : 0;
	end

endmodule


function integer clk_div_width;
	input integer x;
	integer t;
	begin
		t = x - 1;
		if ((t & ~t) == 0) begin
			// x-1 is power of 2
			clk_div_width = $clog2(x) - 1;
		end else begin
			clk_div_width = $clog2(x-1) - 1;
		end
	end
endfunction

module my_clk_div
	#(parameter DIV = 0)
(
	input wire clk,
	output wire out
);
	reg [clk_div_width(DIV):0] r_cnt;
	assign out = (r_cnt < DIV/2) ? 1 : 0;

	always @ (posedge clk)
	begin
		r_cnt <= (r_cnt == DIV-1) ? 0 : r_cnt + 1;
	end

endmodule


module my_seg7_1
(
	input wire clk,
	input wire en,
	input wire [3:0] num,
	output wire [6:0] seg
);
	reg [6:0] r_seg;
	assign seg = r_seg;

	always @ (posedge clk)
	begin
		if (!en)
			r_seg <= 0;
		else begin
			case (num)
				4'h0: r_seg <= 7'b1111110;
				4'h1: r_seg <= 7'b0110000;
				4'h2: r_seg <= 7'b1101101;
				4'h3: r_seg <= 7'b1111001;
				4'h4: r_seg <= 7'b0110011;
				4'h5: r_seg <= 7'b1011011;
				4'h6: r_seg <= 7'b1011111;
				4'h7: r_seg <= 7'b1110000;
				4'h8: r_seg <= 7'b1111111;
				4'h9: r_seg <= 7'b1111011;
				4'hA: r_seg <= 7'b1110111;
				4'hB: r_seg <= 7'b0011111;
				4'hC: r_seg <= 7'b1001110;
				4'hD: r_seg <= 7'b0111101;
				4'hE: r_seg <= 7'b1001111;
				4'hF: r_seg <= 7'b1000111;
			endcase
		end
	end

endmodule


module my_seg7_n #(parameter N = 1)
(
	input wire clk,
	input wire en,
	input wire [N*4-1:0] num,
	output wire [6:0] seg,
	output wire [N-1:0] seg_gnd
);
	reg [N-1:0] r_gnd;
	assign seg_gnd = r_gnd;

	reg [3:0] r_num_out;

	reg [$clog2(N)-1:0] r_display = 0;

	reg [9:0] r_cnt = 0;
	localparam CNT_ON = 1024 - 128;

	my_seg7_1 s7(
		clk,
		en && (r_cnt != 0) && (r_cnt < CNT_ON),
		r_num_out,
		seg
	);

	integer i;
	always@(posedge clk)
	begin
		if (r_cnt == 0) begin
			if (r_display == 0) begin
				r_gnd <= ~(N'd1);
				r_num_out <= num[3:0];
			end else begin
				// shift
				for (i=1; i<N; ++i)
					r_gnd[i] <= r_gnd[i-1];
				r_gnd[0] <= r_gnd[N-1];
			end
			// TODO: unhardcode, support N>4 (shift similar to gnd?)
			case (r_display)
				1: begin
					r_num_out <= num[7:4];
				end
				2: begin
					r_num_out <= num[11:8];
				end
				3: begin
					r_num_out <= num[15:12];
				end
			endcase
		end else if (r_cnt == CNT_ON) begin
			r_display <= (r_display == N-1) ? 0 : r_display + 1;
		end
		r_cnt <= r_cnt + 1;
	end

endmodule


module my_keypad_led(input wire in, output wire out);
	assign out = (in == 1) ? 1'b0 : 1'bZ;
endmodule


module my_debounce #(parameter DELAY = 100000000/12)
	(input wire clk, input wire in, output wire out);

reg [$clog2(DELAY)-1:0] cnt;
reg r_out;
assign out = r_out;

always@(posedge clk) begin
	case (cnt)
		0: begin
			if (in != r_out) begin
				// input changed
				// copy input to output and start counting
				r_out <= in;
				cnt <= 1;
			end
		end
		DELAY: begin
			// end of delay
			// copy input to output and reset counter
			r_out <= in;
			cnt <= 0;
		end
		default: cnt <= cnt + 1;
	endcase
end

endmodule


module my_ws2812 #(parameter CLK_SCALE = 1)
(
	input wire clk,
	input wire clk_ws,
	output wire ctrl,
	output wire [5:0] leddata_addr,
	output wire leddata_start,
	//input wire led_data_done,
	input wire [23:0] leddata_color,
	output wire [7:0] dbgout8,
	output wire [15:0] dbgout16,
	output wire dbg_start,
	output wire dbg_done
);
	localparam CNT_DATA = 120 / CLK_SCALE;
	localparam CNT_DATA_HI_0 = 30 / CLK_SCALE;
	localparam CNT_DATA_HI_1 = 90 / CLK_SCALE;
	localparam CNT_RST = 6000 / CLK_SCALE; // >= 5000
	reg [$clog2(CNT_RST)-1:0] r_cnt;

	// color: GRB
	reg [23:0] r_color;
	reg [23:0] r_color_init;
	reg [4:0] r_cnt_bit24;
	reg [5:0] r_cnt_led;
	localparam CNT_LEDS = 64;
	//assign led_data_addr = r_cnt_led;
	assign dbgout8[7:6] = 0;
	assign dbgout8[5:4] = r_state;
	assign dbgout8[3:0] = r_cnt_led;
	assign dbgout16[15:8] = r_cnt;
	assign dbgout16[7:5] = 0;
	assign dbgout16[4:0] = r_cnt_bit24;

	//assign dbgout8[5:0] = r_cnt_led;
	//assign dbgout8[7:6] = 0;

	localparam S_RST = 0;
	localparam S_H = 1;
	localparam S_L = 2;
	reg [1:0] r_state = S_RST;
	//assign ctrl = (r_state == S_H);
	//assign dbgout8[4:0] = r_cnt_led;
	//assign dbgout8[7:5] = 0;

	//assign dbgout8[5:0] = r_cnt_led;
	//assign dbgout8[7:6] = 0;
	//assign dbgout8[7:4] = r_cnt[3:0];
	//assign dbgout8[3:0] = r_cnt_led[3:0];
	//assign dbgout16 = r_color[15:0];
	//assign dbgout16 = led_data_color[15:0];

	reg r_leddata_start;
	assign leddata_start = r_leddata_start;
	assign leddata_addr = r_cnt_led;
	//assign leddata_start = r_state==S_H && r_cnt_bit24==0 && r_cnt==0;
	assign dbg_start = ~r_leddata_start;

	/*
	always@(posedge r_leddata_start)
	begin
		case (r_cnt_led)
			0: r_color_init <= 24'h0F0101;
			9: r_color_init <= 24'h010F01;
			18: r_color_init <= 24'h01010F;
			27: r_color_init <= 24'h0F0101;
			36: r_color_init <= 24'h0F0101;
			45: r_color_init <= 24'h010F01;
			54: r_color_init <= 24'h01010F;
			63: r_color_init <= 24'h0F000F;
			default: r_color_init <= 24'h010101;
		endcase
	end
	*/

	always@(posedge clk)
	begin
		case (r_state)
			S_RST: begin
				ctrl <= 0;
				r_cnt_led <= 0; // init
				r_cnt_bit24 <= 0; // init
				if (r_cnt == CNT_RST - 1) begin
					r_cnt <= 0;
					r_state <= S_H;
				end else begin
					r_cnt <= r_cnt + 1;
				end
			end
			S_H: begin
				ctrl <= 1;
				if (r_cnt_bit24 == 0) begin
					// starting new led, fetch its color
					if (r_cnt == 0) begin
						// start fetching
						r_leddata_start <= 1;
					end else if (r_cnt == CNT_DATA_HI_0 - 2) begin
						// fetched
						r_color <= leddata_color;
						r_leddata_start <= 0;
					end
				end
				if (r_cnt == (r_color[23] ? CNT_DATA_HI_1 - 1 : CNT_DATA_HI_0 - 1)) begin
					r_state = S_L;
				end
				r_cnt <= r_cnt + 1;
			end
			S_L: begin
				ctrl <= 0;
				if (r_cnt == CNT_DATA - 1) begin
					// end of bit, either switch to the next bit, or to the next led/reset
					r_cnt <= 0;
					if (r_cnt_bit24 == 23) begin
						// processed all 24 bits, current led complete
						// either switch to the next led, or to "reset" state if it was the last led
						if (r_cnt_led == CNT_LEDS-1) begin
							// it was the last led, switch to "reset" state
							r_state <= S_RST;
						end else begin
							// switch to the next led
							r_cnt_bit24 <= 0;
							r_cnt_led <= r_cnt_led + 1;
							r_state <= S_H;
						end
					end else begin
						// switch to the next bit of current led
						r_cnt_bit24 <= r_cnt_bit24 + 1;
						r_color <= {r_color[22:0], r_color[23]}; // rotate
						r_state <= S_H;
					end
				end else begin
					r_cnt <= r_cnt + 1;
				end
			end
		endcase
	end


endmodule

