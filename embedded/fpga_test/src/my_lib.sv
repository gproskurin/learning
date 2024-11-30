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


module my_bin2gray #(parameter W=8)
(
	input wire [W-1:0] bin,
	output wire [W-1:0] gray
);
	assign gray = {bin[W-1], bin[W-2:0] ^ bin[W-1:1]};
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

	reg [N*4-1:0] r_num;

	reg [$clog2(N)-1:0] r_display = 0;

	reg [9:0] r_cnt = 0;
	localparam CNT_ON = 1024 - 128;

	my_seg7_1 s7(
		clk,
		en && (r_cnt != 0) && (r_cnt < CNT_ON),
		r_num[3:0], // low bits contain current digit
		seg
	);

	always@(posedge clk)
	begin
		if (r_cnt == 0) begin
			if (r_display == 0) begin
				r_gnd <= {{(N-1){1'b1}}, 1'b0};
				r_num <= num;
			end else begin
				// shift grounds left (1 bit)
				// it will move active ground bit to left, hence we switch to ground of next segment
				r_gnd <= {r_gnd[N-2:0], r_gnd[N-1]};

				// shift number right (4 bits)
				// it will move next 4 bits of number to lowest 4 bits, which are displayed
				r_num <= {4'b0, r_num[N*4-1:4]};
			end
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


interface ws2812_color_if #(parameter W_ADDR = 6, W_DATA = 24) (input wire clk);
	wire start;
	wire done;
	wire [W_ADDR-1:0] iaddr;
	wire [W_DATA-1:0] odata;
endinterface;


module my_ws2812 #(parameter CLK_SCALE = 1)
(
	input wire clk,
	output wire ctrl,
	ws2812_color_if ColorsIf
);
	localparam CNT_DATA = 120 / CLK_SCALE;
	localparam CNT_DATA_HI_0 = 30 / CLK_SCALE;
	localparam CNT_DATA_HI_1 = 90 / CLK_SCALE;
	localparam CNT_RST = 6000 / CLK_SCALE; // >= 5000
	reg [$clog2(CNT_RST)-1:0] r_cnt;

	// color: GRB
	reg [23:0] r_color;
	reg [4:0] r_cnt_bit24;
	reg [5:0] r_cnt_led;
	localparam CNT_LEDS = 64;

	localparam S_RST = 0;
	localparam S_H = 1;
	localparam S_L = 2;
	reg [1:0] r_state = S_RST;
	assign ctrl = (r_state == S_H);

	reg r_leddata_start;
	assign ColorsIf.start = r_leddata_start;
	assign ColorsIf.iaddr = r_cnt_led;

	always@(posedge clk)
	begin
		case (r_state)
			S_RST: begin
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
				if (r_cnt_bit24 == 0) begin
					// starting new led, fetch its color
					if (r_cnt == 0) begin
						// start fetching
						r_leddata_start <= 1;
					end else if (r_cnt == CNT_DATA_HI_0 - 2) begin
						// fetched
						r_color <= ColorsIf.odata;
						r_leddata_start <= 0;
					end
				end
				if (r_cnt == (r_color[23] ? CNT_DATA_HI_1 - 1 : CNT_DATA_HI_0 - 1)) begin
					r_state <= S_L;
				end
				r_cnt <= r_cnt + 1;
			end
			S_L: begin
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

