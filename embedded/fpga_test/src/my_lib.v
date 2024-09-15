module my_pwm
(
	input wire pwm_clk,
	input [3:0] pwm_total,
	input [3:0] pwm_on,
	output wire out
);
	reg [3:0] cnt = 0;
	reg out_r = (cnt < pwm_on) ? 1 : 0;
	assign out = out_r;

	always @ (posedge pwm_clk)
	begin
		cnt <= (cnt < pwm_total) ? (cnt + 1) : 0;
	end

endmodule


module my_clk_div
	#(parameter W = 27)
(
	input wire clk,
	input [W-1:0] div,
	output wire out
);
	reg [W-1:0] r_cnt;
	reg r_out;
	assign out = r_out;

	always @ (posedge clk)
	begin
		r_out = (r_cnt < div/2) ? 1 : 0;
		r_cnt = (r_cnt < div-1) ? r_cnt + 1 : 0;
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
		if (!en) r_seg <= 0;
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


module my_seg7_2(
	input wire clk,
	input wire en,
	input wire [7:0] num,
	output wire [6:0] seg,
	output wire [1:0] seg_gnd
);
	reg [1:0] r_seg_gnd;
	assign seg_gnd = r_seg_gnd;
	reg r_select;
	reg [3:0] r_num;

	my_seg7_1 my_seg71(clk, en, r_num, seg);
	always @ (posedge clk)
	begin
		if (!en) begin
			r_seg_gnd = {1, 1};
		end else begin
			if (!r_select) begin
				// segment 0
				r_seg_gnd = 2'b10;
				r_num <= num[0:3];
			end else begin
				// segment 1
				r_seg_gnd = 2'b01;
				r_num <= num[7:4];
			end
			r_select <= ~r_select;
		end
	end

endmodule

