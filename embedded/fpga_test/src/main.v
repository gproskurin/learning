module top(clk, led1, led2);

input clk;
output led1;
output led2;


// led1
reg [31:0] clk_count_1 = 32'b0;
reg led1_r = 0;
assign led1 = led1_r;

// led2
reg [7:0] clk_div_256;
wire clk_390khz = ~clk_div_256[7];
reg [18:0] clk_count_2 = 0;

reg led2_r = 0;
assign led2 = led2_r;


always @ (posedge clk)
begin
	// counter for led1
	if (clk_count_1 < 32'd100000000) begin
		if (clk_count_1 < 32'd10000000) begin
			led1_r <= 1;
		end else begin
			led1_r <= 0;
		end
		clk_count_1 <= clk_count_1 + 1;
	end else begin
		clk_count_1 <= 0;
		led1_r <= 1;
	end
end


always @ (posedge clk)
begin
	// counter for led2
	clk_div_256 <= clk_div_256 + 1;
end


always @ (posedge clk_390khz)
begin
	// 100Mhz / 256
	if (clk_count_2 < 19'd390625) begin
		if (clk_count_2 < 19'd250000) begin
			led2_r <= 1;
		end else begin
			led2_r <= 0;
		end
		clk_count_2 <= clk_count_2 + 1;
	end else begin
		clk_count_2 <= 0;
		led2_r <= 0;
	end
end

endmodule

