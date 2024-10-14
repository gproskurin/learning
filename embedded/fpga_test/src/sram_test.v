`default_nettype none

module sram_test(
	input clk,
	output wire [17:0] sram_addr,
	inout wire [15:0] sram_data,
	output wire sram_cs,
	output wire sram_oe,
	output wire sram_we,
	output wire[15:0] dbgout_sram_data,
	output wire out_err
);


reg r_sram_oe = 1;
assign sram_oe = r_sram_oe;

reg r_sram_we = 1;
assign sram_we = r_sram_we;

reg r_sram_cs = 0;
assign sram_cs = r_sram_cs;

reg [15:0] r_sram_data_out;
assign sram_data = r_sram_oe ? r_sram_data_out : 16'bZ;

reg [17:0] r_sram_addr;
assign sram_addr = r_sram_addr;


reg r_write_done = 0;
localparam addr1 = 18'b000101010001011010;
localparam addr2 = 18'b111010110010110100;

localparam CNT_READ1 = 14;//100000000/2;
localparam CNT_READ2 = CNT_READ1+10;//100000000;
localparam CNT_MAX = CNT_READ2 + 10;
reg [$clog2(CNT_MAX)-1:0] r_cnt_sram;
reg [15:0] r_sram_read_data;
assign dbgout_sram_data = r_sram_read_data;

always@(posedge clk)
begin
	if (r_write_done == 0) begin
		out_err <= 0;
		case (r_cnt_sram)
			0: begin
				r_sram_oe <= 1;
				r_sram_we <= 1;
			end

			// write1 - zero
			1: begin
				r_sram_addr <= addr1;
				r_sram_data_out <= 0;
			end
			2: r_sram_we <= 0;
			3: r_sram_we <= 1;

			// write1 - data
			4: r_sram_data_out <= 16'Hdead;
			5: r_sram_we <= 0;
			6: r_sram_we <= 1;

			// write2 - zero
			7: begin
				r_sram_addr <= addr2;
				r_sram_data_out <= 0;
			end
			8: r_sram_we <= 0;
			9: r_sram_we <= 1;

			// write2 - data
			10: r_sram_data_out <= 16'Hbeef;
			11: r_sram_we <= 0;
			12: r_sram_we <= 1;

			13: begin
				r_sram_addr <= 0;
				r_sram_data_out <= 0;
				r_sram_read_data <= 0;
				r_write_done <= 1;
			end
		endcase
	end else begin
		case (r_cnt_sram)
			CNT_READ1: begin
				r_sram_oe <= 0;
			end

			// read1
			CNT_READ1+1: r_sram_addr <= addr1;
			// skip 2 cycles (10ns each): addr settles in 1 cycle, then data is valid in 1 cycle
			CNT_READ1+3: r_sram_read_data <= sram_data;
			CNT_READ1+4: out_err <= (r_sram_read_data == 16'Hdead) ? 0 : 1;
			CNT_READ1+5: out_err <= 0;

			// read2
			CNT_READ2: r_sram_addr <= addr2;
			// skip 2 cycles
			CNT_READ2+2: r_sram_read_data <= sram_data;
			CNT_READ2+3: out_err <= (r_sram_read_data == 16'Hbeef) ? 0 : 1;
			CNT_READ2+4: begin
				out_err <= 0;
				r_sram_oe <= 1;
			end
		endcase
	end
	r_cnt_sram <= (r_cnt_sram == CNT_MAX) ? 0 : r_cnt_sram + 1;
end


endmodule

