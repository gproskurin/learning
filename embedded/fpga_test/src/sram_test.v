`default_nettype none

module sram_test(
	input clk,
	output wire [17:0] sram_addr,
	inout wire [15:0] sram_data,
	output wire sram_cs,
	output wire sram_oe,
	output wire sram_we,
	output wire[15:0] dbgout16,
	output wire[7:0] dbgout8,
	output wire dbgout1,
	output wire out_err
);


reg r_sram_oe = 1;
assign sram_oe = r_sram_oe;

reg r_sram_we = 1;
assign sram_we = r_sram_we;

reg r_sram_cs = 0;
assign sram_cs = r_sram_cs;

reg [15:0] r_sram_data;
assign sram_data = r_sram_oe ? r_sram_data : 16'bZ;

reg [17:0] r_sram_addr;
assign sram_addr = r_sram_addr;

reg [15:0] r_data0 = 0;
reg [15:0] r_data;

reg r_state = 0;
reg r_mode_read = 0;
reg r_init_done = 0;
assign dbgout8 = r_data0[7:0];
assign dbgout16 = r_sram_addr[15:0];
assign dbgout1 = r_mode_read;;

always@(posedge clk)
begin
	if (!r_mode_read) begin
		if (!r_init_done) begin
			// init once before write

			// (ab)use r_state for 2 cycles long init
			if (r_state == 0) begin
				// 1st cycle of init: disable writes and wait until next cycle
				r_sram_we <= 1; // disable writes
				r_state <= 1;
			end else begin
				// 2nd cycle of init: init everything else
				r_sram_oe <= 1;
				r_init_done <= 1;
				r_sram_data <= r_data0;
				r_sram_addr <= 0;
				r_state <= 0;
			end
		end else begin
			// do write
			case (r_state)
				// pulse WE
				0: begin
					r_sram_we <= 0;
					r_state <= 1;
				end
				1: begin
					// rely on sram chip timing: WE should go up fast enough and disable writes before
					r_sram_we <= 1;
					r_state <= 0;
					// update data and addr
					if (&r_sram_addr) begin
						// switch to read
						r_mode_read <= 1;
						r_init_done <= 0;
					end else begin
						r_sram_data <= r_sram_data + 1;
						r_sram_addr <= r_sram_addr + 1;
					end
				end
			endcase
		end
	end else begin
		if (!r_init_done) begin
			r_init_done <= 1;
			r_sram_addr <= 0;
			r_sram_oe <= 0;
			out_err <= 0;
			r_data <= r_data0;
		end else begin
			// read
			// data propagation / timing: after we change address register, skip (delay) one cycle before reading
			// - cycle0: update address register (do the same as in cycle2)
			// - cycle1: address register update propagated to sram address lines, sram chip starts fetching data
			// - cycle2: do multiple things simultaneously:
			//     (a) sram chip data lines contain valid data, read it and act on it
			//     (b) update address register, rely on data hold time for sram chip (tOH >= 3ns), data is
			//         still valid while we are doing (a). TODO: is it reliable?
			case (r_state)
				0: begin
					// delay, some init
					out_err <= 0;
					r_state <= 1;
				end
				1: begin
					// read data
					if (sram_data != r_data)
						out_err <= 1;
					// increment address
					if (&r_sram_addr) begin
						// end of address space, switch to write
						r_init_done <= 0;
						r_mode_read <= 0;
						r_data0 <= r_data0 + 1;
					end else begin
						r_sram_addr <= r_sram_addr + 1;
						r_data <= r_data + 1;
					end
					r_state <= 0;
				end
			endcase
		end
	end
end


endmodule

