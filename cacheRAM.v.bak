module RAM
(
	input  clk,
	output [15:0] data,
	input [15:0] addr,
	input rd, wr,
	input [15:0] indata,
	
	output [15:0] data2,
	input [15:0] addr2,
	input rd2
);
	// Declare the RAM variable
	reg [15:0] ram[22528:0];

	
	always @ (negedge clk)
	begin

		if (wr && !rd) 
		begin
			ram[addr] <= indata;
			data <= {16{1'bz}};
		end
		else if (!wr && rd) begin
			data <= ram[addr];
		end
		else begin
			data <= {16{1'bz}};
		end
		
		if (rd2) begin
			data2 <= ram[addr2];
		end
		else begin
			data2 <= {16{1'bz}};
		end
		
		$display("MEMORY addr:%-x > %-x, %-x, \t %-x, %-x", addr, ram[addr], ram[addr + 1], ram[addr + 2], ram[addr + 3]);
		
	end	
	
initial
begin
  $readmemh("ram.hex", ram);
end

endmodule 