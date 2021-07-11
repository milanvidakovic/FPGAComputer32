
//=======================================================
//  This code is generated by Terasic System Builder
//=======================================================

module vga_module#(parameter N = 16)(
	
	input wire [1:0] vga_mode,

	//////////// CLOCK //////////
	input clk,
	input clk_50,
	input clk_25,

	//////////// reset //////////
	input reset,

	//////////// GPIO //////////
	output reg rr, 
	output reg rg, 
	output reg rb, 
	output wire hs, 
	output wire vs,

	input [N-1:0] data,
	output [N-1:0] raddr,
	output rrd, 
	output rwr,
	input inverse
);

//=======================================================
//  PARAMETER declarations
//=======================================================

localparam VIDEO_MEM_ADDR = 1024/2; //1024/2;
localparam IN_LINE = 0;
localparam H_BLANK = 1;
localparam V_BLANK = 2;

//=======================================================
//  PORT declarations
//=======================================================


//=======================================================
//  REG/WIRE declarations
//=======================================================

reg clk25; // 25MHz signal (clk divided by 2)
reg clk50;

reg r, g, b;
reg [N-1:0] addr;
reg rd; 
reg wr;

reg [9:0] x;
reg [9:0] y;
wire valid;
reg [15:0] curr_char;
reg[1:0] state;


wire [7:0] pixels; // Pixels making up one row of the character 

//=======================================================
//  Structural coding
//=======================================================


// Character generator

chars chars_1(
  .char(curr_char[7:0]),
  .rownum(y[2:0]),
  .pixels(pixels)
  ); 
wire enable;
assign enable = (vga_mode == 0) || (vga_mode == 2);
assign hs = enable ? x < (640 + 16) || x >= (640 + 16 + 96) : 1'bZ;
assign vs = enable ? y < (480 + 10) || y >= (480 + 10 + 2)  : 1'bZ;
assign rr = enable ? r : 1'bZ;
assign rg = enable ? g : 1'bZ;
assign rb = enable ? b : 1'bZ;
assign raddr = enable ? addr : {N{1'bZ}};
assign rrd = enable ? rd : 1'bZ;
assign rwr = enable ? wr : 1'bZ;

assign valid = (x < 640) && (y < 480);

reg [3:0]count_read;
reg [1:0]mem_read;
reg mem_read_again;

always @(posedge clk) begin
	if (reset) begin
		clk50 <= 1'b0;
		clk25 <= 1'b0;
		rd <= 1'b0;
		wr <= 1'b0;
		mem_read <= 1'b0;
	end 
	else begin
		clk50 <= ~clk50;
		if (clk_50 == 1'b1) begin

			if (mem_read) begin
				curr_char <= data;
				rd <= 1'b0;
				wr <= 1'b0;
				mem_read <= 1'b0;
			end
			
			if (reset) begin
				x <= 10'b0;
				y <= 10'b0;
				clk25 <= 1'b0;
				state <= IN_LINE;
			end 
			else begin
				clk25 <= ~clk25;
				if (clk_25 == 1'b1) begin
					if (x < 10'd799) begin
						x <= x + 1'b1;
					end 
					else begin
						x <= 10'b0;
						state <= IN_LINE;
						if (y < 10'd524) begin
							y <= y + 1'b1;
						end 
						else begin
							y <= 10'b0;
						end
					end
				end 
				else begin
					if (vga_mode == 0) begin
						if (x >= 640) begin
							if ((x >= 640) && (y >= 480) && ((state == H_BLANK))) begin
								// when we start the vertical blanking, we need to fetch in advance the first character (0, 0)
								state <= V_BLANK;
								rd <= 1'b1;
								wr <= 1'b0;
								addr <= VIDEO_MEM_ADDR + 0;
								mem_read <= 1'b1;
							end
							else if ((x >= 640) && ((y & 7) < 7) && (state == IN_LINE)) begin
								// when we start the horizontal blanking, and still displaying character in the current line,
								// we need to fetch in advance the first character in the current line (0, y)
								state <= H_BLANK;
								rd <= 1'b1;
								wr <= 1'b0;
								addr <= VIDEO_MEM_ADDR + ((y >> 3)*80);
								mem_read <= 1'b1;
							end
							else if ((x >= 640) && ((y & 7) == 7) && (state == IN_LINE)) begin
								// when we start the horizontal blanking, and we need to go to the next line, 
								// we need to fetch in advance the first character in next line (0, y+1)
								state <= H_BLANK;
								rd <= 1'b1;
								wr <= 1'b0;
								addr <= VIDEO_MEM_ADDR + (((y >> 3) + 1)*80);
								mem_read <= 1'b1;
							end
						end // if (!valid)
						// from this moment on, x and y are valid
						else if ((x < 640) && (!mem_read)) begin
							if ((x & 7) == 7) begin
								// when we are finishing current character, we need to fetch in advance the next character (x+1, y)
								// at the last pixel of the current character, let's fetch next
								rd <= 1'b1;
								wr <= 1'b0;
								addr <= VIDEO_MEM_ADDR + ((x >> 3) + (y >> 3)*80 + 1);
								mem_read <= 1'b1;
							end
						end 
					end
					else if (vga_mode == 2) begin
						if (x >= 640) begin
							if ((x >= 640) && (y >= 480) && ((state == IN_LINE))) begin
								// when we start the vertical blanking, we need to fetch in advance the first character (0, 0)
								state <= V_BLANK;
								rd <= 1'b1;
								wr <= 1'b0;
								addr <= VIDEO_MEM_ADDR + 0;
								mem_read <= 1'b1;
							end
							else if ((x >= 640) && (y < 480) && (state == IN_LINE)) begin
								// when we start the horizontal blanking, and still displaying character in the current line,
								// we need to fetch in advance the first character in the current line (0, y)
								state <= H_BLANK;
								rd <= 1'b1;
								wr <= 1'b0;
								addr <= VIDEO_MEM_ADDR + ((y + 1)*40);
								mem_read <= 1'b1;
							end
						end // if (!valid)
						// from this moment on, x and y are valid
						else if ((x < 640) && (!mem_read)) begin
							if ((x & 15) == 15) begin
								// when we are finishing the last pixel in a batch of 16 pixels, we need to fetch in advance the next batch at (x+1, y)
								// at the last pixel of the current character, let's fetch next
								rd <= 1'b1;
								wr <= 1'b0;
								addr <= VIDEO_MEM_ADDR + ((x >> 4) + (y*40) + 1);
								mem_read <= 1'b1;
							end
						end 
					end
				end
			end
			
			if (valid) begin
				if (vga_mode == 0)  begin
					r <= inverse ^ (pixels[7 - (x & 7)] ? !curr_char[6+8] : curr_char[2+8]);
					g <= inverse ^ (pixels[7 - (x & 7)] ? !curr_char[5+8] : curr_char[1+8]);
					b <= inverse ^ (pixels[7 - (x & 7)] ? !curr_char[4+8] : curr_char[0+8]);
				end
				else if (vga_mode == 2) begin
					r <= inverse ^ (curr_char[15 - (x & 15)]);
					g <= inverse ^ (curr_char[15 - (x & 15)]);
					b <= inverse ^ (curr_char[15 - (x & 15)]);
				end
			end 
			else begin
				// blanking -> no pixels
				r <= 1'b0;
				g <= 1'b0;
				b <= 1'b0;
			end
		end
	end
end

initial begin
		x <= 10'b0;
		y <= 10'b0;
		clk25 <= 1'b0;
		clk50 <= 1'b0;
		state <= IN_LINE;
end

endmodule
