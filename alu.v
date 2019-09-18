
module ALU#(
	parameter N = 32,
	ALU_OP_COUNT = 4,
	FLAGS_COUNT = 5
	)(
	input wire CLK, 
	input wire rst,
	// first operand
	input wire [N-1:0]a,
	// second operand
	input wire [N-1:0]b,
	// operation code (see localparams below)
	input wire [ALU_OP_COUNT-1:0] opcode,
	// if 1, then the operation is unsigned; otherwise it is signed
	input wire uns, 
	// result
	output reg [N-1:0]result,
	// flags 
	output reg [FLAGS_COUNT-1:0] flags,
	// upper bits of MUL, or remainder of DIV
	output reg [N-1:0] high,
	// if high, start multiplication/division; otherwise must be low
	input wire start,
	// goes high when multiplication/division is finished
	output reg finished//,
	// word width for division
	//input wire[7:0] div_counter
	, alu_start  // start alu operations
);

localparam GREATER_EQUAL = 4;
localparam POSITIVE = 3;
localparam OVERFLOW = 2;
localparam CARRY    = 1;
localparam ZERO     = 0;

localparam ALU_ADD = 4'd1;
localparam ALU_SUB = 4'd2;
localparam ALU_MUL = 4'd3;
localparam ALU_DIV = 4'd4;
localparam ALU_AND = 4'd5;
localparam ALU_OR  = 4'd6;
localparam ALU_XOR = 4'd7;
localparam ALU_INV = 4'd8;
localparam ALU_SHL = 4'd9;
localparam ALU_SHR = 4'd10;
localparam ALU_NEG = 4'd11;

reg [N:0] tmp;
reg [N-1:0] tmpAddSub;
reg addSubCarry, overflow;
wire [N-1:0] dres, dhigh, dcount;
wire dfinished, dstart;

div #(.N(N))div1 (
	CLK,
	a, b, 
	dres, dhigh, 
	start,
	dfinished//,
	//div_counter
);


// ZERO
assign	flags[ZERO] = ((opcode == ALU_ADD) || (opcode == ALU_SUB)) ? tmpAddSub == 0 : tmp[N-1:0] == 0;

// CARRY
assign	flags[CARRY] = addSubCarry;
	
// OVERFLOW
assign	flags[OVERFLOW] = overflow;
	
// POSITIVE
assign	flags[POSITIVE] = (uns == 0) ? (((opcode == ALU_ADD) || (opcode == ALU_SUB)) ? ~tmpAddSub[N-1]: ~tmp[N-1] ) : 1;

// GREATER OR EQUAL
assign	flags[GREATER_EQUAL] = (a >= b);

// the result
assign	result = ((opcode == ALU_ADD) || (opcode == ALU_SUB)) ? tmpAddSub : tmp[N-1:0];

add_sub add_sub (
	.add_sub(opcode == ALU_ADD),
	.dataa(a),
	.datab(b),
	.result(tmpAddSub),
	.cout(addSubCarry),
	.overflow(overflow)
	);

	
always @(posedge CLK) begin
if (rst) begin
	tmp <= 0;
end
else
	//if (alu_start) begin
	case (opcode)
	/*
		ALU_ADD: begin 
			tmp <= a + b; 
		end
		ALU_SUB: begin 
			tmp <= a - b; 
		end
	*/
		ALU_MUL: begin 
			{high, tmp[N-1:0]} <= a * b;
			finished <= 1'b1;
		end
		ALU_DIV: begin 
			if (!start && dfinished) begin
				tmp[N-1:0] <= dres;
				high <= dhigh;
				finished <= 1'b1; 
			end
			else begin
			   finished <= 1'b0; 
			end
		end
		ALU_AND: begin tmp <= a & b; tmp[N] <= 0; end
		ALU_OR : begin tmp <= a | b; tmp[N] <= 0; end
		ALU_XOR: begin tmp <= a ^ b; tmp[N] <= 0; end
		ALU_INV: begin tmp <= ~a; tmp[N] <= 0; end
		ALU_SHL: tmp <= a << b;
		ALU_SHR: tmp <= a >> b;
		ALU_NEG: begin tmp <= -b; tmp[N] <= 0; end
	endcase
	//end
	//else begin
	/*
		// ZERO
		flags[ZERO] <= tmp[N-1:0] == 0;

		// CARRY
		flags[CARRY] <= tmp[N];
			
		// OVERFLOW
		flags[OVERFLOW] <= (opcode == ALU_ADD) && (a[N-1] == b[N-1]) && (tmp[N-1] != a[N-1]);
			
		// POSITIVE
		flags[POSITIVE] <= (uns == 0) ? !tmp[N-1] : 1;

		// GREATER OR EQUAL
		flags[GREATER_EQUAL] <= (a >= b);

		// the result
		result <= tmp[N-1:0];
	*/
	//end
	//$display("[[[[flagsPOCZ=%2b, res=%4x, h=%4x, d=%4x, finished=%x]]]]", flags, tmp, dhigh, dres, dfinished);
end


endmodule
