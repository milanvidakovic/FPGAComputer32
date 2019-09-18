module div#(
	parameter N = 32)(CLK, X, Y, res, high, start, finished);

input CLK;
// X / Y
input [N-1:0]  X, Y;
// result of the division, and remainder
output reg [N-1:0] res, high;
// if high, start division; otherwise must be low
input start;
// goes high when division is finished
output reg finished;

// local variables
reg started;
reg [N-1:0] A, Q, M, newA;
reg [7:0] n;

always @ (posedge CLK) begin

	if (start) begin
		started <= 1'b1;
		finished <= 1'b0;
		newA <= 0;
		A <= 0;
		if (X[N-1] == 0) begin
			Q <= X;
		end
		else begin
			Q <= -X;
		end
		if (Y[N-1] == 0) begin
			M <= Y;
		end
		else begin
			M <= -Y;
		end
		n <= N;
	end
	else if (started) begin
		A = {A[N-2:0], Q[N-1]};
		Q = Q << 1;
		newA = A - M;
$display("newA: %x, A: %x, Q: %x, n: %x, M: %x,finished: %d", newA, A, Q, n, M, finished);
		if (newA[N-1] == 0) begin
			A = newA;
			Q[0] = 1'b1;
		end
		n = n - 1'b1;
		if (n == 0) begin
			started <= 1'b0;
			finished <= 1'b1;
			if (X[N-1] == Y[N-1]) begin
				res <= Q;
			end 
			else begin
				res <= -Q;
			end
			if (X[N-1] == A[N-1]) begin
				high <= A;
			end 
			else begin
				high <= -A;
			end
		end
	end
	else if (!started && finished) begin
		finished <= 1'b0;
	end
$display("newA: %x, A: %x, Q: %x, n: %x, M: %x,finished: %d", newA, A, Q, n, M, finished);
end

initial begin
	started <= 1'b0;
	finished <= 1'b0;
end
endmodule
