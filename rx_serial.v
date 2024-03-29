module rx_serial(
	input clk,
	input rst,  // reset signal
	input rx,  // GPIO pin for tx signal
	output [7:0] rx_data,  // received data buffer
	output reg rx_received // goes high when received one byte
);

//localparam RX_CLOCK_MUL = 13'd868;
localparam RX_CLOCK_MUL = 13'd868; // 108 for 921600 baud on 100MHz
// Divide 50 MHz by 434 to get approximately 115200 baud (divide by 5208 to get 9600 baud)
//localparam RX_CLOCK_MUL = 13'd434;
//localparam RX_CLOCK_MUL = 13'd217;
//localparam RX_CLOCK_MUL = 13'd108;
reg [12:0] rx_counter;

localparam SPACE = 1'b0, MARK = 1'b1;

reg [4:0] state, rx_bit_count ; // Bit counter
localparam IDLE = 5'd0, START = 5'd1, SAMPLE_BITS = 5'd2, STOP = 5'd3;

reg [7:0] char1;
reg start_detected;

reg rx1, rx2, rx3, rx4;

assign rx_data = (rx_received) ? char1 : 8'bz;

always @ (posedge clk) begin
  if (rst) begin
    state <= IDLE;
    rx_counter <= 13'b0;
	 rx_received <= 1'b0;
	 rx_bit_count  <= 5'b0;
	 char1 <= 8'b0;
	 rx1 <= 1'b1;
	 rx2 <= 1'b1;
	 rx3 <= 1'b1;
	 rx4 <= 1'b1;
  end 
  else begin 
	rx1 <= rx;
	rx2 <= rx1;
	rx3 <= rx2;
	rx4 <= rx3;
	case (state)
		IDLE: begin
			rx_bit_count  <= 5'b0;
			rx_received <= 1'b0;
			if ((rx3 == 0 && rx2 == 0) || (rx2 == 0 && rx1 == 0)) begin
				rx_counter <= 13'b0;
				state <= START;
			end 
		end
		START: begin
			if(rx_counter == (RX_CLOCK_MUL + (RX_CLOCK_MUL/2) - 1'b1)) begin
				rx_counter <= 13'b0;
				state <= SAMPLE_BITS;
			end 
			else begin
				rx_counter <= rx_counter + 1'b1;
			end
		end
		SAMPLE_BITS: begin
			if(rx_counter == 13'b0) begin
				if(rx_bit_count == 7) begin
					char1 <= {rx, char1[7:1]};				
					state <= STOP;
					rx_received <= 1'b0;
				end 
				else begin
					char1 <= {rx, char1[7:1]};
					rx_bit_count <= rx_bit_count + 1'b1;
				end
			end 
			
			if(rx_counter == (RX_CLOCK_MUL - 1'b1)) begin
				rx_counter <= 13'b0;
			end 
			else begin
				rx_counter <= rx_counter + 1'b1;
			end
		end
		STOP: begin
			// counter is at 1 when entering this state
			if(rx_counter == 13'b0) begin
				if(rx) begin // transmission done
					// Stop bit received
					state <= IDLE;
					
					// the system has until the next stop bit to pull the data
					// and must wait for rx_received to go low before the next data
					// is available
					rx_received <= 1'b1; 
				end 
				else begin 
					// There's no stop bit, so we assume there's a transmission error
					// For now, ignore the data. TODO: transmission error output signal
					state <= IDLE;
				end
			end 
			
			if(rx_counter == (RX_CLOCK_MUL - 1'b1)) begin
				rx_counter <= 13'b0;
			end else begin
				rx_counter <= rx_counter + 1'b1;
			end	
		end
	 endcase
	end
end




initial begin
    state <= IDLE;
    rx_counter <= 13'b0;
	 rx_bit_count  <= 5'b0;
	 rx_received <= 0;
	 char1 <= 0;
	 rx1 <= 1'b1;
	 rx2 <= 1'b1;
	 rx3 <= 1'b1;
	 rx4 <= 1'b1;
end

endmodule
