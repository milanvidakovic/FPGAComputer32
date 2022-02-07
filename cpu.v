//`define READ_ONLY
`define FLOAT
`define CACHE

module cpu #(
  parameter     ADDR_WIDTH    = 24
)
(
  input             clk,
  input             rst,

  // SDRAM
  output   reg      rd_enable_o,
  output   reg      wr_enable_o,
  input             busy_i,
  input             rd_ready_i,
  input  [15:0]     rd_data_i,
  output reg [15:0] wr_data_o,
  output reg [ADDR_WIDTH-1:0] addr_o,

  output [7:0]      LED,
  
  // static RAM
  input  [15:0]     data_i,
  output [15:0]     data_to_write_o,
  output            rd,
  output            wr,
  
  // PORTS
  input  [7:0] 	rx_data,		// UART RECEIVED BYTE 
  output 			tx_send,		// UART TX send signal
  input  			tx_busy,		// UART TX busy signal
  output [7:0] 	tx_data,    // UART TX data
  output [1:0] 	vga_mode,	// VGA mode: 0-text; 1-320x240
  output 			inverse,		// VGA text mode inverse or not
  input  [7:0] 	ps2_data,   // keyboard data
  input  [7:0] 	ps2_data_mouse,   // mouse data
  
  // SPI0
  output  		 	spi_start,
  input [7:0] 		spi_in,
  output [7:0] 	spi_out,
  input 				spi_ready,

  // SPI1
  output  		 	spi_start1,
  input [7:0] 		spi_in1,
  output [7:0] 	spi_out1,
  input 				spi_ready1,

  output				spi_cs,
  output				spi_cs1,
  
  input  [15:0] 	irq_i

);

localparam  INIT        = 1,
				READ_DATA   = 5,
				READ_WAIT	= 10,
				WRITE_DATA	= 15,
				WRITE_WAIT	= 20,
				FETCH       = 25,
				DECODE      = 30,
				EXECUTE     = 35,
				CHECK_IRQ   = 40;

localparam SP_I = 15;
localparam SP = 15;
localparam H  = 14;

localparam IRQ_TIMER = 0;
localparam IRQ_UART  = 1;
localparam IRQ_PS2   = 2;
localparam IRQ_SPI   = 3;
localparam IRQ_SPI1  = 4;
localparam IRQ_PS2_MOUSE   = 5;
localparam IRQ_DMA_1 = 14;
localparam IRQ_DMA_2 = 15; 

localparam FLAGS_COUNT   = 4;
localparam GREATER_EQUAL = 4;
localparam POSITIVE = 3;
localparam OVERFLOW = 2;
localparam CARRY    = 1;
localparam ZERO     = 0;

localparam ALU_OP_COUNT = 4;
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

localparam SDRAM_START_ADDR = 32'hB000/2;

localparam PORT_UART_RX_BYTE					= 640	; //port which contains received byte via UART
localparam PORT_UART_TX_BUSY					= 650	; //port which has 1 when UART TX is busy
localparam PORT_UART_TX_SEND_BYTE				= 660	; //port for sending character via UART
localparam PORT_LED								= 670	; //port for setting eight LEDs (write)
localparam PORT_KEYBOARD 						= 680	; //raw keyboard character read port 
localparam PORT_MOUSE	 						= 800	; //raw mouse byte read port 
localparam PORT_MOUSE_STRUCT_ADDR				= 810	; //mouse struct address
localparam PORT_MILLIS 							= 690	; //current number of milliseconds counted so far

localparam PORT_SPI_IN 							= 700	; //port which contains received byte via SPI
localparam PORT_SPI_OUT 						= 710	; //port for sending byte via SPI
localparam PORT_SPI_OUT_BUSY					= 720	; //port for sending byte via SPI

localparam PORT_SPI1_IN 						= 750	; //port which contains received byte via SPI
localparam PORT_SPI1_OUT 						= 760	; //port for sending byte via SPI
localparam PORT_SPI1_OUT_BUSY					= 770	; //port for sending byte via SPI

localparam PORT_SPI_CS							= 730	; //port for CS (SS).
localparam PORT_SPI1_CS							= 740	; //port for CS1 (SS1).

localparam PORT_VIDEO_MODE						= 1280	; //video mode type (0-text; 1-graphics 1, 2-graphics2), (write)
localparam PORT_TIMER     						= 1290	; //timer irq port (number of milliseconds before the irq is triggered)
localparam VGA_TEXT_INVERSE						= 1300	; //if 1, then the screen is inversed (black letters on white background)

localparam PORT_DMA_ADDR_1						= 1400	; // DMA channel 1, transfer start address
localparam PORT_DMA_COUNT_1						= 1420	; // DMA channel 1, number of bytes to be transferred
localparam PORT_DMA_START_RCV_1					= 1470	; // DMA channel 1, start receiving

localparam PORT_DMA_ADDR_2						= 1430	; // DMA channel 2, transfer start address
localparam PORT_DMA_COUNT_2						= 1450	; // DMA channel 2, number of bytes to be transferred
localparam PORT_DMA_START_RCV_2					= 1480	; // DMA channel 2, start receiving
 

reg	[5:0]  next_state;
reg	[5:0]  state;

reg	[15:0] rd_data_r;
reg [15:0]   data_r, data_to_write;
reg [31:0]   addr;

`ifdef CACHE
// ###########################################
// CACHE
// ###########################################
// cache TAG
	reg [11:0] tag[4095:0];
// cache line
	reg [15:0] cl[4095:0];
// ###########################################
`endif


// ###########################################
// SDRAM INIT PAUSE
// ###########################################
/* Counter to wait until sdram init cycle is complete.  */
reg   [7:0]  init_cnt;
reg init_wait;

assign init_wait = |init_cnt;
always @ (posedge clk)
if (rst) begin
  init_cnt <= 8'hFF; 
end
else if (init_wait) begin
  init_cnt <= init_cnt - 1'b1;
end

// #####################################
// REGISTERS
// #####################################
reg [31:0] regs[16:0];  
reg [31:0] pc, mbr, mbr_e;
reg [15:0] ir;
reg [15:0] mc_count, irq_state;
reg [15:0] irq_r, irq;
reg [15:0] do_irq;
reg [7:0]  rx_data_r, ps2_data_r, ps2_data_r_mouse;
reg [7:0]  spi_in_r;
reg [7:0]  spi_in_r1;

assign irq[13:1] = irq_i[13:1]; 

// #####################################
// TIMERS & COUNTERS
// #####################################
reg [31:0] millis_counter;
reg [31:0] clock_counter;
reg [31:0] timer_counter, timer;


// ################
// MOUSE REGISTERS
// ################
reg [31:0] mouse_struct_addr; // pointer to mouse struct in memory (holds x, y coord, mouse key and status (1-changed, 0-not changed)

// #####################################
// DMA REGISTERS
// #####################################
reg[31:0] dma_addr_1;	// start address of a memory location where data should be received/transmitted
reg[31:0] dma_count_1;  // number of bytes that needs to be received/transmitted
reg[31:0] dma_current_1;// current number of bytes received/transmitted
reg[7:0] dma_byte_1; 	// backup of a received byte, since we need to receive two bytes in order to place them in 16-bit memory
reg dma_start_rcv_1; 	// start receiving data
reg spi_sent_ff_1;
reg dma_spi_received_1;

reg [31:0]not_received_counter;

reg[31:0] dma_addr_2;
reg[31:0] dma_count_2;
reg[31:0] dma_current_2;
reg[7:0] dma_byte_2;
reg dma_start_rcv_2;

// #####################################
// ALU stuff
// #####################################
reg [FLAGS_COUNT-1:0]  f;
wire [FLAGS_COUNT:0]   f_from_alu;
reg [ALU_OP_COUNT-1:0] alu_op;
reg                    alu_uns;
wire [31:0]            alu_a, alu_b, alu_res, alu_high;
wire                   div_finished;
reg                    start_div;
reg						  alu_start;

// #####################################
// FLOATING POINT
// #####################################
`ifdef FLOAT
localparam MANTISSA_LEN = 23;

reg [MANTISSA_LEN*2+1:0] ma, mb, tfe;
wire fdiv_finished;
reg fstart_div;
reg [MANTISSA_LEN*2+1:0] fres, frem;
wire [31:0] fmulres, fdivres, fadd_sub_res;
wire overflow, underflow;
reg clk_en, fp_add_sub;

fpaddsub fpaddsub_op(
	.clock(clk),
	.dataa(ma),
	.datab(mb),
	.clk_en(clk_en),
	.add_sub(fp_add_sub),
	.result(fadd_sub_res), .overflow(overflow), .underflow(underflow));

fpmul fpmul_op(
	.clock(clk),
	.dataa(ma),
	.datab(mb),
	.clk_en(clk_en),
	.result(fmulres), .overflow(overflow), .underflow(underflow));

fpdiv fpdiv_op(
	.clock(clk),
	.dataa(ma),
	.datab(mb),
	.clk_en(clk_en),
	.result(fdivres), .overflow(overflow), .underflow(underflow));
`endif						

integer i, j;

ALU alu(
	// clock
	clk, 
	rst,
	// first operand
	alu_a,
	// second operand
	alu_b,
	// operation code (see localparams below)
	alu_op,
	// signed = 0, unsigned = 1
	alu_uns,
	// result
	alu_res,
	// flags 
	f_from_alu,
	// upper bits of MUL, or remainder of DIV
	alu_high,
	// if high, start multiplication/division; otherwise must be low
	start_div,
	// goes high when multiplication/division is finished
	div_finished,
	alu_start
);

always @ (posedge clk)
begin
if (rst) begin
  state <= INIT;
  clock_counter  <= 0;
  millis_counter <= 0;
  timer <= 0;
  LED <= 0;
  rd_enable_o <= 0;
  wr_enable_o <= 0;
  rd <= 0;
  wr <= 0;
  vga_mode <= 2'b00;
  //regs[SP_I] <= 47100;
  dma_addr_1 <= 0;
  dma_count_1 <= 0;
  dma_current_1 <= 0;
  dma_start_rcv_1 <= 0;
  dma_addr_2 <= 0;
  dma_count_2 <= 0;
  dma_current_2 <= 0;
  dma_start_rcv_2 <= 0;
  spi_sent_ff_1 <= 0;
  dma_spi_received_1 <= 1;
  not_received_counter <= 0;
end
else begin
	if (clock_counter < 100000) begin
		clock_counter <= clock_counter + 1'b1;
	end
	else begin
		clock_counter <= 0;
		millis_counter <= millis_counter + 1'b1;
		if (timer && (timer_counter < timer)) begin
			timer_counter <= timer_counter + 1'b1;
		end
		else if (timer && (timer_counter == timer)) begin
			irq[IRQ_TIMER] <= 1;
			timer_counter <= 0;
		end 
	end
	
	// check if the DMA byte counter 1 (dma_current_1) has reached the given value (dma_count_1), 
	// and if it has reached, then fire the DMA interrupt 1
	if (dma_count_1 && (dma_current_1 == dma_count_1)) begin
			irq[IRQ_DMA_1] <= 1;
			dma_current_1 <= 0;
			dma_count_1 <= 0;
			dma_start_rcv_1 <= 0;
	end 


	if ((irq) && (irq_state==0)) begin
		if (irq[IRQ_TIMER] & do_irq[IRQ_TIMER]) begin
			// timer
			irq[IRQ_TIMER] <= 0;
			irq_r[IRQ_TIMER] <= 1;
			irq_state <= 1;
		end
		if (irq[IRQ_UART] & do_irq[IRQ_UART]) begin
			// UART byte arrived
			rx_data_r <= rx_data;
			irq_r[IRQ_UART] <= 1;
			irq_state <= 1;
		end
		if (irq[IRQ_PS2] & do_irq[IRQ_PS2]) begin
			// PS/2 key pressed/released
			ps2_data_r <= ps2_data;
			irq_r[IRQ_PS2] <= 1;
			irq_state <= 1;
		end		
		if (irq[IRQ_PS2_MOUSE] & do_irq[IRQ_PS2_MOUSE]) begin
			// PS/2 mouse byte arrived
			ps2_data_r_mouse <= ps2_data_mouse;
			irq_r[IRQ_PS2_MOUSE] <= 1;
			irq_state <= 1;
		end		
		if (irq[IRQ_SPI] & do_irq[IRQ_SPI]) begin
			// SPI MISO byte arrived
			spi_in_r <= spi_in;
			irq_r[IRQ_SPI] <= 1;
			irq_state <= 1;
		end		
		if (irq[IRQ_SPI1] & do_irq[IRQ_SPI1]) begin
			// SPI1 MISO byte arrived
			spi_in_r1 <= spi_in1;
			irq_r[IRQ_SPI1] <= 1;
			irq_state <= 1;
		end		
		if (irq[IRQ_DMA_1] & do_irq[IRQ_DMA_1]) begin
			// DMA channel 1 transfer finished -> DMA IRQ 1 triggered
			irq[IRQ_DMA_1] <= 0;
			irq_r[IRQ_DMA_1] <= 1;
			irq_state <= 1;
		end	 
	end

 case (state)
  INIT:
    if (init_wait == 0) begin
		rd <= 1'b0;
		wr <= 1'b0;
		rd_enable_o <= 1'b0;
		wr_enable_o <= 1'b0;
		
		pc <= 0;

		f <= 0;
		
		// enable interrupts 
		do_irq <= 16'b1111111111111111;
		irq_state <= 0;
	 
      state <= FETCH; 
	 end
	FETCH: begin
		`ifdef DEBUG
		$display("FETCH");
		`endif
		addr <= pc >> 1;
		next_state <= DECODE;
		state <= READ_DATA;
		ir <= 0;
		alu_uns <= 1'b0;

		if ((irq_state == 0) && (irq_r)) begin
			irq_state <= 1;
		end
		
		if (dma_start_rcv_1 && spi_ready && !spi_sent_ff_1 && dma_spi_received_1) begin
			spi_out <= 255; // send FF to initiate spi read
			spi_start <= 1;
			spi_sent_ff_1 <= 1;
			dma_spi_received_1 <= 0;
		end 
	end
	DECODE: begin
		`ifdef DEBUG
		$display("DECODE");
		`endif
		ir <= data_r;
		state <= EXECUTE;
		mc_count <= 0;
	end
	EXECUTE: begin
		`ifdef DEBUG
		$display("EXECUTE");
		`endif
		// pc already points to the next instruction, or argument
		// data_r register has the fetched data
		case (ir[3:0])
			// GROUP - 0 (NOP, MOV, IN, OUT, PUSH, POP, RET, IRET, SWAP, HALT)
			4'b0000: begin
				case (ir[7:4]) 
					// NOP
					4'b0000: begin
						`ifdef DEBUG
						$display("%2x: NOP", ir[3:0]);
						`endif
						state <= CHECK_IRQ;
						pc <= pc + 2;
					end // end of NOP
					4'b0001: begin
						// MOV.W regx, regy
						`ifdef DEBUG
						$display("%2x: MOV.W r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
						`endif
						regs[ir[11:8]] <= regs[ir[15:12]];
						state <= CHECK_IRQ;
						pc <= pc + 2;
					end // end of MOV regx, regy
					4'b0010: begin
						// MOV.S reg, xx
						`ifdef DEBUG
						$display("%2x: MOV.S r%-d, %4d",ir[3:0], (ir[11:8]), data_r);
						`endif
						case (mc_count)
							0: begin
								// get the xx
								addr <= (pc + 2) >> 1;
								pc <= pc + 2;
								mc_count <= 1;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							1: begin
								//regs[ir[11:8]] <= {{16{data_r[15]}}, data_r};
								regs[ir[11:8]] <= {16'd0, data_r};
								state <= CHECK_IRQ;
								pc <= pc + 2;
							end
						endcase
					end // end of MOV reg, xx
					// USED TO BE IN reg, [xx]
					4'b0011: begin
						`ifdef DEBUG
						$display("%2x: USED TO BE IN r%-d, [%4d]",ir[3:0], (ir[11:8]), data_r);
						`endif
					end // end of IN reg, [xx]
					// USED TO BE OUT [xx], reg
					4'b0100: begin
						`ifdef DEBUG
						$display("%2x: USED TO BE OUT [%4d], r%-d",ir[3:0], data_r, (ir[15:12]));
						`endif
					end // end of OUT [xx], reg
					// PUSH reg
					4'b0101: begin
					`ifdef DEBUG
					$display("%2x: PUSH r%-d", ir[3:0], (ir[11:8]));
					`endif
						case (mc_count)
							0: begin
								addr <= (regs[SP] - 2'd2) >> 1;
								data_to_write <= regs[ir[11:8]][15:0];
								// move sp to the next location
								regs[SP] <= regs[SP] - 2'd2;
								// next step
								mc_count <= 1;
								next_state <= EXECUTE;
								state <= WRITE_DATA;
							end
							1: begin
								addr <= (regs[SP] - 2'd2) >> 1;
								data_to_write <= regs[ir[11:8]][31:16];
								// move sp to the next location
								regs[SP] <= regs[SP] - 2'd2;
								// next step
								mc_count <= 2;
								next_state <= EXECUTE;
								state <= WRITE_DATA;
							end
							2: begin
								// step 2: initiate fetch
								state <= CHECK_IRQ;
								pc <= pc + 2;
							end
							default: begin
							end
					endcase
					end // end of PUSH reg
					// PUSH xx
					4'b0110: begin
						`ifdef DEBUG
						$display("%2x: PUSH %d", ir[3:0], data_r);
						`endif
						case (mc_count)
							0: begin
								mbr <= 0;
								// get the xx
								addr <= (pc + 2) >> 1;
								pc <= pc + 2;
								mc_count <= 1;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							1: begin
								mbr <= data_r; // upper 16 bits
								// get the xx
								addr <= (pc + 2) >> 1;
								pc <= pc + 2;
								mc_count <= 2;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							2: begin
								addr <= (regs[SP] - 2'd2) >> 1;
								data_to_write <= data_r; // lower 16 bits
								// move sp to the next location
								regs[SP] <= regs[SP] - 2'd2;
								// next step
								mc_count <= 3;
								next_state <= EXECUTE;
								state <= WRITE_DATA;
							end
							3: begin
								addr <= (regs[SP] - 2'd2) >> 1;
								data_to_write <= mbr;
								// move sp to the next location
								regs[SP] <= regs[SP] - 2'd2;
								// next step
								mc_count <= 4;
								next_state <= EXECUTE;
								state <= WRITE_DATA;
							end
							4: begin
								// step 2: initiate fetch
								state <= CHECK_IRQ;
								pc <= pc + 2;
							end
							default: begin
							end
						endcase // mc_count
					end // end of PUSH xx
					// POP reg
					4'b0111: begin
							`ifdef DEBUG
							$display("%2x: POP r%-d", ir[3:0], (ir[11:8]));
							`endif
						case (mc_count)
							0: begin
								// step 1: we try to read from the stack
								regs[SP] <= regs[SP] + 2'd2;
								addr <= (regs[SP]) >> 1;
								// move to the next step
								mc_count <= 1;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							1: begin
								regs[ir[11:8]][31:16] <= data_r;
								// step 1: we try to read from the stack
								regs[SP] <= regs[SP] + 2'd2;
								addr <= (regs[SP]) >> 1;
								// move to the next step
								mc_count <= 2;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							2: begin
								regs[ir[11:8]][15:0] <= data_r;
								// initiate fetch
								pc <= pc + 2;
								state <= CHECK_IRQ;
							end
							default: begin
							end
						endcase // mc_count
					end // end of POP reg
					// RET
					4'b1000: begin
							`ifdef DEBUG
							$display("%2x: RET", ir[3:0]);
							`endif
						case (mc_count)
							0: begin
								mbr <= 0;
								// step 1: we try to read the return value from the stack
								regs[SP] <= regs[SP] + 2'd2;
								addr <= (regs[SP]) >> 1;
								// move to the next step
								mc_count <= 1;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							1: begin
								mbr <= data_r;
								regs[SP] <= regs[SP] + 2'd2;
								addr <= (regs[SP]) >> 1;
								// move to the next step
								mc_count <= 2;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							2: begin
								pc[15:0] <= data_r;
								pc[31:16] <= mbr;
								state <= CHECK_IRQ;
							end
							default: begin
							end
						endcase
					end // end of RET
					// IRET
					4'b1001: begin
							`ifdef DEBUG
							$display("%2x: IRET", ir[3:0]);
							`endif
						case (mc_count)
							0: begin
								mbr <= 0;
								// step 1: we try to pop  the flags from the stack
								regs[SP_I] <= regs[SP_I] + 2'd2;
								addr <= (regs[SP_I]) >> 1;
								// move to the next step
								mc_count <= 1;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							1: begin
								f <= data_r[FLAGS_COUNT-1:0];

								regs[SP_I] <= regs[SP_I] + 2'd2;
								addr <= (regs[SP_I]) >> 1;
								// move to the next step
								mc_count <= 2;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							2: begin
								mbr <= data_r;
								
								// step 2: we try to pop the return value from the stack
								regs[SP_I] <= regs[SP_I] + 2'd2;
								addr <= (regs[SP_I]) >> 1;
								// move to the next step
								mc_count <= 3;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							3: begin
								// jump to the location from stack
								pc[15:0] <= data_r;
								pc[31:16] <= mbr;
								state <= CHECK_IRQ;
							end
							default: begin
							end
						endcase
					end // end of IRET
					// SWAP regx, regy
					4'b1010: begin
						`ifdef DEBUG
						$display("%2x: SWAP r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
						`endif
						regs[ir[11:8]]  <= regs[ir[15:12]];
						regs[ir[15:12]] <= regs[ir[11:8]];
						state <= CHECK_IRQ;
						pc <= pc + 2;
					end // END OF SWAP
					4'b1011: begin
						// MOV.W reg, xx
						`ifdef DEBUG
						$display("%2x: MOV.W r%-d, %4d",ir[3:0], (ir[11:8]), data_r);
						`endif
						case (mc_count)
							0: begin
								mbr <= 0;
								// get the xx
								addr <= (pc + 2) >> 1;
								pc <= pc + 2;
								mc_count <= 1;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							1: begin
								mbr[31:16] <= data_r;
								addr <= (pc + 2) >> 1;
								pc <= pc + 2;
								mc_count <= 2;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							2: begin
								regs[ir[11:8]] <= mbr + data_r;
								state <= CHECK_IRQ;
								pc <= pc + 2;
							end
						endcase
					end // end of MOV.W reg, xx
					4'b1100: begin
						// IRQ irq_no, 1 or 0
						`ifdef DEBUG
						$display("%2x: IRQ %4d, %1d",ir[3:0], ir[15:12], ir[8]);
						`endif
						do_irq[ir[15:12]] <= ir[8];
						state <= CHECK_IRQ;
						pc <= pc + 2;
					end // end of IRQ
					4'b1101: begin
						// MOV.B reg, xx
						`ifdef DEBUG
						$display("%2x: MOV.B r%-d, %4d",ir[3:0], (ir[11:8]), data_r);
						`endif
						case (mc_count)
							0: begin
								mbr <= 0;
								// get the xx
								addr <= (pc + 2) >> 1;
								pc <= pc + 2;
								mc_count <= 1;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							1: begin
								regs[ir[11:8]] <= {24'd0, data_r[7:0]};
//								regs[ir[11:8]] <= {{24{data_r[7]}}, data_r[7:0]};
								state <= CHECK_IRQ;
								pc <= pc + 2;
							end
						endcase
					end // end of MOV.B reg, xx
					4'b1110: begin
						// MOV.W regx, regy + xx
						/*
						`ifdef DEBUG
						$display("%2x: MOV.W r%-d, %4d",ir[3:0], (ir[11:8]), data_r);
						`endif
						case (mc_count)
							0: begin
								mbr <= 0;
								// get the xx
								addr <= (pc + 2) >> 1;
								pc <= pc + 2;
								mc_count <= 1;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							1: begin
								mbr[31:16] <= data_r;
								addr <= (pc + 2) >> 1;
								pc <= pc + 2;
								mc_count <= 2;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							2: begin
								regs[ir[11:8]] <= mbr + data_r + regs[ir[15:12]];
								state <= CHECK_IRQ;
								pc <= pc + 2;
							end
						endcase
						*/
					end // end of MOV.W regx, regy + xx
					// HALT
					4'b1111: begin
						`ifdef DEBUG
						$display("HALT");
						`endif
						state <= CHECK_IRQ;
					end
					default: begin
						// unknown instruction in this group
					end
				endcase
			end // end of GROUP 0
			// GROUP - 1 (JUMP)
			4'b0001: begin 
				if (ir[7:4] == 4'b1111) begin
					// Jump to the registar content address
					`ifdef DEBUG
					$display("%2x: JR r%-d", ir[3:0], ir[15:12]);
					`endif
					pc <= regs[ir[15:12]];
					state <= CHECK_IRQ;
				end // end of JR
				else begin
					case (mc_count)
						0: begin
							mbr <= 0;
							`ifdef DEBUG
							$display("JUMP SECTION");
							`endif
							// read the xx
							addr <= (pc + 2) >> 1;
							pc <= pc + 2;
							mc_count <= 1;
							next_state <= EXECUTE;
							state <= READ_DATA;
						end
						1: begin
							mbr[31:16] <= data_r;
							addr <= (pc + 2) >> 1;
							pc <= pc + 4;
							next_state <= EXECUTE;
							state <= READ_DATA;
							mc_count <= 2;
						end
						2: begin
							case (ir[7:4])
								// J xx
								4'b0000: begin
								`ifdef DEBUG
								$display("%2x: J %x", ir[3:0], data_r);
								`endif
									pc <= mbr + data_r;
								end // end of J xx
								// JZ xx
								4'b0001: begin
								`ifdef DEBUG
								$display("%2x: JZ %x", ir[3:0], data_r);
								`endif
									if (f[ZERO] == 1) begin
										pc <= mbr + data_r;
									end
								end // end of JZ xx
								// JNZ xx
								4'b0010: begin
								`ifdef DEBUG
								$display("%2x: JNZ %x", ir[3:0], data_r);
								`endif
									if (f[ZERO] == 0) begin
										pc <= mbr + data_r;
									end
								end // end of JNZ xx
								// JC xx
								4'b0011: begin
								`ifdef DEBUG
								$display("%2x: JC %x", ir[3:0], data_r);
								`endif
									if (f[CARRY] == 1) begin
										pc <= mbr + data_r;
									end
								end // end of JC xx
								// JNC xx
								4'b0100: begin
								`ifdef DEBUG
								$display("%2x: JNC %x", ir[3:0], data_r);
								`endif
									if (f[CARRY] == 0) begin
										pc <= mbr + data_r;
									end
								end // end of JNC xx
								// JO xx
								4'b0101: begin
								`ifdef DEBUG
								$display("%2x: JO %x", ir[3:0], data_r);
								`endif
									if (f[OVERFLOW] == 1) begin
										pc <= mbr + data_r;
									end
								end // end of JO xx
								// JNO xx
								4'b0110: begin
								`ifdef DEBUG
								$display("%2x: JNO %x", ir[3:0], data_r);
								`endif
									if (f[OVERFLOW] == 0) begin
										pc <= mbr + data_r;
									end
								end // end of JNO xx
								// JP xx (JGE xx)
								4'b0111: begin
								`ifdef DEBUG
								$display("%2x: JP(JGE) %x", ir[3:0], data_r);
								`endif
									if (f[POSITIVE] == 1) begin
										pc <= mbr + data_r;
									end
								end // end of JP xx (JGE)
								// JNP xx (JS xx)
								4'b1000: begin
								`ifdef DEBUG
								$display("%2x: JNP(JS) %x", ir[3:0], data_r);
								`endif
									if (f[POSITIVE] == 0) begin
										pc <= mbr + data_r;
									end
								end // end of JNP xx (JS xx)
								// JG xx
								4'b1001: begin
								`ifdef DEBUG
								$display("%2x: JG %x", ir[3:0], data_r);
								`endif
									if (f[POSITIVE] == 1 && f[ZERO] == 0) begin
										pc <= mbr + data_r;
									end
								end // end of JG xx
								// JSE xx
								4'b1010: begin
								`ifdef DEBUG
								$display("%2x: JSE %x", ir[3:0], data_r);
								`endif
									if (f[POSITIVE] == 0 || f[ZERO] == 1) begin
										pc <= mbr + data_r;
									end
								end // end of JSE xx
								// JGS xx (JG signed)
								4'b1011: begin
								`ifdef DEBUG
								$display("%2x: JGS %x", ir[3:0], data_r);
								`endif
									if ((!(f[CARRY] ^ f[OVERFLOW])) && (f[ZERO] == 0)) begin
										pc <= mbr + data_r;
									end
								end // end of JGS xx
								// JGES xx (JGE signed)
								4'b1100: begin
								`ifdef DEBUG
								$display("%2x: JGES %x", ir[3:0], data_r);
								`endif
									if (!(f[CARRY] ^ f[OVERFLOW])) begin
										pc <= mbr + data_r;
									end
								end // end of JGES xx
								// JSS xx (JS signed)
								4'b1101: begin
								`ifdef DEBUG
								$display("%2x: JSS %x", ir[3:0], data_r);
								`endif
									if (f[CARRY] ^ f[OVERFLOW]) begin
										pc <= mbr + data_r;
									end
								end // end of JSS xx
								// JSES xx (JSE signed)
								4'b1110: begin
								`ifdef DEBUG
								$display("%2x: JGS %x", ir[3:0], data_r);
								`endif
									if (((f[CARRY] ^ f[OVERFLOW])) || (f[ZERO] == 1)) begin
										pc <= mbr + data_r;
									end
								end // end of JSES xx
								default: begin
								end
							endcase	
							mc_count <= 3;
						end
						3: begin
							state <= CHECK_IRQ;
						end
					endcase
				end
			end // end of GROUP 1
			// GROUP - 2 (CALL)
			4'b0010: begin 
				case (mc_count)
					0: begin
						mbr <= 0;
						// push the return address to the stack
						addr <= (regs[SP] - 2'd2) >> 1;
						regs[SP] <= regs[SP] - 2;
						// the return address is in the pc + 4 
						if (ir[7:4] == 4'b1111) begin
							data_to_write <= (pc + 2) & 16'hFFFF;
						end
						else begin
							data_to_write <= (pc + 6) & 16'hFFFF;
						end
						mc_count <= 1;
						next_state <= EXECUTE;
						state <= WRITE_DATA;
					end
					1: begin
						addr <= (regs[SP] - 2'd2) >> 1;
						regs[SP] <= regs[SP] - 2;
						// the return address is in the pc + 4 
						if (ir[7:4] == 4'b1111) begin
							data_to_write <= (pc + 2) >> 16;
						end
						else begin
							data_to_write <= (pc + 6) >> 16;
						end
						mc_count <= 2;
						next_state <= EXECUTE;
						state <= WRITE_DATA;
					end
					2: begin
						if (ir[7:4] == 4'b1111) begin
							// Jump to the registar content address
							`ifdef DEBUG
							$display("%2x: CALLR r%-d", ir[3:0], ir[15:12]);
							`endif
							pc <= regs[ir[15:12]];
							state <= CHECK_IRQ;
						end // end of CALLR
						else begin
							// obtain xx
							addr <= (pc + 2) >> 1;
							pc <= pc + 2;
							mc_count <= 3;
							next_state <= EXECUTE;
							state <= READ_DATA;
						end
					end
					3: begin
						mbr[31:16] <= data_r;
						addr <= (pc + 2) >> 1;
						pc <= pc + 4;
						mc_count <= 4;
						next_state <= EXECUTE;
						state <= READ_DATA;
					end
					4: begin
						case (ir[7:4])
							// CALL xx
							4'b0000: begin
							`ifdef DEBUG
							$display("%2x: CALL %x", ir[3:0], data_r);
							`endif
								pc <= mbr + data_r;
							end
							// CALLZ xx
							4'b0001: begin
								if (f[ZERO] == 1) begin
									pc <= mbr + data_r;
								end
								else begin
									// qick&dirty
									regs[SP] <= regs[SP] + 4;
								end
							end
							// CALLNZ xx
							4'b0010: begin
								if (f[ZERO] == 0) begin
									pc <= mbr + data_r;
								end
								else begin
									// qick&dirty
									regs[SP] <= regs[SP] + 4;
								end
							end
						default: begin
						end
						endcase
						mc_count <= 5;
					end
					5: begin
						state <= CHECK_IRQ;
					end
				endcase
			end // end of GROUP 2
			// GROUP - 3 (LOAD, STORE)
			4'b0011, 4'b1100: begin
				case (ir[7:4]) 
					// LD.W regx, [regy]; LD.S regx, [regy]
					4'b0000: begin
						`ifdef DEBUG
							if (ir[3] == 1) 
								$display("%2x: LD.W r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
							else
								$display("%2x: LD.S r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
						`endif
						case (mc_count)
							0: begin
								mbr <= 0;
								// step 1: we try to read memory from the regy address
								addr <= regs[ir[15:12]] >> 1;
								if (ir[3] == 1)
									// LD.W regx, [regy]
									mc_count <= 1;
								else	
									// LD.S regx, [regy]
									mc_count <= 3;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							1: begin
								// LD.W regx, [regy]
								mbr[31:16] <= data_r;
								// step 1: we try to read memory from the regy address
								addr <= addr + 1;
								mc_count <= 2;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							2: begin
								// LD.W regx, [regy]
								// step 2: we get the memory content from the data bus and put it in the regx
								regs[ir[11:8]] <= (mbr + data_r);
								// pc already points to the next instruction
								pc <= pc + 2;
								state <= CHECK_IRQ;
							end
							3: begin
								// LD.S regx, [regy]
								// step 2: we get the memory content from the data bus and put it in the regx
								regs[ir[11:8]] <= data_r; //{{16{data_r[15]}}, data_r};
								// pc already points to the next instruction
								pc <= pc + 2;
								state <= CHECK_IRQ;
							end
							default: begin
							end
						endcase // mc_count
					end	// end of LD regx, [regy]
					// LD.W reg, [xx]; LD.W regx, [regy + xx], LD.S reg, [xx]; LD.S regx, [regy + xx]
					4'b0001, 4'b0010: begin
						`ifdef DEBUG
							if (ir[4] == 1) begin
								if (ir[3] == 1) 
									$display("%2x: LD.W r%-d, [%x]", ir[3:0], (ir[11:8]), data_r);
								else
									$display("%2x: LD.S r%-d, [%x]", ir[3:0], (ir[11:8]), data_r);
							end 
							else begin
								if (ir[3] == 1) 
									$display("%2x: LD.W r%-d, [r%-d + %x]", ir[3:0], (ir[11:8]), (ir[15:12]), data_r);
								else
									$display("%2x: LD.S r%-d, [r%-d + %x]", ir[3:0], (ir[11:8]), (ir[15:12]), data_r);
							end
						`endif
						case (mc_count)
							0: begin
								mbr <= 0;
								// step 0: obtain the xx
								addr <= (pc + 2)>> 1;
								pc <= pc + 2;
								mc_count <= 1;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							1: begin
								mbr[31:16] <= data_r;
								addr <= (pc + 2)>> 1;
								pc <= pc + 2;
								mc_count <= 2;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							2: begin
								if (ir[4] == 1) begin
									// LD.W reg, [xx]; LD.S reg, [xx]
									// step 1: we try to read memory from the xx address
									addr <= (mbr + data_r) >> 1;
								end
								else begin
									// LD.W regx, [regy + xx];  LD.S regx, [regy + xx]
									// step 1: we try to read memory from the (regy + xx) address
									addr <= (regs[ir[15:12]] + (mbr + data_r)) >> 1;
								end
								mc_count <= 3;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							3: begin
								// step 2: we get the memory content from the data bus and put it in the regx
								if (ir[3] == 0) begin
									// LD.S regx, [xx], LD.S regx, [regy + xx]
									regs[ir[11:8]] <=  data_r; 
									// pc points to the next instruction
									state <= CHECK_IRQ;
									pc <= pc + 2;
								end 
								else begin
									// LD.W regx, [xx], LD.W regx, [regy + xx]
									regs[ir[11:8]][31:16] <=  data_r; 
									addr <= addr + 1;
									mc_count <= 4;
									next_state <= EXECUTE;
									state <= READ_DATA;
								end
							end
							4: begin
								// LD.W regx, [xx], LD.W regx, [regy + xx]
								regs[ir[11:8]][15:0] <=  data_r; 
								// pc points to the next instruction
								state <= CHECK_IRQ;
								pc <= pc + 2;
							end
							default: begin
							end
						endcase // mc_count
					end	// end of LD regx, [xx]
					// LD.B regx, [regy], LD.B regx, [xx], LD.B regx, [regy + xx]
					4'b0011, 4'b0100, 4'b0101: begin
							`ifdef DEBUG
								case (ir[7:4])
								3:	$display("%2x: LD.B r%-d, [r%-d]", ir[3:0], (ir[11:8]), (ir[15:12]));
								4:	$display("%2x: LD.B r%-d, [%-d]", ir[3:0], (ir[11:8]), data_r);
								5:	$display("%2x: LD.B r%-d, [r%-d + %-d]", ir[3:0], (ir[11:8]), (ir[15:12]), data_r);
								endcase
							`endif
						case (mc_count)
							0: begin
								case (ir[7:4])
									3: begin
										// step 1: we try to read memory from the regy address
										addr <= regs[ir[15:12]] >> 1;
										mbr_e <= regs[ir[15:12]];
										next_state <= EXECUTE;
										state <= READ_DATA;
										mc_count <= 3;
									end
									4, 5: begin
										// LD.B regx, [xx], LD.B regx, [regy + xx]
										mbr <= 0;
										// step 0: obtain the xx
										addr <= (pc + 2)>> 1;
										pc <= pc + 2;
										mc_count <= 1;
										next_state <= EXECUTE;
										state <= READ_DATA;
									end
								endcase
							end
							1: begin
								mbr[31:16] <= data_r;
								addr <= (pc + 2)>> 1;
								pc <= pc + 2;
								mc_count <= 2;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							2: begin
								if (ir[7:4] == 4) begin
									// LD.B regx, [xx]
									addr <= (mbr + data_r) >> 1;
									mbr_e <= (mbr + data_r);
								end else begin
									// LD.B regx, [regy + xx]
									addr <= (regs[ir[15:12]] + mbr + data_r)>> 1;
									mbr_e <= (regs[ir[15:12]] + mbr + data_r);
								end
								mc_count <= 3;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							3: begin
									// step 2: we get the memory content from the data bus and put it in the regx
									if (mbr_e[0] == 1) begin
										// odd address
										regs[ir[11:8]] <= {24'd0, data_r[7:0]};
									end
									else begin
										// even address
										regs[ir[11:8]] <= {24'd0, data_r[15:8]};
									end
									// pc already points to the next instruction
									pc <= pc + 2;
									state <= CHECK_IRQ;
								end
							default: begin
							end
						endcase // mc_count
					end	// end of LD.B regx, [regy]
					// ST.W [regy], regx; ST.S [regy], regx
					4'b1000: begin
						`ifdef DEBUG
						if (ir[3] == 1)
							$display("%2x: ST.W [r%-d], r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
						else
							$display("%2x: ST.S [r%-d], r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
						`endif
						case (mc_count) 
							0: begin
								// step 1: we try to write into the memory defined by the regy 
								// put regy to the addr
								addr <= regs[ir[11:8]] >> 1;
								if (ir[3] == 1) begin
									// ST.W [regy], regx
									data_to_write <= regs[ir[15:12]][31:16];
									mc_count <= 1;
								end
								else begin
									// ST.S [regy], regx
									data_to_write <= regs[ir[15:12]];
									mc_count <= 2;
								end
								next_state <= EXECUTE;
								state <= WRITE_DATA;
							end
							1: begin
								// // ST.W [regy], regx
								addr <= addr + 1;
								data_to_write <= regs[ir[15:12]][15:0];
								mc_count <= 2;
								next_state <= EXECUTE;
								state <= WRITE_DATA;
							end
							2: begin
								pc <= pc + 2;
								state <= CHECK_IRQ;
							end
						endcase
					end	 // end of ST [regy], regx		
					// ST.W [xx], reg; ST.W [regx + xx], regy; ST.S [xx], reg; ST.S [regx + xx], regy
					4'b1001, 4'b1010: begin
						`ifdef DEBUG
							if (ir[4] == 1) begin 
								if (ir[3] == 1)
									$display("%2x: ST.W [%x], r%-d", ir[3:0], data_r, (ir[11:8]));
								else
									$display("%2x: ST.S [%x], r%-d", ir[3:0], data_r, (ir[11:8]));
							end
							else begin
								if (ir[3] == 1)
									$display("%2x: ST.W [r%-d + %x], r%d", ir[3:0], (ir[11:8]), data_r, (ir[15:12]));
								else
									$display("%2x: ST.S [r%-d + %x], r%d", ir[3:0], (ir[11:8]), data_r, (ir[15:12]));
							end
						`endif
						case (mc_count) 
							0: begin
								mbr <= 0;
								// read the xx 
								addr <= (pc + 2)>> 1;
								pc <= pc + 2;
								mc_count <= 1;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							1: begin
								mbr[31:16] <= data_r;
								addr <= (pc + 2)>> 1;
								pc <= pc + 2;
								mc_count <= 2;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							2: begin
								if (ir[4] == 1) begin
									// ST.W [xx], reg; ST.S [xx], reg
									addr <= (mbr + data_r) >> 1;
								end
								else begin
									// ST.W [regx + xx], regy; ST.S [regx + xx], regy
									addr <= (mbr + regs[ir[11:8]] + data_r) >> 1;
								end
								if (ir[3] == 0) begin
									// ST.S [xx], reg; ST.S [regx + xx], regy
									data_to_write <= regs[ir[15:12]];
									mc_count <= 4;
								end
								else begin
									// ST.W [xx], reg; ST.W [regx + xx], regy
									data_to_write <= regs[ir[15:12]][31:16];
									mc_count <= 3;
								end
								next_state <= EXECUTE;
								state <= WRITE_DATA;
							end
							3: begin
								// ST.W [xx], reg; ST.W [regx + xx], regy
								addr <= addr + 1;
								data_to_write <= regs[ir[15:12]][15:0];
								mc_count <= 4;
								next_state <= EXECUTE;
								state <= WRITE_DATA;
							end
							4: begin
								state <= CHECK_IRQ;
								pc <= pc + 2;
							end
						endcase
					end // end of ST [xx], reg; ST [regy + xx], regx						
					// ST.B [regy], regx
					4'b1011: begin
						`ifdef DEBUG
						$display("%2x: ST.B [r%-d], r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
						`endif
						case (mc_count)
							0: begin
								// step 1: we read the destination memory content
								// put regy to the addr
								addr <= regs[ir[11:8]] >> 1;
								mc_count <= 1;	
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							1: begin
								// step 2: we try to write into the memory defined by the regy 
								// put regy to the addr
								addr <= regs[ir[11:8]] >> 1;
								if (regs[ir[11:8]][0] == 1'b1) begin
									data_to_write <= {data_r[15:8], regs[ir[15:12]][7:0]};
								end
								else begin
									data_to_write <= {regs[ir[15:12]][7:0], data_r[7:0]};
								end
								mc_count <= 2;
								next_state <= EXECUTE;
								state <= WRITE_DATA;
							end
							2: begin
								// step 2: initiate fetch
								pc <= pc + 2;
								state <= CHECK_IRQ;
							end
							default: begin
							end
						endcase // mc_count
					end	// end of ST.B [regy], regx
					// ST.B [xx], reg; ST.B [regx + xx], regy
					4'b1100, 4'b1101: begin
						`ifdef DEBUG
							case (ir[7:4]) 
							12: $display("%2x: ST.B [%x], r%-d", ir[3:0], data_r, (ir[11:8]));
							13: $display("%2x: ST.B [r%-d + %x], r%d", ir[3:0], (ir[11:8]), data_r, (ir[15:12]));
							endcase
						`endif
						case (mc_count)
							0: begin
								mbr <= 0;
								// read the xx 
								addr <= (pc + 2)>> 1;
								pc <= pc + 2;
								mc_count <= 1;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							1: begin
								mbr[31:16] <= data_r;
								addr <= (pc + 2)>> 1;
								pc <= pc + 2;
								mc_count <= 2;
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							2: begin
								// step 1: we read the destination memory content
								if (ir[4] == 0) begin
									// ST.B [xx], reg
									// put xx to the addr
									addr <= (mbr + data_r) >> 1;
									mbr_e <= mbr + data_r;
								end
								else begin
									// ST.B [regx + xx], regy
									// put regy to the addr
									addr <= (mbr + data_r + regs[ir[11:8]]) >> 1;
									mbr_e <= mbr + data_r + regs[ir[11:8]];
								end
								mc_count <= 3;	
								next_state <= EXECUTE;
								state <= READ_DATA;
							end
							3: begin
								// step 2: we try to write into the memory defined by the regy 
								// put regy to the addr
								addr <= mbr_e >> 1;
								if (mbr_e[0] == 1'b1) begin
									data_to_write <= {data_r[15:8], regs[ir[15:12]][7:0]};
								end
								else begin
									data_to_write <= {regs[ir[15:12]][7:0], data_r[7:0]};
								end
								mc_count <= 4;
								next_state <= EXECUTE;
								state <= WRITE_DATA;
							end
							4: begin
								// step 3: initiate fetch
								state <= CHECK_IRQ;
								pc <= pc + 2;
							end
							default: begin
							end
						endcase // mc_count
					end	// end of ST.B [regy], regx
					default: begin
					end
				endcase
			end // end of GROUP 3
			// GROUP - 4, 5, 6, 7, 8 (ADD.W, SUB.W, AND.W, OR.W, XOR.W, NEG.W, SHL.W, SHR.W, MUL.W, DIV.W)
			4'b0100, 4'b0101, 4'b0110, 4'b0111, 4'b1000: begin
				case (ir[7:4]) 
					// ADD.W/AND.W/XOR.W/SHL.W/MUL.W regx, regy; ADD.S/AND.S/XOR.S/SHL.S/MUL.S regx, xx
					4'b0000, 4'b0001: begin
						`ifdef DEBUG
							if (ir[4] == 0) begin
								case (ir[3:0])
								4: $display("%2x: ADD.W r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								5: $display("%2x: AND.W r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								6: $display("%2x: XOR.W r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								7: $display("%2x: SHL.W r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								8: $display("%2x: MUL.W r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
							end
							else begin
								case (ir[3:0])
								4: $display("%2x: ADD.S r%-d, %-d", ir[3:0], (ir[11:8]), data_r);
								5: $display("%2x: AND.S r%-d, %-d", ir[3:0], (ir[11:8]), data_r);
								6: $display("%2x: XOR.S r%-d, %-d", ir[3:0], (ir[11:8]), data_r);
								7: $display("%2x: SHL.S r%-d, %-d", ir[3:0], (ir[11:8]), data_r);
								8: $display("%2x: MUL.S r%-d, %-d", ir[3:0], (ir[11:8]), data_r);
								default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
							end
						`endif
						case (mc_count)
							0: begin
								if (ir[4] == 1) begin
									// ALU.X regx, xx
									// read the xx 
									addr <= (pc + 2) >> 1;
									pc <= pc + 2;
									next_state <= EXECUTE;
									state <= READ_DATA;
								end
								mc_count <= 1;
							end
							1: begin
								case (ir[3:0])
									4: alu_op <= ALU_ADD;
									5: alu_op <= ALU_AND;
									6: alu_op <= ALU_XOR;
									7: alu_op <= ALU_SHL;
									8: alu_op <= ALU_MUL;
									default: alu_op <= 0;
								endcase
								alu_a <= regs[ir[11:8]];
								if (ir[4] == 0)
									// ALU.X regx, regy
									alu_b <= regs[ir[15:12]];
								else
									// ALU.S regx, xx
									alu_b <= data_r;
								alu_start <= 1;
								mc_count <= 2;
							end
							2: begin
								alu_start <= 0;
								mc_count <= 3;
							end
							3: begin
								regs[ir[11:8]] <= alu_res;
								f[FLAGS_COUNT-1:0] <= f_from_alu;
								regs[H] <= alu_high;
								state <= CHECK_IRQ;
								pc <= pc + 2;
							end
							default: begin
							end
						endcase
					end // end of ADD.W/AND.W/XOR.W/SHL.W/MUL.W regx, regy
					// SUB.W/OR.W/NEG.W/SHR.W/DIV.W regx, regy; SUB.S/OR.S/NEG.S/SHR.S/DIV.S regx, xx
					4'b1000, 4'b1001: begin
						`ifdef DEBUG
							if (ir[4] == 0) begin
								case (ir[3:0])
									4: $display("%2x: SUB.W r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
									5: $display("%2x: OR.W r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
									6: $display("%2x: NEG.W r%-d", ir[3:0], (ir[15:12]));
									7: $display("%2x: SHR.W r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
									8: $display("%2x: DIV.W r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
							end
							else begin
								case (ir[3:0])
									4: $display("%2x: SUB.S r%-d, %-d", ir[3:0], (ir[11:8]), data_r);
									5: $display("%2x: OR.S r%-d, %-d", ir[3:0], (ir[11:8]), data_r);
									7: $display("%2x: SHR.S r%-d, %-d", ir[3:0], (ir[11:8]), data_r);
									8: $display("%2x: DIV.S r%-d, %-d", ir[3:0], (ir[11:8]), data_r);
									default: $display("WRONG OPCODE: %02d, %02x", ir[3:0], ir[3:0]);
								endcase
							end
						`endif
						case (mc_count)
							0: begin
								if (ir[4] == 1) begin
									// ALU.W regx, xx
									// read the xx 
									addr <= (pc + 2) >> 1;
									pc <= pc + 2;
									next_state <= EXECUTE;
									state <= READ_DATA;
								end
								mc_count <= 1;
							end
							1: begin
								case (ir[3:0])
									4: alu_op <= ALU_SUB;
									5: alu_op <= ALU_OR;
									6: alu_op <= ALU_NEG;
									7: alu_op <= ALU_SHR;
									8: alu_op <= ALU_DIV;
									default: alu_op <= 0;
								endcase
								alu_a <= regs[ir[11:8]];
								if (ir[4] == 0)
									// ALU.W regx, regy
									alu_b <= regs[ir[15:12]];
								else
									// ALU.S regx, xx
									alu_b <= data_r;
									
								if (ir[3:0] == 8'd8) begin
									// DIV only
									start_div <= 1'b1;
								end
								alu_start <= 1;
								mc_count <= 2;
							end
							2: begin
								alu_start <= 0;
								mc_count <= 3;
							end
							3: begin
								if (ir[3:0] == 8'd8) begin
									// DIV only
									start_div <= 1'b0;
									if (div_finished) begin
										regs[ir[11:8]] <= alu_res;
										f[FLAGS_COUNT-1:0] <= f_from_alu;
										regs[H] <= alu_high;
										pc <= pc + 2;
										state <= CHECK_IRQ;
									end
								end
								else begin
									if (ir[3:0] == 6) // NEG
										regs[ir[15:12]] <= alu_res;
									else
										regs[ir[11:8]] <= alu_res;
									f[FLAGS_COUNT-1:0] <= f_from_alu;
									pc <= pc + 2;
									state <= CHECK_IRQ;
								end
							end
							default: begin
							end
						endcase
					end // end of SUB.S/OR.S/NEG.S/SHR.S/DIV.S regx, regx
				endcase
			end
			// GROUP - 9 (INC, DEC)
			4'b1001: begin 
				case (ir[7:4])
					// INC/DEC reg; INC/DEC [reg]
					4'b0000, 4'b1000, 4'b0001, 4'b1001: begin
						`ifdef DEBUG
							if (ir[4] == 0) begin
								if (ir[7] == 0)
									$display("%2x: INC r%-d", ir[3:0], (ir[11:8]));
								else
									$display("%2x: DEC r%-d", ir[3:0], (ir[11:8]));
							end
							else begin
								if (ir[7] == 0)
									$display("%2x: INC [r%-d]", ir[3:0], (ir[11:8]));
								else
									$display("%2x: DEC [r%-d]", ir[3:0], (ir[11:8]));
							end
						`endif
						case (mc_count)
							0: begin
								if (ir[4] == 1) begin
									// read the [reg]
									addr <= regs[ir[11:8]] >> 1;
									next_state <= EXECUTE;
									state <= READ_DATA;
								end
								mc_count <= 1;
							end
							1: begin
								if (ir[7] == 0)
									alu_op <= ALU_ADD;
								else
									alu_op <= ALU_SUB;
								if (ir[4] == 0) 
									alu_a <= regs[ir[11:8]];
								else
									alu_a <= data_r;
								alu_b <= 1;
								
								alu_start <= 1;
								mc_count <= 2;
							end
							2: begin
								alu_start <= 0;
								mc_count <= 3;
							end
							3: begin
								if (ir[4] == 1) begin
									// write alu_res into [reg]
									addr <= regs[ir[11:8]] >> 1;
									data_to_write <= alu_res;
									next_state <= EXECUTE;
									state <= WRITE_DATA;
								end
								else begin
									regs[ir[11:8]] <= alu_res;
								end
								mc_count <= 4;
								f[FLAGS_COUNT-1:0] <= f_from_alu;
							end
							4: begin
								state <= CHECK_IRQ;
								pc <= pc + 2;
							end
							default: begin
							end
						endcase
					end // end of INC/DEC reg
				endcase
			end
			// GROUP - 10 (CMP/INV)
			4'b1010: begin 
				case (ir[7:4])
					// CMP regx, regy; CMP reg, xx
					4'b0000, 4'b0001: begin
						`ifdef DEBUG
							if (ir[4] == 0) begin
								$display("%2x: CMP r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
							end
							else begin
								$display("%2x: CMP r%-d, %-d", ir[3:0], (ir[11:8]), data_r);
							end
						`endif
						case (mc_count)
							0: begin
								if (ir[4] == 1) begin
									// read the xx 
									addr <= (pc + 2) >> 1;
									pc <= pc + 2;
									next_state <= EXECUTE;
									state <= READ_DATA;
								end
								mc_count <= 1;
							end
							1: begin
								alu_op <= ALU_SUB;
								alu_a <= regs[ir[11:8]];
								if (ir[4] == 0)
									alu_b <= regs[ir[15:12]];
								else
									alu_b <= data_r;
								
								alu_start <= 1;
								mc_count <= 2;
							end
							2: begin
								alu_start <= 0;
								mc_count <= 3;
							end
							3: begin
								f[FLAGS_COUNT-1:0] <= f_from_alu;
								pc <= pc + 2;
								state <= CHECK_IRQ;
							end
						endcase
					end // end of CMP regx, regy
					// INV reg
					4'b1000: begin
						`ifdef DEBUG
						$display("%2x: INV r%-d", ir[3:0], (ir[11:8]));
						`endif
						case (mc_count)
							0: begin
								alu_op <= ALU_INV;
								alu_a <= regs[ir[11:8]];
								alu_b <= 1;
								alu_start <= 1;
								mc_count <= 1;
							end
							1: begin
								alu_start <= 0;
								mc_count <= 2;
							end
							2: begin
								regs[ir[11:8]] <= alu_res;
								f[FLAGS_COUNT-1:0] <= f_from_alu;
								pc <= pc + 2;
								state <= CHECK_IRQ;
							end
							default: begin
							end
						endcase
					end // end of INV reg
				endcase
			end // end of GROUP 10
			
			// GROUP - 13, 14 (0x0d, 0x0e) (ALU.W REG, XX; ALU.B REG, XX)
			4'b1101, 4'b1110: begin 
				case (mc_count)
				0: begin
					mbr <= 0;
					// read the xx 
					addr <= pc + 2 >> 1;
					mbr_e <= pc + 2;
					pc <= pc + 2;
					next_state <= EXECUTE;
					state <= READ_DATA;
					if (ir[3:0] == 13) begin
						mc_count <= 1;
					end
					else begin
						mc_count <= 2;
					end
				end
				1: begin
					mbr[31:16] <= data_r;
					// read the xx 
					addr <= pc + 2 >> 1;
					pc <= pc + 2;
					next_state <= EXECUTE;
					state <= READ_DATA;
					mc_count <= 2;
				end
				2: begin
					case (ir[7:4])
						0: alu_op <= ALU_ADD;
						1: alu_op <= ALU_SUB;
						2: alu_op <= ALU_AND;
						3: alu_op <= ALU_OR;
						4: alu_op <= ALU_XOR;
						5: alu_op <= ALU_SHL;
						6: alu_op <= ALU_SHR;
						7: alu_op <= ALU_MUL;
						8: alu_op <= ALU_DIV;
						9: alu_op <= ALU_SUB;	// CMP.W REG, XX
						default: alu_op <= 0;
					endcase
					alu_a <= regs[ir[11:8]];
					if (ir[3:0] == 13) begin
						// ALU.W regx, xx
						alu_b <= mbr + data_r;
					end
					else begin
						// ALU.B regx, xx
						alu_b <= {{24{data_r[7]}}, data_r[7:0]};
					end
					alu_start <= 1;
					mc_count <= 3;
					if (ir[7:4] == 8'd8) begin
						// DIV only
						start_div <= 1'b1;
					end
				end
				3: begin
					alu_start <= 0;
					mc_count <= 4;
				end
				4: begin
					if (ir[7:4] == 8'd8) begin
						// DIV only
						start_div <= 1'b0;
						if (div_finished) begin
							regs[ir[11:8]] <= alu_res;
							f[FLAGS_COUNT-1:0] <= f_from_alu;
							regs[H] <= alu_high;
							pc <= pc + 2;
							state <= CHECK_IRQ;
						end
					end
					else begin
						if (ir[7:4] != 9) begin
							// EVERYTHING BUT CMP.W REG, XX
							regs[ir[11:8]] <= alu_res;
						end
						f[FLAGS_COUNT-1:0] <= f_from_alu;
						state <= CHECK_IRQ;
						pc <= pc + 2;
					end
				end
				endcase
			end // end of GROUP 13, 14
			
				// GROUP - 11 (0x0b) (FLOATING POINT GROUP , SEX, BLIT, INT GROUP)
				// 32 bits: SIGN, EXPONENT-8bit, MANTISSA-23bit
				// BIAS: 127
				4'b1011: begin 
					case (ir[7:4])
`ifdef FLOAT
						// FADD/FSUB regx, regy
						4'b0010, 4'b0011: begin
							`ifdef DEBUG
								if (ir[7:4] == 4'b0010)
									$display("%2x: FADD r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
								else
									$display("%2x: FSUB r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
							`endif
							case (mc_count)
								0: begin
									mc_count <= 1;
									tfe <= 23;
									if (ir[7:4] == 4'b0010) 
										fp_add_sub <= 1;
									else
										fp_add_sub <= 0;
									clk_en <= 1;
									ma <= regs[ir[11:8]];
									mb <= regs[ir[15:12]];
								end
								1: begin
									if (tfe == 0) begin
										mc_count <= 2;
										clk_en <= 0;
									end
									else begin
										tfe <= tfe - 1;
									end
								end
								2: begin
									regs[ir[11:8]] <= fadd_sub_res;
									state <= CHECK_IRQ;
									pc <= pc + 2;
								end
							endcase
						end // end of FADD regx, regy
						// FMUL regx, regy
						4'b0100: begin
							`ifdef DEBUG
							$display("%2x: FMUL r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
							`endif
							case (mc_count)
								0: begin
									mc_count <= 1;
									tfe <= 23;
									clk_en <= 1;
									ma <= regs[ir[11:8]];
									mb <= regs[ir[15:12]];
								end
								1: begin
									if (tfe == 0) begin
										mc_count <= 2;
										clk_en <= 0;
									end
									else begin
										tfe <= tfe - 1;
									end
								end
								2: begin
									regs[ir[11:8]] <= fmulres;
									state <= CHECK_IRQ;
									pc <= pc + 2;
								end
							endcase
						end   // end of FMUL regx, regy	
						// FDIV regx, regy
						4'b0101: begin
							`ifdef DEBUG
							$display("%2x: FDIV r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
							`endif
							case (mc_count)
								0: begin
									mc_count <= 1;
									tfe <= 23;
									clk_en <= 1;
									ma <= regs[ir[11:8]];
									mb <= regs[ir[15:12]];
								end
								1: begin
									if (tfe == 0) begin
										mc_count <= 2;
										clk_en <= 0;
									end
									else begin
										tfe <= tfe - 1;
									end
								end
								2: begin
									regs[ir[11:8]] <= fdivres;
									state <= CHECK_IRQ;
									pc <= pc + 2;
								end
							endcase
						end   // end of FDIV regx, regy							
`endif					
						// SEX.B regx, regy
						4'b0110: begin
							// SEX.B regx, regy
							`ifdef DEBUG
							$display("%2x: SEX.B r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
							`endif
							regs[ir[11:8]] <= {{24{regs[ir[15:12]][7]}}, regs[ir[15:12]][7:0]};
							state <= CHECK_IRQ;
							pc <= pc + 2;
						end // END OF SEX.B regx, regy
						// SEX.S regx, regy
						4'b0111: begin
							// SEX.S regx, regy
							`ifdef DEBUG
							$display("%2x: SEX.S r%-d, r%-d", ir[3:0], (ir[11:8]), (ir[15:12]));
							`endif
							regs[ir[11:8]] <= {{16{regs[ir[15:12]][15]}}, regs[ir[15:12]][15:0]};
							state <= CHECK_IRQ;
							pc <= pc + 2;
						end // END OF SEX.S regx, regy
						4'b1000: begin
							// BLIT (r1, r2, r3) - r1 - dst; r2 - src; r3 - count
							case (mc_count)
								0: begin
									addr <= regs[2] >> 1;
									regs[2] <= regs[2] + 2;
									regs[3] <= regs[3] - 2;
									mc_count <= 1;
									next_state <= EXECUTE;
									state <= READ_DATA;
								end
								1: begin
									addr <= regs[1] >> 1;
									data_to_write <= data_r;
									regs[1] <= regs[1] + 2;
									next_state <= EXECUTE;
									state <= WRITE_DATA;
									if (regs[3] <= 0) begin
										mc_count <= 2;
									end
									else 
										mc_count <= 0;
								end
								2: begin
									state <= CHECK_IRQ;
									pc <= pc + 2;
								end
							endcase
						end
						4'b1001: begin
							// PIX (r0, r1, r2, r3, r4, r5) - r0 - x; r1 - y; r2 - color; r3 - framebuffer address (1024 is for the real video mem); 
							// r4 - line width in bytes; r5 - buffer height
							// if(x < 0 || y < 0 || x >= bufferW * bufferPPS || y >= bufferH)
							//if ((regs[0] >= 0) && (regs[1] >= 0) && (regs[0] < (regs[4] << (vga_mode==1?1:3))) && regs[1] < regs[5]) begin
								case (mc_count)
									0: begin
										// location = (x >> 2)*2 + y * bufferW; p = backbuffer + location;
										addr <= (regs[3] + ((regs[0] >> (vga_mode == 1?2:4)) << 1) + (regs[1] * regs[4])) >> 1; 
										// xOffset = (x % 4) * 4; xOffsetMask = 0xF000 >> xOffset; pColor = (color<<(12-xOffset)) & xOffsetMask;
										mbr <= (regs[2] << ((vga_mode == 1?12:15)-((regs[0] & (vga_mode == 1?32'd3:32'd15)) << (vga_mode == 1?2:0)))) & ((vga_mode == 1?16'hF000:16'h8000) >> ((regs[0] & (vga_mode == 1?32'd3:32'd15)) << (vga_mode == 1?2:0)));
										// xOffsetMask = 0xF000 >> xOffset; 
										mbr_e <= ((vga_mode == 1?16'hF000:16'h8000) >> ((regs[0] & (vga_mode == 1?32'd3:32'd15)) << (vga_mode == 1?2:0))); 
										mc_count <= 1;
										next_state <= EXECUTE;
										state <= READ_DATA;
									end
									1: begin
										// bgColor = (~xOffsetMask) & *p; *p = pColor | bgColor;
										data_to_write <= (data_r & (~mbr_e)) | mbr;  
										mc_count <= 2;
										next_state <= EXECUTE;
										state <= WRITE_DATA;
									end
									2: begin
										state <= CHECK_IRQ;
										pc <= pc + 2;
									end
								endcase
							//end
						end
						// INT xx, SOFTWARE INTERRUPT
						4'b1111: begin
							/*
							`ifdef DEBUG
							$display("%2x: INT %d", ir[3:0], data_r);
							`endif
							case (mc_count)
								0: begin
									// get the lower 16 bits of xx
									addr <= (pc + 2) >> 1;
									pc <= pc + 2;
									mc_count <= 1;
									mbr_e <= pc + 6;  // remember the address of the next instruction (the return point from the interrupt)
									next_state <= EXECUTE;
									state <= READ_DATA;
								end
								1: begin
									mbr <= data_r;
									// push to the stack the return value
									addr <= (regs[SP_I]  - 2'd2) >> 1;
									regs[SP] <= regs[SP_I] - 2'd2;
									// the return value is in the mbr_e and it is already pointing to the next instruction
									data_to_write <= mbr_e[15:0];
									mc_count <= 2;
									next_state <= EXECUTE;
									state <= WRITE_DATA;
								end
								2: begin
									addr <= (regs[SP_I] - 2'd2) >> 1;
									regs[SP] <= regs[SP_I] - 2'd2;
									// the return value is in the mbr_e and it is already pointing to the next instruction
									data_to_write <= mbr_e[31:16];
									mc_count <= 3;
									next_state <= EXECUTE;
									state <= WRITE_DATA;
								end
								3: begin
									// push to the stack flags
									addr <= (regs[SP] - 2'd2) >> 1;
									regs[SP] <= regs[SP] - 2'd2;
									// back up the flags register
									data_to_write <= f;
									mc_count <= 4;
									next_state <= EXECUTE;
									state <= WRITE_DATA;
								end
								4: begin
									pc <= mbr;
									addr <= mbr >> 1;
									mc_count <= 0;
									state <= FETCH;
									ir <= 0;
								end
							endcase
							*/
						end   // end of INT xx
					endcase // end of case (ir[7:4])					
				end // end of FLOATING POINT GROUP & INT GROUP
			
		endcase
	end // EXECUTE state
	CHECK_IRQ: begin
		if (irq_state) begin 
			case (irq_state) 
				1: begin
					// first we try to read memory at the interrupt routine
					if (irq_r[IRQ_UART]) begin
						addr <= 16'd8;
						mbr <= 1;
						next_state <= CHECK_IRQ;
						state <= READ_DATA;		
						irq_state <= 2;			
					end
					else if (irq_r[IRQ_PS2]) begin
						addr <= 16'd12;
						mbr <= 2;
						next_state <= CHECK_IRQ;
						state <= READ_DATA;		
						irq_state <= 2;			
					end
					else if (irq_r[IRQ_SPI]) begin
						if (dma_count_1 && dma_start_rcv_1) begin
							// SPI byte arrived, but we are reading it using DMA, not using interrupts
							irq_state <= 6;
						end
						else begin
							addr <= 16'd28;
							mbr <= 3;
							next_state <= CHECK_IRQ;
							state <= READ_DATA;		
							irq_state <= 2;			
						end
					end
					else if (irq_r[IRQ_SPI1]) begin
						addr <= 16'd32;
						mbr <= 4;
						next_state <= CHECK_IRQ;
						state <= READ_DATA;		
						irq_state <= 2;			
					end
					else if (irq_r[IRQ_TIMER]) begin
						addr <= 16'd4;
						mbr <= 0;
						next_state <= CHECK_IRQ;
						state <= READ_DATA;		
						irq_state <= 2;			
					end
					else if (irq_r[IRQ_PS2_MOUSE]) begin
						addr <= 16'd36;
						mbr <= 5;
						next_state <= CHECK_IRQ;
						state <= READ_DATA;		
						irq_state <= 2;			
					end
					else if (irq_r[IRQ_DMA_1]) begin
						addr <= 16'd40;
						mbr <= 14;
						next_state <= CHECK_IRQ;
						state <= READ_DATA;		
						irq_state <= 2;			
					end
				end
				2: begin
					if (data_r == 0) begin
						// if it is zero, then we don't jump and we reset the interrupt
						case (mbr) 
							0: begin
								irq_r[IRQ_TIMER] <= 1'b0;
							end
							1: begin
								irq_r[IRQ_UART] <= 1'b0;
							end
							2: begin
								irq_r[IRQ_PS2] <= 1'b0;
							end
							3: begin
								irq_r[IRQ_SPI] <= 1'b0;
							end
							4: begin
								irq_r[IRQ_SPI1] <= 1'b0;
							end
							5: begin
								irq_r[IRQ_PS2_MOUSE] <= 1'b0;
							end
							14: begin
								irq_r[IRQ_DMA_1] <= 1'b0;
							end
						endcase
						irq_state <= 0;
						state <= FETCH;
						ir <= 0;
					end
					else begin
						irq_state <= 3;
					end
				end
				3: begin
					// push to the stack the return value
					addr <= (regs[SP_I]  - 2'd2) >> 1;
					regs[SP_I] <= regs[SP_I] - 2'd2;
					// the return value is in the pc and it is already pointing to the next instruction
					data_to_write <= pc[15:0];
					irq_state <= 4;
					next_state <= CHECK_IRQ;
					state <= WRITE_DATA;
				end
				4: begin
					addr <= (regs[SP_I] - 2'd2) >> 1;
					regs[SP_I] <= regs[SP_I] - 2'd2;
					// the return value is in the pc and it is already pointing to the next instruction
					data_to_write <= pc[31:16];
					irq_state <= 5;
					next_state <= CHECK_IRQ;
					state <= WRITE_DATA;
				end
				5: begin
					// push to the stack flags
					addr <= (regs[SP_I] - 2'd2) >> 1;
					regs[SP_I] <= regs[SP_I] - 2'd2;
					// back up the flags register
					data_to_write <= f;
					irq_state <= 6;
					next_state <= CHECK_IRQ;
					state <= WRITE_DATA;
				end
				6: begin
					if (irq_r[IRQ_TIMER]) begin
						// timer
						pc <= 16'd8;
						addr <= 16'd4;
						irq_r[IRQ_TIMER] <= 0;
						irq_state <= 0;
						state <= FETCH;
						ir <= 0;
					end 
					else if (irq_r[IRQ_UART]) begin
						// UART byte arrived
						pc <= 16'd16;
						addr <= 16'd8;
						irq_r[IRQ_UART] <= 0;
						irq_state <= 0;
						state <= FETCH;
						ir <= 0;
					end
					else if (irq_r[IRQ_PS2]) begin
						// PS/2 key pressed/released
						pc <= 16'd24;
						addr <= 16'd12;
						irq_r[IRQ_PS2] <= 0;
						irq_state <= 0;
						state <= FETCH;
						ir <= 0;
					end
					else if (irq_r[IRQ_PS2_MOUSE]) begin
						// PS/2 mouse byte arrived
						pc <= 16'd72;
						addr <= 16'd36;
						irq_r[IRQ_PS2_MOUSE] <= 0;
						irq_state <= 0;
						state <= FETCH;
						ir <= 0;
					end
					else if (irq_r[IRQ_SPI]) begin
						if (dma_count_1 && dma_start_rcv_1) begin
							if (dma_current_1[0] == 0) begin
								// even address
								dma_byte_1 <= spi_in_r;
								//dma_addr_1 <= dma_addr_1 + 1;
								dma_current_1 <= dma_current_1 + 1;

								irq_state <= 7;
							end
							else begin
								// odd address
								addr <= (dma_addr_1 + dma_current_1) >> 1;
								data_to_write <= (dma_byte_1 << 8) | spi_in_r;
								next_state <= CHECK_IRQ;
								state <= WRITE_DATA;
								
								//dma_addr_1 <= dma_addr_1 + 1;
								dma_current_1 <= dma_current_1 + 1;
								
								irq_state <= 7;
							end
						end
						else begin
							irq_r[IRQ_SPI] <= 0; 
							pc <= 16'd56;
							addr <= 16'd28;
							irq_state <= 0;
							state <= FETCH;
							ir <= 0;
						end
					end
					else if (irq_r[IRQ_SPI1]) begin
						// SPI1 byte received
						pc <= 16'd64;
						addr <= 16'd32;
						irq_r[IRQ_SPI1] <= 0;
						irq_state <= 0;
						state <= FETCH;
						ir <= 0;
					end
					else if (irq_r[IRQ_DMA_1]) begin
						// DMA channel 1 transfer finished
						pc <= 16'd80;
						addr <= 16'd40;
						irq_r[IRQ_DMA_1] <= 0;

						irq_state <= 0;
						state <= FETCH;
						ir <= 0;
					end					
				end
				7: begin
					// special case for DMA transfer
					irq_r[IRQ_SPI] <= 0; 
					dma_spi_received_1 <= 1;
					not_received_counter <= 0;
					irq_state <= 0;
					state <= FETCH;
					ir <= 0;
				end 
			endcase
		end
		else begin
			state <= FETCH;
		end
	end // end of CHECK_IRQ
	READ_DATA: begin
		if (dma_start_rcv_1 && spi_sent_ff_1) begin
			spi_start <= 0;
			spi_sent_ff_1 <= 0;
		end 
	
		if (addr[30] == 1'b1) begin
			// memory mapped IO
			case (addr & 32'h3FFFFFFF)
				PORT_MILLIS/2: begin	// milliseconds counted so far
					data_r <= millis_counter >> 16;
				end
				PORT_MILLIS/2 + 1: begin	// milliseconds counted so far
					data_r <= millis_counter & 16'hFFFF;
				end
				PORT_UART_RX_BYTE/2: begin    // UART RX DATA
					data_r <= {24'b0, rx_data_r};
				end
				PORT_UART_TX_BUSY/2: begin   // UART TX BUSY
					data_r <= {31'b0, tx_busy};
				end
				PORT_KEYBOARD/2: begin    // keyboard data
					data_r <= {24'b0, ps2_data_r};
				end
				PORT_MOUSE/2: begin    // mouse data
					data_r <= {24'b0, ps2_data_r_mouse};
				end
				PORT_SPI_IN/2: begin
					data_r <= {24'b0, spi_in_r};
				end
				PORT_SPI_OUT_BUSY/2: begin
					data_r <= {31'b0, ~spi_ready};
				end
				PORT_SPI1_IN/2: begin
					data_r <= {24'b0, spi_in_r1};
				end
				PORT_SPI1_OUT_BUSY/2: begin
					data_r <= {31'b0, ~spi_ready1};
				end
				PORT_DMA_ADDR_1/2: begin
					data_r <= dma_addr_1[31:16];
				end
				(PORT_DMA_ADDR_1/2) + 1: begin
					data_r <= dma_addr_1[15:0];
				end
				PORT_DMA_COUNT_1/2: begin
					data_r <= dma_count_1[31:16];
				end
				(PORT_DMA_COUNT_1/2) + 1: begin
					data_r <= dma_count_1[15:0];
				end 	
				PORT_MOUSE_STRUCT_ADDR/2: begin
					data_r <= mouse_struct_addr[31:16];
				end
				(PORT_MOUSE_STRUCT_ADDR/2) + 1: begin
					data_r <= mouse_struct_addr[15:0];
				end 				
			endcase // end of case(mbr)
			state <= next_state;
		end
		else begin
			addr_o <= addr;
			if (addr >= SDRAM_START_ADDR) begin
				`ifdef CACHE
				if (tag[addr[11:0]] == addr[23:12]) begin
					// cache hit (required data is in cache)
					data_r <= cl[addr[11:0]];
					state <= READ_WAIT + 1;
				end
				else begin
					// cache miss -> we need to read from SDRAM
				`endif
					rd_enable_o <= 1'b1;
					if (busy_i) begin
						state <= READ_WAIT;
					end
			`ifdef CACHE
				end
			`endif
			end
			else begin
				rd <= 1'b1;
				wr <= 1'b0;
				state <= READ_WAIT;
			end
		end
	end
	READ_WAIT: begin
		if (addr >= SDRAM_START_ADDR) begin
			`ifdef CACHE
			if (tag[addr[11:0]] == addr[23:12]) begin
				state <= READ_WAIT + 1;
			end
			else begin
			`endif
				rd_enable_o <= 1'b0;
				if (rd_ready_i) begin
					data_r <= rd_data_i;
					`ifdef CACHE
					// we store the fetched data into the cache
					cl[addr[11:0]] <= rd_data_i;
					// write tag
					tag[addr[11:0]] <= addr[23:12];
					`endif
					state <= READ_WAIT + 1;
				end
			`ifdef CACHE
			end
			`endif
		end
		else begin
			rd <= 1'b0;
			wr <= 1'b0;
			data_r <= data_i;
			state <= READ_WAIT + 1;
		end
	end
	READ_WAIT + 1: begin
		if (addr >= SDRAM_START_ADDR) begin
			//data_r <= rd_data_i;
			`ifdef CACHE
			state <= next_state;
			`else
			// NO CACHE
			state <= next_state;
			`endif
		end
		else begin
			//data_r <= data_i;
			state <= next_state;
		end
	end
	WRITE_DATA: begin
		if (addr[30] == 1'b1) begin
			// Memory mapped IO
			case (addr & 32'h3FFFFFFF)
				PORT_TIMER/2: begin
					// number of milliseconds to expire to cause IRQ 0
					timer <= data_to_write;
				end
				PORT_UART_TX_SEND_BYTE/2: begin  // UART TX data 
					tx_data <= data_to_write;
					tx_send <= 1'b1;
				end
				PORT_VIDEO_MODE/2: begin  // graphics mode: 0 - text; 1 - 320x240 8 colors; 2 - 640x480 two colors
					vga_mode <= data_to_write;
				end
				VGA_TEXT_INVERSE/2: begin
					// if 0 -> normal (white letters on black background)
					// if 1 -> inverted (black letters on white background)
					inverse <= data_to_write;
				end
				PORT_LED/2: begin  // LEDs
					LED[7:0] <= data_to_write;
				end
				PORT_SPI_OUT/2: begin
					spi_out <= data_to_write;
					spi_start <= 1'b1;
				end
				PORT_SPI1_OUT/2: begin
					spi_out1 <= data_to_write;
					spi_start1 <= 1'b1;
				end
				PORT_SPI_CS/2: begin
					spi_cs <= data_to_write[0];
				end
				PORT_SPI1_CS/2: begin
					spi_cs1 <= data_to_write[0];
				end
				PORT_DMA_ADDR_1/2: begin
					dma_addr_1[31:16] <= data_to_write;
				end
				(PORT_DMA_ADDR_1/2) + 1: begin
					dma_addr_1[15:0] <= data_to_write;
				end
				PORT_DMA_COUNT_1/2: begin
					dma_count_1[31:16] <= data_to_write;
					dma_current_1 <= 0;
				end
				(PORT_DMA_COUNT_1/2) + 1: begin
					dma_count_1[15:0] <= data_to_write;
					dma_current_1 <= 0;
				end
				PORT_DMA_START_RCV_1/2: begin
					dma_start_rcv_1 <= data_to_write[0];
					spi_sent_ff_1 <= 0;
					dma_spi_received_1 <= 1;
					not_received_counter <= 0;
				end 
				PORT_MOUSE_STRUCT_ADDR/2: begin
					mouse_struct_addr[31:16] <= data_to_write;
				end
				(PORT_MOUSE_STRUCT_ADDR/2) + 1: begin
					mouse_struct_addr[15:0] <= data_to_write;
				end

				default: begin
				end
			endcase  // end of case (data)
			state <= WRITE_WAIT;
		end
		else begin
			addr_o <= addr;
			if (addr >= SDRAM_START_ADDR) begin
				// Write through, meaning that we save data in both SDRAM and cache
				wr_data_o <= data_to_write;
				`ifdef CACHE
				// now we need to store the data that has to be saved right into this cache
				cl[addr[11:0]] <= data_to_write;
				// write tag
				tag[addr[11:0]] <= addr[23:12];
				`endif
				wr_enable_o <= 1'b1;
				if (busy_i) begin
					state <= WRITE_WAIT;
				end
			end
			else begin
				`ifdef READ_ONLY
				if (((addr < 24'h80) || (addr > 24'h1c4)) && (addr >= 4))
				`endif
				begin
					rd <= 1'b0;
					wr <= 1'b1;
					data_to_write_o <= data_to_write;
					state <= WRITE_WAIT;
				end
				`ifdef READ_ONLY
				else begin
					state <= next_state;
				end
				`endif
			end
		end
	end
	WRITE_WAIT: begin
		if (addr[30] == 1'b1) begin
			// Memory mapped IO
			tx_send <= 1'b0;
			spi_start <= 1'b0;
			spi_start1 <= 1'b0;
			state <= next_state;
		end
		else begin
			if (addr >= SDRAM_START_ADDR) begin
				wr_enable_o <= 1'b0;
				if (~busy_i) begin
					state <= next_state;
				end
			end
			else begin
				rd <= 1'b0;
				wr <= 1'b0;
				state <= next_state;
			end
		end
	end
	default: begin
		state <= INIT;
	end
 endcase
end
end



endmodule
