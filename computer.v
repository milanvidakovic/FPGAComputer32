`define GRAPHICS
//`define UART

//=======================================================
//  This code is generated by Terasic System Builder
//=======================================================
`timescale 1ns / 10ps
module computer(

	//////////// CLOCK //////////
	CLOCK_50,

	//////////// LED //////////
	LED,

	//////////// KEY //////////
	KEY,

	//////////// SW //////////
	SW,

	//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
	gpio0,
	gpio0_IN,

	//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
	gpio1,
	gpio1_IN,

	/* SDRAM INTERFACE */
	sdram_ba_pad_o,
	sdram_a_pad_o,
	sdram_dq_pad_io,
	sdram_dqm_pad_o,
	sdram_cas_pad_o,
	sdram_ras_pad_o,
	sdram_we_pad_o,
	sdram_cs_n_pad_o,
	sdram_cke_pad_o,
	sdram_clk_pad_o

);

//=======================================================
//  PARAMETER declarations
//=======================================================


//=======================================================
//  PORT declarations
//=======================================================

//////////// CLOCK //////////
input 		          		CLOCK_50;

//////////// LED //////////
output		     [7:0]		LED;

//////////// KEY //////////
input 		     [1:0]		KEY;

//////////// SW //////////
input 		     [3:0]		SW;

//////////// GPIO_0, GPIO_0 connect to GPIO Default //////////
inout 		    [33:0]		gpio0;
input 		     [1:0]		gpio0_IN;

//////////// GPIO_1, GPIO_1 connect to GPIO Default //////////
inout 		    [33:0]		gpio1;
input 		     [1:0]		gpio1_IN;

  /* SDRAM INTERFACE */
output [1:0]  sdram_ba_pad_o;
output [12:0] sdram_a_pad_o;
inout  [15:0] sdram_dq_pad_io;
output [1:0]  sdram_dqm_pad_o;
output        sdram_cas_pad_o;
output        sdram_ras_pad_o;
output        sdram_we_pad_o;
output        sdram_cs_n_pad_o;
output        sdram_cke_pad_o;
output        sdram_clk_pad_o;


//=======================================================
//  Constants
//=======================================================

localparam IRQ_TIMER = 0;
localparam IRQ_UART  = 1;
localparam IRQ_PS2   = 2;
localparam IRQ_SPI   = 3;
localparam IRQ_SPI1  = 4;
localparam IRQ_PS2_MOUSE   = 5;
localparam IRQ_DMA_1 = 14;
localparam IRQ_DMA_2 = 15;

localparam N = 6'd32;

//=======================================================
//  Structural coding
//=======================================================
// address bus, data bus
wire [31:0] addr, addr2;
wire [15:0] data, data2;
wire [15:0] data_to_write;
// memory read signal
reg rd, rd2;
// memory write signal
reg wr, wr2;

// ###################
// Video VGA text instance 
// ###################

wire r, g, b, hs, vs, rs, gs, bs;
wire vgaoe = 1'b1;

assign gpio0[3] = vgaoe ? b  : 1'bZ ;
assign gpio0[1] = vgaoe ? g  : 1'bZ ;
assign gpio0[0] = vgaoe ? r  : 1'bZ ;
assign gpio0[5] = vgaoe ? hs : 1'bZ ;
assign gpio0[7] = vgaoe ? vs : 1'bZ ;

assign gpio0[13] = vgaoe ? bs  : 1'bZ ;
assign gpio0[11] = vgaoe ? gs  : 1'bZ ;
assign gpio0[9] = vgaoe ? rs  : 1'bZ ;

reg [1:0]vga_mode;
reg  inverse;
//wire v0, v1;
//assign v0 = (vga_mode == 2'b00);
//assign v1 = (vga_mode == 2'b01);

vga_module #(.N(N))vga0(
	vga_mode,
	
	//////////// CLOCK //////////
	clk100,
	CLOCK_50,
	clk25,

	//////////// RESET KEY //////////
	~KEY[0],

	//////////// GPIO //////////
	r, 
	g, 
	b, 
	hs, 
	vs,
	rs,
	gs,
	bs,
	
	data2,
	addr2,
	rd2, 
	wr2,
	inverse
);

`ifdef GRAPHICS
vga_320x240 vga1 (
	vga_mode,
	
	//////////// CLOCK //////////
	clk100,
	CLOCK_50,
	clk25,

	//////////// RESET KEY //////////
	~KEY[0],

	//////////// GPIO //////////
	r, 
	g, 
	b, 
	hs, 
	vs,

	rs,
	gs,
	bs,

	data2,
	addr2,
	rd2, 
	wr2
);
`endif

// ####################################################################################################################
// RAM instance (dual port RAM; one port is connected to the CPU, while the other is connected to the video subsystem)
// ####################################################################################################################
RAM ram(
	clk100,
	data,
	addr,
	rd, wr,
	data_to_write,
	
	data2,
	addr2,
	rd2
);

// ####################################
// SPI Master instance
// ####################################
wire 		 spi_start;
wire [7:0] spi_in;
reg [7:0] spi_out;
wire spi_ready;
wire spi_received;
reg [7:0] spi_in_r;
reg fake_CS;

SPI_Master_With_Single_CS spi0 (
	.i_Clk(clk100),
	.i_Rst_L(KEY[0]),
	.i_TX_Count(1),
	.i_TX_DV(spi_start),
	.o_RX_Byte(spi_in),
	.i_TX_Byte(spi_out),
	.o_RX_DV(spi_received),
	.o_TX_Ready(spi_ready),
	
	.o_SPI_MOSI(gpio0[32]),
	.i_SPI_MISO(gpio0[30]),
	.o_SPI_Clk(gpio0[28]),
	.o_SPI_CS_n(fake_CS)
	
);

wire 		 spi_start1;
wire [7:0] spi_in1;
reg [7:0] spi_out1;
wire spi_ready1;
wire spi_received1;
reg [7:0] spi_in_r1;
reg fake_CS1;

SPI_Master_With_Single_CS spi1 (
	.i_Clk(clk100),
	.i_Rst_L(KEY[0]),
	.i_TX_Count(1),
	.i_TX_DV(spi_start1),
	.o_RX_Byte(spi_in1),
	.i_TX_Byte(spi_out1),
	.o_RX_DV(spi_received1),
	.o_TX_Ready(spi_ready1),
	
	.o_SPI_MOSI(gpio0[21]),
	.i_SPI_MISO(gpio0[23]),
	.o_SPI_Clk(gpio0[19]),
	.o_SPI_CS_n(fake_CS1)
	
);

// ####################################
// PS/2 keyboard instance
// ####################################
wire [7:0] ps2_data;
wire ps2_received;
reg [7:0] ps2_data_r;

PS2_Controller #(.INITIALIZE_MOUSE(0)) PS2 (
	// Inputs
	.CLOCK_50			(CLOCK_50),
	.reset				(~KEY[0]),

	// Bidirectionals
	.PS2_CLK			(gpio0[33]),
 	.PS2_DAT			(gpio0[31]),

	// Outputs
	.received_data		(ps2_data),
	.received_data_en	(ps2_received)
); 

// ####################################
// PS/2 mouse instance
// ####################################
wire [7:0] ps2_data_mouse;
wire ps2_received_mouse;
reg [7:0] ps2_data_r_mouse;

PS2_Controller PS2_mouse (
	// Inputs
	.CLOCK_50			(CLOCK_50),
	.reset				(~KEY[0]),

	// Bidirectionals
	.PS2_CLK			(gpio0[2]),
 	.PS2_DAT			(gpio0[4]),

	// Outputs
	.received_data		(ps2_data_mouse),
	.received_data_en	(ps2_received_mouse)
); 


// ####################################
// UART receiver instance
// ####################################
wire [7:0] rx_data;
wire rx_received;
reg [7:0] rx_data_r;
`ifdef UART
rx_serial rsr(
  clk100,
  ~KEY[0],
  gpio0[27], 		// Input pin - receive line
  rx_data,  		// here we will receive a character
  rx_received    	// if something came from serial, this goes high
);
`endif

// ####################################
// UART transmitter instance
// ####################################
wire [7:0] tx_data;
wire tx_send;
wire tx_busy;
`ifdef UART
tx_serial tsr(
  clk100,
  ~KEY[0],
  tx_data, 		// Character to output
  tx_send,		// High = request a send
  gpio0[25],	// Output pin
  tx_busy		// High while character is being output
);
`endif

// ########################################
// IRQs 
// ########################################

reg cpu_reset = 1'b1;
reg [31:0] reset_counter;

// IRQ signals
wire [15:0] irq;
//reg [7:0] fakeLED;

always @ (posedge clk100) begin
	if (~KEY[0]) begin
		irq <= 0;
		reset_counter <= 32'd0;
		cpu_reset <= 1'b1;
	end
	if (reset_counter == 32'd200) begin
		cpu_reset <= 1'b0;
	end
	else
		reset_counter <= reset_counter + 1'b1;
	
	// ############################### IRQ4 - SPI1 Master (ethernet) #############################
	if (spi_received1) begin
		spi_in_r1 <= spi_in1;
		// if we have received a byte from the MISO1, we will trigger the IRQ#4
		irq[IRQ_SPI1] <= 1'b1;
	end
	else 
	begin
		irq[IRQ_SPI1] <= 1'b0;
	end
	// ############################### IRQ3 - SPI Master (SD card) #############################
	if (spi_received) begin
		spi_in_r <= spi_in;
		// if we have received a byte from the MISO, we will trigger the IRQ#3
		irq[IRQ_SPI] <= 1'b1;
	end
	else 
	begin
		irq[IRQ_SPI] <= 1'b0;
	end
	// ############################### IRQ2 - PS/2 keyboard #############################
	if (ps2_received) begin
		ps2_data_r <= ps2_data;
		// if we have received a byte from the keyboard, we will trigger the IRQ#2
		irq[IRQ_PS2] <= 1'b1;
	end
	else 
	begin
		irq[IRQ_PS2] <= 1'b0;
	end
	// ############################### IRQ5 - PS/2 mouse #############################
	if (ps2_received_mouse) begin
		ps2_data_r_mouse <= ps2_data_mouse;
		// if we have received a byte from the keyboard, we will trigger the IRQ#2
		irq[IRQ_PS2_MOUSE] <= 1'b1;
	end
	else 
	begin
		irq[IRQ_PS2_MOUSE] <= 1'b0;
	end
	// ############################### IRQ1 - UART #############################
	if (rx_received) begin
		rx_data_r <= rx_data;
		// if we have received a byte from the UART, we will trigger the IRQ#1
		irq[IRQ_UART] <= 1'b1;
	end
	else 
	begin
		irq[IRQ_UART] <= 1'b0;
	end
end

// ####################################
// CPU instance
// ####################################
cpu cpu0 (
  .clk         (clk100),
  .rst         (cpu_reset),
	
	// SDRAM interface
  .rd_enable_o (rd_enable),
  .wr_enable_o (wr_enable),
  .busy_i      (busy),
  .rd_ready_i  (rd_ready),
  .rd_data_i   (rd_data),
  .wr_data_o   (wr_data),
  .addr_o      (addr),

  .LED         (LED),
	// STATIC RAM interface
  .rd          (rd),
  .wr          (wr),
  .data_i      (data),
  .data_to_write_o(data_to_write),

	// UART interface
  .rx_data    (rx_data_r),  // UART RX data
  .tx_send    (tx_send),    // UART TX send signal
  .tx_busy    (tx_busy),    // UART TX busy signal
  .tx_data    (tx_data),    // UART TX data

	// VGA interface
  .vga_mode   (vga_mode),	 // VGA mode: 0-text; 1-320x240
  .inverse    (inverse),	 // inverse colors on VGA text mode
	// PS/2 interface
  .ps2_data   (ps2_data_r), // keyboard data
  .ps2_data_mouse   (ps2_data_r_mouse), // mouse data
  
  	// SPI0 interface
  .spi_start	(spi_start),
  .spi_in		(spi_in_r),
  .spi_out		(spi_out),
  .spi_ready	(spi_ready),  

  	// SPI1 interface
  .spi_start1	(spi_start1),
  .spi_in1		(spi_in_r1),
  .spi_out1		(spi_out1),
  .spi_ready1	(spi_ready1),  

  .spi_cs		(gpio0[26]),
  .spi_cs1		(gpio0[29]),
  
  // INTERRUPTS interface
  .irq_i      (irq)

);


// ########################################
// SDRAM stuff 
// ########################################
wire	[15:0] wr_data;
wire         wr_enable;

wire	[15:0] rd_data;
wire         rd_enable;

wire         clk100;
wire			 clk25;
wire         rd_ready;
wire         busy;

/* PLL */
pll plli (
  .inclk0    (CLOCK_50),
  .c0        (clk100),
  .c1        (clk25),
  .c2        (),
  .areset    (~KEY[0])
);

assign sdram_clk_pad_o = clk100;

/* SDRam */
sdram_controller sdram_controlleri (
    /* HOST INTERFACE */
    .wr_addr(addr),
    .wr_data(wr_data),
    .wr_enable(wr_enable),

    .rd_addr(addr),
    .rd_data(rd_data),
    .rd_ready(rd_ready),
    .rd_enable(rd_enable),

    .busy(busy), .rst_n(KEY[0]), .clk(clk100),

    /* SDRAM SIDE */
    .addr          (sdram_a_pad_o),
    .bank_addr     (sdram_ba_pad_o),
    .data          (sdram_dq_pad_io),
    .clock_enable  (sdram_cke_pad_o),
    .cs_n          (sdram_cs_n_pad_o),
    .ras_n         (sdram_ras_pad_o),
    .cas_n         (sdram_cas_pad_o),
    .we_n          (sdram_we_pad_o),
    .data_mask_low (sdram_dqm_pad_o[0]),
    .data_mask_high(sdram_dqm_pad_o[1])
);

initial begin
	vga_mode <= 2'b00; // text mode
end

endmodule
