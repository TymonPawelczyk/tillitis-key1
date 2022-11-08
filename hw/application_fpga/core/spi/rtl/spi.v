//======================================================================
//
// spi.v
// -----
// Top level wrapper for the SPI core. The wrapper also instantiates
// A RX-FIFO.
//
//
// Author: Joachim Strombergson
// Copyright (c) 2014, Secworks Sweden AB
// SPDX-License-Identifier: BSD-2-Clause
//
// Redistribution and use in source and binary forms, with or
// without modification, are permitted provided that the following
// conditions are met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in
//    the documentation and/or other materials provided with the
//    distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
// ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

module spi (
            input wire           clk,
            input wire           reset_n,

            input  wire          SPI_SCK,
            input  wire          SPI_SS,
            input  wire          SPI_MOSI,
            output wire          SPI_MISO,

            input wire           cs,
            input wire           we,
            input wire [7 : 0]   address,
            input wire [31 : 0]  write_data,
            output wire [31 : 0] read_data,
            output wire          ready
           );


  //----------------------------------------------------------------
  // Internal constant and parameter definitions.
  //----------------------------------------------------------------
  localparam ADDR_RX_STATUS    = 8'h20;
  localparam ADDR_RX_DATA      = 8'h21;

  localparam ADDR_TX_STATUS    = 8'h40;
  localparam ADDR_TX_DATA      = 8'h41;


  //----------------------------------------------------------------
  // Registers including update variables and write enable.
  //----------------------------------------------------------------


  //----------------------------------------------------------------
  // Wires.
  //----------------------------------------------------------------
  wire          core_rxd_syn;
  wire [7 : 0]  core_rxd_data;
  wire          core_rxd_ack;

  reg           core_txd_syn;
  reg [7 : 0]   core_txd_data;
  wire          core_txd_ready;

  wire          fifo_out_syn;
  wire [7 : 0]  fifo_out_data;
  reg           fifo_out_ack;

  reg [31 : 0]  tmp_read_data;
  reg           tmp_ready;


  //----------------------------------------------------------------
  // Concurrent connectivity for ports etc.
  //----------------------------------------------------------------
  assign read_data = tmp_read_data;
  assign ready     = tmp_ready;


  //----------------------------------------------------------------
  // Module instantiations.
  //----------------------------------------------------------------
  spi_slave core(
                 .clk(clk),
                 .rst_n(reset_n),

                 .SPI_SCK(SPI_SCK),
                 .SPI_SS(SPI_SS),
                 .SPI_MOSI(SPI_MOSI),
                 .SPI_MISO(SPI_MISO),

		 .spi_active(),

                 // Internal receive interface.
                 .rx_data_valid(core_rxd_syn),
                 .rx_data(core_rxd_data),
                 .rx_data_ack(core_rxd_ack),

                 // Internal transmit interface.
                 .tx_data_valid(core_txd_syn),
                 .tx_data(core_txd_data),
                 .tx_data_ack(core_txd_ready)
                 );


  spi_fifo fifo(
		.clk(clk),
		.reset_n(reset_n),

		.in_syn(core_rxd_syn),
		.in_data(core_rxd_data),
		.in_ack(core_rxd_ack),

		.out_syn(fifo_out_syn),
		.out_data(fifo_out_data),
		.out_ack(fifo_out_ack)
		);


  //----------------------------------------------------------------
  // reg_update
  //
  // Update functionality for all registers in the core.
  // All registers are positive edge triggered with synchronous
  // active low reset.
  //----------------------------------------------------------------
  always @ (posedge clk)
    begin: reg_update
      if (!reset_n) begin
        bit_rate_reg  <= DEFAULT_BIT_RATE;
        data_bits_reg <= DEFAULT_DATA_BITS;
        stop_bits_reg <= DEFAULT_STOP_BITS;
      end
      else begin
        if (bit_rate_we) begin
          bit_rate_reg  <= write_data[15 : 0];
        end

        if (data_bits_we) begin
          data_bits_reg  <= write_data[3 : 0];
        end

        if (stop_bits_we) begin
          stop_bits_reg  <= write_data[1 : 0];
        end
      end
    end // reg_update


  //----------------------------------------------------------------
  // api
  //
  // The core API that allows an internal host to control the
  // core functionality.
  //----------------------------------------------------------------
  always @*
    begin: api
      // Default assignments.
      bit_rate_we   = 1'h0;
      data_bits_we  = 1'h0;
      stop_bits_we  = 1'h0;
      core_txd_syn  = 1'h0;
      fifo_out_ack  = 1'h0;
      tmp_read_data = 32'h0;
      tmp_ready     = 1'h0;

      core_txd_data = write_data[7 : 0];

      if (cs) begin
	tmp_ready = 1'h1;

        if (we) begin
          case (address)
	    ADDR_TX_DATA: begin
	      core_txd_syn = 1'h1;
	    end

            default: begin
            end
          endcase // case (address)
        end

	else begin
          case (address)
	    ADDR_RX_STATUS: begin
              tmp_read_data = {31'h0, fifo_out_syn};
	    end

	    ADDR_RX_DATA: begin
	      fifo_out_ack  = 1'h1;
              tmp_read_data = {24'h0, fifo_out_data};
	    end

	    ADDR_TX_STATUS: begin
              tmp_read_data = {31'h0, core_txd_ready};
	    end

            default: begin
            end
          endcase // case (address)
        end
      end
    end

endmodule // uart

//======================================================================
// EOF uart.v
//======================================================================
