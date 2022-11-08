//======================================================================
//
// spi_slave.v
// -----------
// Simple SPI slave.
//
// Received bytes are presented to local consumer by setting the
// rx_data_valid flag. The consumer is expected to signal consumption
// by asserting rx_data_ack for one cycle.
//
// Bytes to be transmitted from a local producer are presented to
// the SPI slave by setting the tx_data_valid. When the SPI has
// accepted the data for transmission it will assert tx_data_ack
// for one cycle.
//
//
// Author: Joachim Strömbergson
// Copyright (c) 2021, Mullvad VPN AB
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following
// conditions are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above
//   copyright notice, this list of conditions and the following
//   disclaimer in the documentation and/or other materials provided
//   with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//======================================================================

`default_nettype none

module spi_slave(
                 input  wire         clk,
                 input  wire         rst_n,

                 input  wire         SPI_SCK,
                 input  wire         SPI_SS,
                 input  wire         SPI_MOSI,
                 output wire         SPI_MISO,

                 output wire         spi_active,

                 output wire         rx_data_valid,
                 output wire [7 : 0] rx_data,
                 input wire          rx_data_ack,

                 input wire          tx_data_valid,
                 input  wire [7 : 0] tx_data,
                 output wire         tx_data_ack
                );


  //----------------------------------------------------------------
  // Local parameters.
  //----------------------------------------------------------------
  localparam RX_CTRL_IDLE = 3'h0;
  localparam RX_CTRL_BITS = 3'h1;
  localparam RX_CTRL_BYTE = 3'h2;
  localparam RX_CTRL_DONE = 3'h3;

  localparam TX_CTRL_IDLE = 3'h0;
  localparam TX_CTRL_BYTE = 3'h1;
  localparam TX_CTRL_BITS = 3'h2;
  localparam TX_CTRL_DONE = 3'h3;


  //----------------------------------------------------------------
  // Registers and associated wires.
  //----------------------------------------------------------------
  reg [2 : 0] spi_sck_reg;
  reg [2 : 0] spi_ss_reg;
  reg [2 : 0] spi_mosi_reg;

  reg [7 : 0] rx_data_reg;
  reg [7 : 0] rx_data_new;
  reg         rx_data_rst;
  reg         rx_data_nxt;
  reg         rx_data_we;

  reg         rx_data_valid_reg;
  reg         rx_data_valid_new;
  reg         rx_data_valid_we;

  reg         spi_active_reg;
  reg         spi_active_new;
  reg         spi_active_we;

  reg [3 : 0] rx_bit_ctr_reg;
  reg [3 : 0] rx_bit_ctr_new;
  reg         rx_bit_ctr_rst;
  reg         rx_bit_ctr_inc;
  reg         rx_bit_ctr_we;

  reg [3 : 0] tx_bit_ctr_reg;
  reg [3 : 0] tx_bit_ctr_new;
  reg         tx_bit_ctr_rst;
  reg         tx_bit_ctr_inc;
  reg         tx_bit_ctr_we;

  reg [7 : 0] tx_data_reg;
  reg [7 : 0] tx_data_new;
  reg         tx_data_nxt;
  reg         tx_data_load;
  reg         tx_data_rst;
  reg         tx_data_we;

  reg         tx_data_ack_reg;
  reg         tx_data_ack_new;

  reg [2 : 0] rx_ctrl_reg;
  reg [2 : 0] rx_ctrl_new;
  reg         rx_ctrl_we;

  reg [2 : 0] tx_ctrl_reg;
  reg [2 : 0] tx_ctrl_new;
  reg         tx_ctrl_we;

  reg         spi_miso_buf;
  reg         spi_miso_buf_we;

  //----------------------------------------------------------------
  // Concurrent assignments for ports.
  //----------------------------------------------------------------
  //assign SPI_MISO      = tx_data_reg[0];
  assign SPI_MISO      = spi_miso_buf;

  assign spi_active    = spi_active_reg;

  assign rx_data_valid = rx_data_valid_reg;
  assign rx_data       = rx_data_reg;

  assign tx_data_ack   = tx_data_ack_reg;


  //----------------------------------------------------------------
  // reg_update
  //----------------------------------------------------------------
  always @(posedge clk)
    begin : reg_update
      if (!rst_n)
        begin
          spi_mosi_reg      <= 3'h0;
          spi_sck_reg       <= 3'h0;
          spi_ss_reg        <= 3'h7;
          spi_active_reg    <= 1'h0;
          rx_data_reg       <= 8'h0;
          tx_data_reg       <= 8'h0;
          spi_miso_buf      <= 1'h0;
          rx_data_valid_reg <= 1'h0;
          rx_bit_ctr_reg    <= 4'h0;
          tx_bit_ctr_reg    <= 4'h0;
          tx_data_ack_reg   <= 1'h0;
          rx_ctrl_reg       <= RX_CTRL_IDLE;
          tx_ctrl_reg       <= TX_CTRL_IDLE;
        end

      else
        begin
          spi_sck_reg       <= {spi_sck_reg[1 : 0],  SPI_SCK};
          spi_ss_reg        <= {spi_ss_reg[1 : 0],   SPI_SS};
          spi_mosi_reg      <= {spi_mosi_reg[1 : 0], SPI_MOSI};

          tx_data_ack_reg   <=  tx_data_ack_new;

          if (rx_data_we) begin
            rx_data_reg <= rx_data_new;
          end

          if (rx_data_valid_we) begin
            rx_data_valid_reg <= rx_data_valid_new;
          end

          if (tx_data_we) begin
            tx_data_reg <= tx_data_new;
          end

          if (spi_miso_buf_we) begin
            spi_miso_buf <= tx_data_reg[7];
          end

          if (rx_bit_ctr_we) begin
            rx_bit_ctr_reg <= rx_bit_ctr_new;
          end

          if (tx_bit_ctr_we) begin
            tx_bit_ctr_reg <= tx_bit_ctr_new;
          end

          if (spi_active_we) begin
            spi_active_reg <= spi_active_new;
          end

          if (rx_ctrl_we) begin
            rx_ctrl_reg <= rx_ctrl_new;
          end

          if (tx_ctrl_we) begin
            tx_ctrl_reg <= tx_ctrl_new;
          end
        end
    end


  //----------------------------------------------------------------
  // data_regs_update
  //
  // Logic to handle load, reset and shift operations
  // of the tx_data.
  //----------------------------------------------------------------
  always @*
    begin : data_regs_logic
      rx_data_new = 8'h0;
      rx_data_we  = 1'h0;
      tx_data_new = 8'h0;
      tx_data_we  = 1'h0;
      spi_miso_buf_we = 1'h0;


      if (rx_data_rst) begin
        rx_data_new = 8'h0;
        rx_data_we  = 1'h1;
      end

      else if (rx_data_nxt) begin
        rx_data_new = {rx_data_reg[6:0], spi_mosi_reg[2]};
        rx_data_we  = 1'h1;
      end


      if (tx_data_rst) begin
        tx_data_new = 8'h0;
        tx_data_we  = 1'h1;
        spi_miso_buf_we = 1'h1;
      end

      else if (tx_data_load) begin
        tx_data_new = tx_data;
        tx_data_we  = 1'h1;
      end

      else if (tx_data_nxt) begin
        tx_data_new = {tx_data_reg[6 : 0], 1'h0};
        tx_data_we  = 1'h1;
	spi_miso_buf_we = 1'h1;
      end
    end

  //----------------------------------------------------------------
  // rx_bit_ctr
  //----------------------------------------------------------------
  always @*
    begin : rx_bit_ctr
      rx_bit_ctr_new = 4'h0;
      rx_bit_ctr_we  = 1'h0;

      if (rx_bit_ctr_rst) begin
        rx_bit_ctr_new = 4'h0;
        rx_bit_ctr_we  = 1'h1;
      end

      else if (rx_bit_ctr_inc) begin
        rx_bit_ctr_new = rx_bit_ctr_reg + 1'h1;
        rx_bit_ctr_we  = 1'h1;
      end
    end


  //----------------------------------------------------------------
  // tx_bit_ctr
  //----------------------------------------------------------------
  always @*
    begin : tx_bit_ctr
      tx_bit_ctr_new = 4'h0;
      tx_bit_ctr_we  = 1'h0;

      if (tx_bit_ctr_rst) begin
        tx_bit_ctr_new = 4'h0;
        tx_bit_ctr_we  = 1'h1;
      end

      else if (tx_bit_ctr_inc) begin
        tx_bit_ctr_new = tx_bit_ctr_reg + 1'h1;
        tx_bit_ctr_we  = 1'h1;
      end
    end


  //----------------------------------------------------------------
  // rx_ctrl
  //
  // Logic for controlling recive side of the SPI slave.
  //----------------------------------------------------------------
  always @*
    begin : rx_ctrl
      reg spi_sck_posedge;
      reg spi_ss_low;

      spi_active_new    = 1'h0;
      spi_active_we     = 1'h0;
      rx_data_rst       = 1'h0;
      rx_data_nxt       = 1'h0;
      rx_bit_ctr_rst    = 1'h0;
      rx_bit_ctr_inc    = 1'h0;
      rx_data_valid_new = 1'h0;
      rx_data_valid_we  = 1'h0;
      rx_ctrl_new       = RX_CTRL_IDLE;
      rx_ctrl_we        = 1'h0;


      // Detect received SPI events.
      spi_ss_low      = ~spi_ss_reg[2];
      spi_sck_posedge = ((!spi_sck_reg[2]) && (spi_sck_reg[1]));


      case (rx_ctrl_reg)
        RX_CTRL_IDLE: begin
          if (spi_ss_low) begin
            spi_active_new = 1'h1;
            spi_active_we  = 1'h1;
            rx_data_rst    = 1'h1;
            rx_bit_ctr_rst = 1'h1;
            rx_ctrl_new    = RX_CTRL_BITS;
            rx_ctrl_we     = 1'h1;
          end
        end


        RX_CTRL_BITS: begin
          if (spi_ss_low) begin
            if (spi_sck_posedge) begin
              rx_data_nxt        = 1'h1;
              rx_bit_ctr_inc     = 1'h1;

              if (rx_bit_ctr_reg == 4'h7) begin
                rx_data_valid_new = 1'h1;
                rx_data_valid_we  = 1'h1;
                rx_ctrl_new       = RX_CTRL_BYTE;
                rx_ctrl_we        = 1'h1;
              end
            end
          end else begin
            rx_ctrl_new = RX_CTRL_DONE;
            rx_ctrl_we  = 1'h1;
          end
        end


        RX_CTRL_BYTE: begin
          if (spi_ss_low) begin
            if (rx_data_ack) begin
              rx_data_valid_new = 1'h0;
              rx_data_valid_we  = 1'h1;
              rx_bit_ctr_rst    = 1'h1;
              rx_ctrl_new       = RX_CTRL_BITS;
              rx_ctrl_we        = 1'h1;
            end
          end else begin
            rx_ctrl_new = RX_CTRL_DONE;
            rx_ctrl_we  = 1'h1;
          end
        end


        RX_CTRL_DONE: begin
          spi_active_new    = 1'h0;
          spi_active_we     = 1'h1;
          rx_data_valid_new = 1'h0;
          rx_data_valid_we  = 1'h1;
          rx_data_rst       = 1'h1;
          rx_bit_ctr_rst    = 1'h1;
          rx_ctrl_new       = RX_CTRL_IDLE;
          rx_ctrl_we        = 1'h1;
        end

        default : begin
        end
      endcase // case (spi_slave_ctrl_reg)
    end


  //----------------------------------------------------------------
  // tx_ctrl
  //
  // Logic for controlling the transmit side of the SPI slave.
  //----------------------------------------------------------------
  always @*
    begin : tx_ctrl
      reg spi_sck_negedge;
      reg spi_ss_low;

      tx_data_ack_new = 1'h0;
      tx_data_rst     = 1'h0;
      tx_data_load    = 1'h0;
      tx_data_nxt     = 1'h0;
      tx_bit_ctr_rst  = 1'h0;
      tx_bit_ctr_inc  = 1'h0;
      tx_ctrl_new     = TX_CTRL_IDLE;
      tx_ctrl_we      = 1'h0;

      // Detect received SPI events.
      spi_ss_low      = ~spi_ss_reg[2];
      spi_sck_negedge = ((spi_sck_reg[2]) && (!spi_sck_reg[1]));

      case (tx_ctrl_reg)
        TX_CTRL_IDLE: begin
          if (spi_ss_low) begin
            tx_bit_ctr_rst = 1'h1;
            tx_data_rst    = 1'h1;
            tx_ctrl_new    = TX_CTRL_BYTE;
            tx_ctrl_we     = 1'h1;
          end
        end


        TX_CTRL_BYTE: begin
          if (spi_ss_low) begin
            if (tx_data_valid) begin
              tx_bit_ctr_rst  = 1'h1;
              tx_data_load    = 1'h1;
              tx_data_ack_new = 1'h1;
              tx_ctrl_new     = TX_CTRL_BITS;
              tx_ctrl_we      = 1'h1;
            end
          end else begin
            tx_ctrl_new = TX_CTRL_DONE;
            tx_ctrl_we  = 1'h1;
          end
        end

        TX_CTRL_BITS: begin
          if (spi_ss_low) begin
            if (spi_sck_negedge) begin
              tx_data_nxt    = 1'h1;
              tx_bit_ctr_inc = 1'h1;
              if (tx_bit_ctr_reg == 4'h7) begin
                tx_ctrl_new    = TX_CTRL_BYTE;
                tx_ctrl_we     = 1'h1;
              end
            end
          end else begin
            tx_ctrl_new     = TX_CTRL_DONE;
            tx_ctrl_we      = 1'h1;
          end
        end


        TX_CTRL_DONE: begin
          tx_bit_ctr_rst = 1'h1;
          tx_data_rst    = 1'h1;
          tx_ctrl_new    = TX_CTRL_IDLE;
          tx_ctrl_we     = 1'h1;
        end

        default: begin
        end
      endcase // case (tx_ctrl_reg)
    end

endmodule // spi_slave

//======================================================================
// EOF spi_slave.v
//======================================================================
