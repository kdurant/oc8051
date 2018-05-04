//////////////////////////////////////////////////////////////////////
////                                                              ////
////  8051 cores serial interface                                 ////
////                                                              ////
////  This file is part of the 8051 cores project                 ////
////  http://www.opencores.org/cores/8051/                        ////
////                                                              ////
////  Description                                                 ////
////   uart for 8051 core                                         ////
////                                                              ////
////  To Do:                                                      ////
////   Nothing                                                    ////
////                                                              ////
////  Author(s):                                                  ////
////      - Simon Teran, simont@opencores.org                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
////                                                              ////
//// Copyright (C) 2000 Authors and OPENCORES.ORG                 ////
////                                                              ////
//// This source file may be used and distributed without         ////
//// restriction provided that this copyright statement is not    ////
//// removed from the file and that any derivative work contains  ////
//// the original copyright notice and the associated disclaimer. ////
////                                                              ////
//// This source file is free software; you can redistribute it   ////
//// and/or modify it under the terms of the GNU Lesser General   ////
//// Public License as published by the Free Software Foundation; ////
//// either version 2.1 of the License, or (at your option) any   ////
//// later version.                                               ////
////                                                              ////
//// This source is distributed in the hope that it will be       ////
//// useful, but WITHOUT ANY WARRANTY; without even the implied   ////
//// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      ////
//// PURPOSE.  See the GNU Lesser General Public License for more ////
//// details.                                                     ////
////                                                              ////
//// You should have received a copy of the GNU Lesser General    ////
//// Public License along with this source; if not, download it   ////
//// from http://www.opencores.org/lgpl.shtml                     ////
////                                                              ////
//////////////////////////////////////////////////////////////////////
//
// CVS Revision History
//
// $Log: not supported by cvs2svn $
// Revision 1.14  2003/04/29 11:25:42  simont
// prepared start of receiving if ren is not active.
//
// Revision 1.13  2003/04/10 08:57:16  simont
// remove signal sbuf_txd [12:11]
//
// Revision 1.12  2003/04/07 14:58:02  simont
// change sfr's interface.
//
// Revision 1.11  2003/04/07 13:29:16  simont
// change uart to meet timing.
//
// Revision 1.10  2003/01/13 14:14:41  simont
// replace some modules
//
// Revision 1.9  2002/09/30 17:33:59  simont
// prepared header
//
//

// synopsys translate_off
`include "oc8051_timescale.v"
// synopsys translate_on

`include "oc8051_defines.v"

module oc8051_uart (
    rst,
    clk,
    bit_in,
    data_in,
    wr_addr,
    wr,
    wr_bit,
    rxd,
    txd,
    intr,
    brate2,
    t1_ow,
    pres_ow,
    rclk,
    tclk,
    //registers
    scon,
    pcon,
    sbuf
);
input                       rst;
input                       clk;
input                       bit_in;
input                       wr;
input                       rxd;
input                       wr_bit;
input                       t1_ow;
input                       brate2;
input                       pres_ow;
input                       rclk;
input                       tclk;
input   [7:0]               data_in;
input   [7:0]               wr_addr;

output                      txd;
input                       intr;
output  [7:0]               scon;
output  [7:0]               pcon;
output  [7:0]               sbuf;


reg                         t1_ow_buf;
//
reg     [7:0]               scon;
reg     [7:0]               pcon;


reg                         txd;
reg                         trans;
reg                         receive;
reg                         tx_done;
reg                             rx_done;
reg                             rxd_r;
reg                             shift_tr;
reg                             shift_re;
reg     [1:0]               rx_sam;
reg     [3:0]               tr_count;
reg     [3:0]               re_count;
reg     [7:0]               sbuf_rxd;
reg     [11:0]              sbuf_rxd_tmp;
reg     [10:0]              sbuf_txd;

assign sbuf = sbuf_rxd;
assign intr = scon[1] | scon [0];

//
//serial port control register
//
wire                        ren;
wire                        tb8;
wire                        rb8;
wire                        ri;

assign ren = scon[4];
assign tb8 = scon[3];
assign rb8 = scon[2];
assign ri  = scon[0];

always @(posedge clk or posedge rst)
begin
    if (rst)
        scon <= #DELAY `OC8051_RST_SCON;
    else if ((wr) & !(wr_bit) & (wr_addr==`OC8051_SFR_SCON))
        scon <= #DELAY data_in;
    else if ((wr) & (wr_bit) & (wr_addr[7:3]==`OC8051_SFR_B_SCON))
        scon[wr_addr[2:0]] <= #DELAY bit_in;
    else if (tx_done)
        scon[1] <= #DELAY 1'b1;
    else if (!rx_done) begin
        if (scon[7:6]==2'b00) begin
            scon[0] <= #DELAY 1'b1;
        end else if ((sbuf_rxd_tmp[11]) | !(scon[5])) begin
            scon[0] <= #DELAY 1'b1;
            scon[2] <= #DELAY sbuf_rxd_tmp[11];
        end else
            scon[2] <= #DELAY sbuf_rxd_tmp[11];
    end
end

//
//power control register
//
wire smod;
assign smod = pcon[7];
always @(posedge clk or posedge rst)
begin
    if (rst)
    begin
        pcon <= #DELAY `OC8051_RST_PCON;
    end else if ((wr_addr==`OC8051_SFR_PCON) & (wr) & !(wr_bit))
        pcon <= #DELAY data_in;
end


//
//serial port buffer (transmit)
//

wire wr_sbuf;
assign wr_sbuf = (wr_addr==`OC8051_SFR_SBUF) & (wr) & !(wr_bit);

always @(posedge clk or posedge rst)
begin
    if (rst) begin
        txd      <= #DELAY 1'b1;
        tr_count <= #DELAY 4'd0;
        trans    <= #DELAY 1'b0;
        sbuf_txd <= #DELAY 11'h00;
        tx_done  <= #DELAY 1'b0;
        //
        // start transmiting
        //
    end else if (wr_sbuf) begin
        case (scon[7:6]) /* synopsys parallel_case */
            2'b00: begin  // mode 0
                sbuf_txd <= #DELAY {3'b001, data_in};
            end
            2'b01: begin // mode 1
                sbuf_txd <= #DELAY {2'b01, data_in, 1'b0};
            end
            default: begin  // mode 2 and mode 3
                sbuf_txd <= #DELAY {1'b1, tb8, data_in, 1'b0};
            end
        endcase
        trans    <= #DELAY 1'b1;
        tr_count <= #DELAY 4'd0;
        tx_done  <= #DELAY 1'b0;
        //
        // transmiting
        //
    end else if (trans & (scon[7:6] == 2'b00) & pres_ow) // mode 0
    begin
        if (~|sbuf_txd[10:1]) begin
            trans   <= #DELAY 1'b0;
            tx_done <= #DELAY 1'b1;
        end else begin
            {sbuf_txd, txd} <= #DELAY {1'b0, sbuf_txd};
            tx_done         <= #DELAY 1'b0;
        end
    end else if (trans & (scon[7:6] != 2'b00) & shift_tr) begin // mode 1, 2, 3
        tr_count <= #DELAY tr_count + 4'd1;
        if (~|tr_count) begin
            if (~|sbuf_txd[10:0]) begin
                trans   <= #DELAY 1'b0;
                tx_done <= #DELAY 1'b1;
                txd <= #DELAY 1'b1;
            end else begin
                {sbuf_txd, txd} <= #DELAY {1'b0, sbuf_txd};
                tx_done         <= #DELAY 1'b0;
            end
        end
    end else if (!trans) begin
        txd     <= #DELAY 1'b1;
        tx_done <= #DELAY 1'b0;
    end
end

//
//
reg sc_clk_tr, smod_clk_tr;
always @(brate2 or t1_ow or t1_ow_buf or scon[7:6] or tclk)
begin
    if (scon[7:6]==8'b10) begin //mode 2
        sc_clk_tr = 1'b1;
    end else if (tclk) begin //
        sc_clk_tr = brate2;
    end else begin //
        sc_clk_tr = !t1_ow_buf & t1_ow;
    end
end

always @(posedge clk or posedge rst)
begin
    if (rst) begin
        smod_clk_tr <= #DELAY 1'b0;
        shift_tr    <= #DELAY 1'b0;
    end else if (sc_clk_tr) begin
        if (smod) begin
            shift_tr <= #DELAY 1'b1;
        end else begin
            shift_tr    <= #DELAY  smod_clk_tr;
            smod_clk_tr <= #DELAY !smod_clk_tr;
        end
    end else begin
        shift_tr <= #DELAY 1'b0;
    end
end


//
//serial port buffer (receive)
//
always @(posedge clk or posedge rst)
begin
    if (rst) begin
        re_count     <= #DELAY 4'd0;
        receive      <= #DELAY 1'b0;
        sbuf_rxd     <= #DELAY 8'h00;
        sbuf_rxd_tmp <= #DELAY 12'd0;
        rx_done      <= #DELAY 1'b1;
        rxd_r        <= #DELAY 1'b1;
        rx_sam       <= #DELAY 2'b00;
    end else if (!rx_done) begin
        receive <= #DELAY 1'b0;
        rx_done <= #DELAY 1'b1;
        sbuf_rxd <= #DELAY sbuf_rxd_tmp[10:3];
    end else if (receive & (scon[7:6]==2'b00) & pres_ow) begin //mode 0
        {sbuf_rxd_tmp, rx_done} <= #DELAY {rxd, sbuf_rxd_tmp};
    end else if (receive & (scon[7:6]!=2'b00) & shift_re) begin //mode 1, 2, 3
        re_count <= #DELAY re_count + 4'd1;
        case (re_count) /* synopsys full_case parallel_case */
            4'h7: rx_sam[0] <= #DELAY rxd;
            4'h8: rx_sam[1] <= #DELAY rxd;
            4'h9: begin
                {sbuf_rxd_tmp, rx_done} <= #DELAY {(rxd==rx_sam[0] ? rxd : rx_sam[1]), sbuf_rxd_tmp};
            end
        endcase
        //
        //start receiving
        //
    end else if (scon[7:6]==2'b00) begin //start mode 0
        rx_done <= #DELAY 1'b1;
        if (ren && !ri && !receive) begin
            receive      <= #DELAY 1'b1;
            sbuf_rxd_tmp <= #DELAY 10'h0ff;
        end
    end else if (ren & shift_re) begin
        rxd_r <= #DELAY rxd;
        rx_done <= #DELAY 1'b1;
        re_count <= #DELAY 4'h0;
        receive <= #DELAY (rxd_r & !rxd);
        sbuf_rxd_tmp <= #DELAY 10'h1ff;
    end else if (!ren) begin
        rxd_r <= #DELAY rxd;
    end else
        rx_done <= #DELAY 1'b1;
end

//
//
reg sc_clk_re, smod_clk_re;
always @(brate2 or t1_ow or t1_ow_buf or scon[7:6] or rclk)
begin
    if (scon[7:6]==8'b10) begin //mode 2
        sc_clk_re = 1'b1;
    end else if (rclk) begin //
        sc_clk_re = brate2;
    end else begin //
        sc_clk_re = !t1_ow_buf & t1_ow;
    end
end

always @(posedge clk or posedge rst)
begin
    if (rst) begin
        smod_clk_re <= #DELAY 1'b0;
        shift_re    <= #DELAY 1'b0;
    end else if (sc_clk_re) begin
        if (smod) begin
            shift_re <= #DELAY 1'b1;
        end else begin
            shift_re    <= #DELAY  smod_clk_re;
            smod_clk_re <= #DELAY !smod_clk_re;
        end
    end else begin
        shift_re <= #DELAY 1'b0;
    end
end



//
//
//

always @(posedge clk or posedge rst)
begin
    if (rst) begin
        t1_ow_buf <= #DELAY 1'b0;
    end else begin
        t1_ow_buf <= #DELAY t1_ow;
    end
end



endmodule

