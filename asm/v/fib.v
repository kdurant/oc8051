
///
/// created by p8051Rom.exe
/// author: Simon Teran (simont@opencores.org)
///
/// source file: D:\verilog\oc8051\test\fib.hex
/// date: 6.6.02
/// time: 22:00:50
///

module oc8051_rom (rst, clk, addr, ea_int, data1, data2, data3);

parameter INT_ROM_WID= 9;

input rst, clk;
input [15:0] addr;
output ea_int;
output [7:0] data1, data2, data3;
reg [7:0] data1, data2, data3;
reg [7:0] buff [65535:0];
integer i;

wire ea;

assign ea = | addr[15:INT_ROM_WID];
assign ea_int = ! ea;

initial
begin
    for (i=0; i<65536; i=i+1)
      buff [i] = 8'h00;
#2

    buff [16'h00_00] = 8'h02;
    buff [16'h00_01] = 8'h00;
    buff [16'h00_02] = 8'hC2;
    buff [16'h00_03] = 8'h8B;
    buff [16'h00_04] = 8'h12;
    buff [16'h00_05] = 8'h8A;
    buff [16'h00_06] = 8'h13;
    buff [16'h00_07] = 8'h89;
    buff [16'h00_08] = 8'h14;
    buff [16'h00_09] = 8'h8D;
    buff [16'h00_0a] = 8'h15;
    buff [16'h00_0b] = 8'h74;
    buff [16'h00_0c] = 8'h01;
    buff [16'h00_0d] = 8'h12;
    buff [16'h00_0e] = 8'h00;
    buff [16'h00_0f] = 8'hFB;
    buff [16'h00_10] = 8'hAB;
    buff [16'h00_11] = 8'h12;
    buff [16'h00_12] = 8'hAA;
    buff [16'h00_13] = 8'h13;
    buff [16'h00_14] = 8'hA9;
    buff [16'h00_15] = 8'h14;
    buff [16'h00_16] = 8'hF5;
    buff [16'h00_17] = 8'h82;
    buff [16'h00_18] = 8'h75;
    buff [16'h00_19] = 8'h83;
    buff [16'h00_1a] = 8'h00;
    buff [16'h00_1b] = 8'h12;
    buff [16'h00_1c] = 8'h01;
    buff [16'h00_1d] = 8'h0D;
    buff [16'h00_1e] = 8'h75;
    buff [16'h00_1f] = 8'h16;
    buff [16'h00_20] = 8'h02;
    buff [16'h00_21] = 8'hAD;
    buff [16'h00_22] = 8'h16;
    buff [16'h00_23] = 8'hED;
    buff [16'h00_24] = 8'h33;
    buff [16'h00_25] = 8'h95;
    buff [16'h00_26] = 8'hE0;
    buff [16'h00_27] = 8'hFC;
    buff [16'h00_28] = 8'hC3;
    buff [16'h00_29] = 8'hED;
    buff [16'h00_2a] = 8'h95;
    buff [16'h00_2b] = 8'h15;
    buff [16'h00_2c] = 8'h74;
    buff [16'h00_2d] = 8'h80;
    buff [16'h00_2e] = 8'hF8;
    buff [16'h00_2f] = 8'h6C;
    buff [16'h00_30] = 8'h98;
    buff [16'h00_31] = 8'h50;
    buff [16'h00_32] = 8'h3E;
    buff [16'h00_33] = 8'hAF;
    buff [16'h00_34] = 8'h16;
    buff [16'h00_35] = 8'hEF;
    buff [16'h00_36] = 8'h33;
    buff [16'h00_37] = 8'h95;
    buff [16'h00_38] = 8'hE0;
    buff [16'h00_39] = 8'hFE;
    buff [16'h00_3a] = 8'hEF;
    buff [16'h00_3b] = 8'h24;
    buff [16'h00_3c] = 8'hFE;
    buff [16'h00_3d] = 8'hFD;
    buff [16'h00_3e] = 8'hEE;
    buff [16'h00_3f] = 8'h34;
    buff [16'h00_40] = 8'hFF;
    buff [16'h00_41] = 8'hAB;
    buff [16'h00_42] = 8'h12;
    buff [16'h00_43] = 8'hAA;
    buff [16'h00_44] = 8'h13;
    buff [16'h00_45] = 8'hA9;
    buff [16'h00_46] = 8'h14;
    buff [16'h00_47] = 8'h8D;
    buff [16'h00_48] = 8'h82;
    buff [16'h00_49] = 8'hF5;
    buff [16'h00_4a] = 8'h83;
    buff [16'h00_4b] = 8'h12;
    buff [16'h00_4c] = 8'h00;
    buff [16'h00_4d] = 8'hCE;
    buff [16'h00_4e] = 8'hFD;
    buff [16'h00_4f] = 8'hEF;
    buff [16'h00_50] = 8'h24;
    buff [16'h00_51] = 8'hFF;
    buff [16'h00_52] = 8'hFF;
    buff [16'h00_53] = 8'hEE;
    buff [16'h00_54] = 8'h34;
    buff [16'h00_55] = 8'hFF;
    buff [16'h00_56] = 8'h8F;
    buff [16'h00_57] = 8'h82;
    buff [16'h00_58] = 8'hF5;
    buff [16'h00_59] = 8'h83;
    buff [16'h00_5a] = 8'h12;
    buff [16'h00_5b] = 8'h00;
    buff [16'h00_5c] = 8'hCE;
    buff [16'h00_5d] = 8'h2D;
    buff [16'h00_5e] = 8'hFF;
    buff [16'h00_5f] = 8'hE5;
    buff [16'h00_60] = 8'h16;
    buff [16'h00_61] = 8'hFD;
    buff [16'h00_62] = 8'h33;
    buff [16'h00_63] = 8'h95;
    buff [16'h00_64] = 8'hE0;
    buff [16'h00_65] = 8'h8D;
    buff [16'h00_66] = 8'h82;
    buff [16'h00_67] = 8'hF5;
    buff [16'h00_68] = 8'h83;
    buff [16'h00_69] = 8'hEF;
    buff [16'h00_6a] = 8'h12;
    buff [16'h00_6b] = 8'h01;
    buff [16'h00_6c] = 8'h0D;
    buff [16'h00_6d] = 8'h05;
    buff [16'h00_6e] = 8'h16;
    buff [16'h00_6f] = 8'h80;
    buff [16'h00_70] = 8'hB0;
    buff [16'h00_71] = 8'h22;
    buff [16'h00_72] = 8'h8B;
    buff [16'h00_73] = 8'h12;
    buff [16'h00_74] = 8'h8A;
    buff [16'h00_75] = 8'h13;
    buff [16'h00_76] = 8'h89;
    buff [16'h00_77] = 8'h14;
    buff [16'h00_78] = 8'h8D;
    buff [16'h00_79] = 8'h15;
    buff [16'h00_7a] = 8'hE4;
    buff [16'h00_7b] = 8'hF5;
    buff [16'h00_7c] = 8'h16;
    buff [16'h00_7d] = 8'hAD;
    buff [16'h00_7e] = 8'h16;
    buff [16'h00_7f] = 8'hED;
    buff [16'h00_80] = 8'h33;
    buff [16'h00_81] = 8'h95;
    buff [16'h00_82] = 8'hE0;
    buff [16'h00_83] = 8'hFC;
    buff [16'h00_84] = 8'hC3;
    buff [16'h00_85] = 8'hED;
    buff [16'h00_86] = 8'h95;
    buff [16'h00_87] = 8'h15;
    buff [16'h00_88] = 8'h74;
    buff [16'h00_89] = 8'h80;
    buff [16'h00_8a] = 8'hF8;
    buff [16'h00_8b] = 8'h6C;
    buff [16'h00_8c] = 8'h98;
    buff [16'h00_8d] = 8'h50;
    buff [16'h00_8e] = 8'h19;
    buff [16'h00_8f] = 8'hAB;
    buff [16'h00_90] = 8'h12;
    buff [16'h00_91] = 8'hAA;
    buff [16'h00_92] = 8'h13;
    buff [16'h00_93] = 8'hA9;
    buff [16'h00_94] = 8'h14;
    buff [16'h00_95] = 8'hAF;
    buff [16'h00_96] = 8'h16;
    buff [16'h00_97] = 8'hEF;
    buff [16'h00_98] = 8'h33;
    buff [16'h00_99] = 8'h95;
    buff [16'h00_9a] = 8'hE0;
    buff [16'h00_9b] = 8'h8F;
    buff [16'h00_9c] = 8'h82;
    buff [16'h00_9d] = 8'hF5;
    buff [16'h00_9e] = 8'h83;
    buff [16'h00_9f] = 8'h12;
    buff [16'h00_a0] = 8'h00;
    buff [16'h00_a1] = 8'hCE;
    buff [16'h00_a2] = 8'hF5;
    buff [16'h00_a3] = 8'h80;
    buff [16'h00_a4] = 8'h05;
    buff [16'h00_a5] = 8'h16;
    buff [16'h00_a6] = 8'h80;
    buff [16'h00_a7] = 8'hD5;
    buff [16'h00_a8] = 8'h22;
    buff [16'h00_a9] = 8'h7B;
    buff [16'h00_aa] = 8'h00;
    buff [16'h00_ab] = 8'h7A;
    buff [16'h00_ac] = 8'h00;
    buff [16'h00_ad] = 8'h79;
    buff [16'h00_ae] = 8'h08;
    buff [16'h00_af] = 8'h7D;
    buff [16'h00_b0] = 8'h0A;
    buff [16'h00_b1] = 8'h12;
    buff [16'h00_b2] = 8'h00;
    buff [16'h00_b3] = 8'h03;
    buff [16'h00_b4] = 8'h7B;
    buff [16'h00_b5] = 8'h00;
    buff [16'h00_b6] = 8'h7A;
    buff [16'h00_b7] = 8'h00;
    buff [16'h00_b8] = 8'h79;
    buff [16'h00_b9] = 8'h08;
    buff [16'h00_ba] = 8'h7D;
    buff [16'h00_bb] = 8'h0A;
    buff [16'h00_bc] = 8'h12;
    buff [16'h00_bd] = 8'h00;
    buff [16'h00_be] = 8'h72;
    buff [16'h00_bf] = 8'h80;
    buff [16'h00_c0] = 8'hFE;
    buff [16'h00_c1] = 8'h22;
    buff [16'h00_c2] = 8'h78;
    buff [16'h00_c3] = 8'h7F;
    buff [16'h00_c4] = 8'hE4;
    buff [16'h00_c5] = 8'hF6;
    buff [16'h00_c6] = 8'hD8;
    buff [16'h00_c7] = 8'hFD;
    buff [16'h00_c8] = 8'h75;
    buff [16'h00_c9] = 8'h81;
    buff [16'h00_ca] = 8'h16;
    buff [16'h00_cb] = 8'h02;
    buff [16'h00_cc] = 8'h00;
    buff [16'h00_cd] = 8'hA9;
    buff [16'h00_ce] = 8'hBB;
    buff [16'h00_cf] = 8'h01;
    buff [16'h00_d0] = 8'h0C;
    buff [16'h00_d1] = 8'hE5;
    buff [16'h00_d2] = 8'h82;
    buff [16'h00_d3] = 8'h29;
    buff [16'h00_d4] = 8'hF5;
    buff [16'h00_d5] = 8'h82;
    buff [16'h00_d6] = 8'hE5;
    buff [16'h00_d7] = 8'h83;
    buff [16'h00_d8] = 8'h3A;
    buff [16'h00_d9] = 8'hF5;
    buff [16'h00_da] = 8'h83;
    buff [16'h00_db] = 8'hE0;
    buff [16'h00_dc] = 8'h22;
    buff [16'h00_dd] = 8'h50;
    buff [16'h00_de] = 8'h06;
    buff [16'h00_df] = 8'hE9;
    buff [16'h00_e0] = 8'h25;
    buff [16'h00_e1] = 8'h82;
    buff [16'h00_e2] = 8'hF8;
    buff [16'h00_e3] = 8'hE6;
    buff [16'h00_e4] = 8'h22;
    buff [16'h00_e5] = 8'hBB;
    buff [16'h00_e6] = 8'hFE;
    buff [16'h00_e7] = 8'h06;
    buff [16'h00_e8] = 8'hE9;
    buff [16'h00_e9] = 8'h25;
    buff [16'h00_ea] = 8'h82;
    buff [16'h00_eb] = 8'hF8;
    buff [16'h00_ec] = 8'hE2;
    buff [16'h00_ed] = 8'h22;
    buff [16'h00_ee] = 8'hE5;
    buff [16'h00_ef] = 8'h82;
    buff [16'h00_f0] = 8'h29;
    buff [16'h00_f1] = 8'hF5;
    buff [16'h00_f2] = 8'h82;
    buff [16'h00_f3] = 8'hE5;
    buff [16'h00_f4] = 8'h83;
    buff [16'h00_f5] = 8'h3A;
    buff [16'h00_f6] = 8'hF5;
    buff [16'h00_f7] = 8'h83;
    buff [16'h00_f8] = 8'hE4;
    buff [16'h00_f9] = 8'h93;
    buff [16'h00_fa] = 8'h22;
    buff [16'h00_fb] = 8'hBB;
    buff [16'h00_fc] = 8'h01;
    buff [16'h00_fd] = 8'h06;
    buff [16'h00_fe] = 8'h89;
    buff [16'h00_ff] = 8'h82;
    buff [16'h01_00] = 8'h8A;
    buff [16'h01_01] = 8'h83;
    buff [16'h01_02] = 8'hF0;
    buff [16'h01_03] = 8'h22;
    buff [16'h01_04] = 8'h50;
    buff [16'h01_05] = 8'h02;
    buff [16'h01_06] = 8'hF7;
    buff [16'h01_07] = 8'h22;
    buff [16'h01_08] = 8'hBB;
    buff [16'h01_09] = 8'hFE;
    buff [16'h01_0a] = 8'h01;
    buff [16'h01_0b] = 8'hF3;
    buff [16'h01_0c] = 8'h22;
    buff [16'h01_0d] = 8'hF8;
    buff [16'h01_0e] = 8'hBB;
    buff [16'h01_0f] = 8'h01;
    buff [16'h01_10] = 8'h0D;
    buff [16'h01_11] = 8'hE5;
    buff [16'h01_12] = 8'h82;
    buff [16'h01_13] = 8'h29;
    buff [16'h01_14] = 8'hF5;
    buff [16'h01_15] = 8'h82;
    buff [16'h01_16] = 8'hE5;
    buff [16'h01_17] = 8'h83;
    buff [16'h01_18] = 8'h3A;
    buff [16'h01_19] = 8'hF5;
    buff [16'h01_1a] = 8'h83;
    buff [16'h01_1b] = 8'hE8;
    buff [16'h01_1c] = 8'hF0;
    buff [16'h01_1d] = 8'h22;
    buff [16'h01_1e] = 8'h50;
    buff [16'h01_1f] = 8'h06;
    buff [16'h01_20] = 8'hE9;
    buff [16'h01_21] = 8'h25;
    buff [16'h01_22] = 8'h82;
    buff [16'h01_23] = 8'hC8;
    buff [16'h01_24] = 8'hF6;
    buff [16'h01_25] = 8'h22;
    buff [16'h01_26] = 8'hBB;
    buff [16'h01_27] = 8'hFE;
    buff [16'h01_28] = 8'h05;
    buff [16'h01_29] = 8'hE9;
    buff [16'h01_2a] = 8'h25;
    buff [16'h01_2b] = 8'h82;
    buff [16'h01_2c] = 8'hC8;
    buff [16'h01_2d] = 8'hF2;
    buff [16'h01_2e] = 8'h22;
end

always @(posedge clk)
begin
  data1 <= #1 buff [addr];
  data2 <= #1 buff [addr+1];
  data3 <= #1 buff [addr+2];
end

endmodule
