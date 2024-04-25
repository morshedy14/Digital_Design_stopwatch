module OurProject(u_en, nu_rst, u_dir, nu_add2, u_times2, u_div2, u_clk, clkdisp ,q);
input u_en, nu_rst, u_dir, u_times2, u_div2, u_clk, nu_add2;
wire u_rst, u_add2;

not n1(u_rst, nu_rst);
not n2(u_add2, nu_add2);

wire [13:0]qtemp;
wire flshmode;
output [15:0]q;
output clkdisp;
wire out_clk, out_dir, out_add2, out_en;
control_unit cu(u_en, u_dir, u_times2, u_div2, u_add2, u_clk, out_clk, out_dir, out_add2, out_en, flshmode);
counter cntr(out_en, u_rst, u_clk, out_dir, u_add2, qtemp);

muxtot mt(qtemp, flshmode, out_clk, q);
assign clkdisp = out_clk;

endmodule




///////////21 multiplexer/////////
module mux21(a0, a1, s, y);

	input s, a0, a1;
	output y;
	wire w1, w2, n_s;
	
	not not1(n_s, s);
	and and1(w1, s, a1);
	and and2(w2, n_s, a0);
	or or1(y, w1, w2);

endmodule
/////////////////////////////
module mux41(a0, a1, a2, a3, s0, s1, y);
input a0,a1,a2,a3,s0,s1; output y;
//if s0 s1 are 0; output a0
//  s0=1, s1=0; a1
// s0=0, s1=1; a2
// s0=1,s1=1; a3
wire w1, w2;
mux21 m1(a0, a1, s0, w1);
mux21 m2(a2, a3, s0, w2);
mux21 m3(w1, w2, s1, y);

endmodule


////output mux/////
module muxtot(qin1, s,clk ,qout);
input [13:0]qin1;
input s,clk;
reg [15:0]q_of5;
reg [15:0]q_flash;
assign q_of5 = {16'b0101010101010101};
output [15:0]qout;
and and0(q_flash[0], q_of5[0], clk);
and and1(q_flash[1], q_of5[1], clk);
and and2(q_flash[2], q_of5[2], clk);
and and3(q_flash[3], q_of5[3], clk);
and and4(q_flash[4], q_of5[4], clk);
and and5(q_flash[5], q_of5[5], clk);
and and6(q_flash[6], q_of5[6], clk);
and and7(q_flash[7], q_of5[7], clk);
and and8(q_flash[8], q_of5[8], clk);
and and9(q_flash[9], q_of5[9], clk);
and and10(q_flash[10], q_of5[10], clk);
and and11(q_flash[11], q_of5[11], clk);
and and12(q_flash[12], q_of5[12], clk);
and and13(q_flash[13], q_of5[13], clk);
and and14(q_flash[14], q_of5[14], clk);
and and15(q_flash[15], q_of5[15], clk);

mux21  m0(qin1[0],  q_flash[0], s, qout[0]);
mux21  m1(qin1[1],  q_flash[1], s, qout[1]);
mux21  m2(qin1[2],  q_flash[2], s, qout[2]);
mux21  m3(qin1[3],  q_flash[3], s, qout[3]);

mux21  m4(qin1[4],  q_flash[4], s, qout[4]);
mux21  m5(qin1[5],  q_flash[5], s, qout[5]);
mux21  m6(qin1[6],  q_flash[6], s, qout[6]);
mux21  m7(1'b0,     q_flash[7], s, qout[7]);

mux21  m8( qin1[7], q_flash[8], s, qout[8]);
mux21  m9( qin1[8], q_flash[9], s, qout[9]);
mux21 m10( qin1[9], q_flash[10],s, qout[10]);
mux21 m11( qin1[10],q_flash[11],s, qout[11]);

mux21 m12( qin1[11],q_flash[12],s, qout[12]);
mux21 m13( qin1[12],q_flash[13],s, qout[13]);
mux21 m14( qin1[13],q_flash[14],s, qout[14]);
mux21 m15( 1'b0,    q_flash[15],s, qout[15]);
endmodule
///////////////////

module tristate_buf(a0, s, y);
input a0, s; output y;
mux21 muxa(1'bZ, a0, s, y);
endmodule


//demux////////////////////////
module demux21(In, Sel, A, B);
input In, Sel; output A,B;
wire  nSel;

not notSel(nSel, Sel);
and selA(A, nSel, In);
and selB(B,  Sel, In);

endmodule
////////////////////////////////////////////////////////////////////

//////////enot/////////////////
module enot(en, a, y);
input en, a;
output y;
wire n_a;
not n1(n_a, a);
mux21 mymux(a, n_a, en, y);

endmodule
//////////////////////////
//
module minVal_comp(a, y);
input [13:0]a;
output y;
reg [13:0]min = {14'b00100000100000};
wire [13:0]w;

xnor xnor0(w[0], min[0], a[0]);
xnor xnor1(w[1], min[1], a[1]);
xnor xnor2(w[2], min[2], a[2]);
xnor xnor3(w[3], min[3], a[3]);
xnor xnor4(w[4], min[4], a[4]);
xnor xnor5(w[5], min[5], a[5]);
xnor xnor6(w[6], min[6], a[6]);
xnor xnor7(w[7], min[7], a[7]);
xnor xnor8(w[8], min[8], a[8]);
xnor xnor9(w[9], min[9], a[9]);
xnor xnor10(w[10], min[10], a[10]);
xnor xnor11(w[11], min[11], a[11]);
xnor xnor12(w[12], min[12], a[12]);
xnor xnor13(w[13], min[13], a[13]);

and and1(y, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7], w[8], w[9], w[10], w[11], w[12], w[13]);


endmodule

module maxVal_comp(a, y);
input [13:0]a;
output y;
reg [13:0]max = {14'b10010010110000};
wire [13:0]w;

xnor xnor0(w[0], max[0], a[0]);
xnor xnor1(w[1], max[1], a[1]);
xnor xnor2(w[2], max[2], a[2]);
xnor xnor3(w[3], max[3], a[3]);
xnor xnor4(w[4], max[4], a[4]);
xnor xnor5(w[5], max[5], a[5]);
xnor xnor6(w[6], max[6], a[6]);
xnor xnor7(w[7], max[7], a[7]);
xnor xnor8(w[8], max[8], a[8]);
xnor xnor9(w[9], max[9], a[9]);
xnor xnor10(w[10], max[10], a[10]);
xnor xnor11(w[11], max[11], a[11]);
xnor xnor12(w[12], max[12], a[12]);
xnor xnor13(w[13], max[13], a[13]);

and and1(y, w[0], w[1], w[2], w[3], w[4], w[5], w[6], w[7], w[8], w[9], w[10], w[11], w[12], w[13]);

endmodule

module _4inpcomp(y, a,b);


input [3:0]a; input [3:0]b; output y;
wire [3:0]w;
xnor xnor0(w[0], b[0], a[0]);
xnor xnor1(w[1], b[1], a[1]);
xnor xnor2(w[2], b[2], a[2]);
xnor xnor3(w[3], b[3], a[3]);
and and1(y, w[0], w[1],w[2],w[3]);
endmodule

module _3inpcomp(y, a,b);
input [2:0]a; input [2:0]b; output y;
wire [2:0]w;
xnor xnor0(w[0], b[0], a[0]);
xnor xnor1(w[1], b[1], a[1]);
xnor xnor2(w[2], b[2], a[2]);
and and1(y, w[0], w[1],w[2]);
endmodule


module max_of(en, dig1, dig2, dig3, dig4, y);
input [2:0] dig1; input [2:0] dig3;
input [3:0] dig2; input [3:0] dig4;
input en;
//input en;
output y;
//overflow cases//////////////
reg [0:2]ofdig1 = {3'b100};
reg [0:3]ofdig2_1 = {4'b0111};
reg [0:3]ofdig2_2 = {4'b1000};
reg [0:3]ofdig2_3 = {4'b1001};
reg [0:2]ofdig3_1 = {3'b101};
reg [0:2]ofdig3_2 = {3'b100};
reg [0:2]ofdig3_3 = {3'b011};
reg [0:3]ofdig4 = {4'b0000};
////////////////////////////////
wire [7:0]w;
//comparators to overflow cases///
_3inpcomp c0(w[0], dig1, ofdig1); //first dig:::::4
_4inpcomp c1(w[1], dig2, ofdig2_1); //second dig::"7
_4inpcomp c2(w[2], dig2, ofdig2_2);//second dig:::"8
_4inpcomp c3(w[3], dig2, ofdig2_3);//second dig:::"9
_3inpcomp c4(w[4],dig3, ofdig3_1); //third dig :::""5
_3inpcomp c5(w[5],dig3, ofdig3_2); //third dig::::""4
_3inpcomp c6(w[6],dig3, ofdig3_3); //third dig::::""3
_4inpcomp c7(w[7], dig4, ofdig4); //fourth dig::::"""0
/////////////////////////////////

//comb logic////////////////////
wire w_n_one;
wire ytemp;
wire of48, of49, ofm_s473;
not not1(w_n_one, w[7]);
and and1(of48, w[0], w[2]);
and and2(of49, w[0], w[3]);
and and3(ofm_s473, w[0], w[1], w[6]);
wire or1_out, or2_out, mout, w_andf_o;
or or1(or1_out, w[1], w[2], w[3]);
or or2(or2_out, w[4], w[5], w[6]);
mux21 mux_1(1'b1, w_n_one, ofm_s473, mout);
and andf(w_andf_o, or1_out, or2_out, mout, w[0]);
or orf(ytemp, of48, of49, w_andf_o);
mux21 mux_f(1'b0, ytemp, en, y);

endmodule




module clk_div_logic(two, half, hz1, hz2, hz4);
input logic two, half;
output logic hz1, hz2, hz4;
wire w1, w2;
wire n_two, n_half;
// those outputs will determine what the output 
//of out clock divider will be, this can be implimented
//later on with a simple mux switch or so on

not n1(n_two, two);
not n2(n_half, half);

and a1(w1, two, half);
and a11(w2, n_two, n_half);
or o1(hz2,w1,w2);

and a2(hz4,two, n_half);
and a3(hz1,n_two, half);

endmodule


module count_mode(en, cm, p_cm, y);
input en, cm, p_cm;
output y;
wire n_en, w1, w2;
not n1(n_en, en);
and a1(w1, n_en, p_cm);
and a2(w2, en, cm);
or o1(y, w1,w2);
endmodule



module min_of(en, dig1, dig2, dig3, dig4, y);
input [2:0] dig1; input [2:0] dig3;
input [3:0] dig2; input [3:0] dig4;
input en;
output y;
wire wand1, wand2, wand3, w_or1, w_or2;
reg [0:2]ofdig1 = {3'b001}; //dig 1

reg [0:3]ofdig2_1 = {4'b0001};//dig 2
reg [0:3]ofdig2_2 = {4'b0010};
reg [0:3]ofdig2_3 = {4'b0000};

reg [0:2]ofdig3_1 = {3'b000};//dig 3
reg [0:2]ofdig3_2 = {3'b001};
wire w1,w2,w3,w4,w5,w6;

_3inpcomp d1(w1, dig1, ofdig1);

_4inpcomp d21(w2, dig2, ofdig2_1);
_4inpcomp d22(w3, dig2, ofdig2_2);
_4inpcomp d23(w4, dig2, ofdig2_3);

_3inpcomp d31(w5, dig3, ofdig3_1);
_3inpcomp d32(w6, dig3, ofdig3_2);

wire yt;
or or1(w_or1, w2, w3, w4);
or or2(w_or2, w5, w6);
and and1(wand1, w_or1, w_or2, w1);

and and2(wand2, w1, w2);
and and3(wand3, w1, w4);

or or3(yt, wand1, wand2, wand3);
mux21 mux(1'b0, yt, en, y);


endmodule


 module fulladder(sum, a, b, cin, cout);
 input a, b, cin;
 output cout, sum;
wire [2:0]w;
xor xor1(w[0], a, b);
xor xorf(sum, w[0], cin);
and and1(w[1], w[0], cin);
and and2(w[2], a, b);
or or_fin(cout, w[1], w[2]);
 endmodule

module add2(in, out);
input [6:0]in; output[6:0]out;
wire [7:0]w; wire [7:0]cinripple; wire [7:0]fcinripple;
wire grnd, wand1, wand2, wor1, wand3, wand4, wor2, hidden_out;
assign grnd = 1'b0;

fulladder i0(w[0], grnd, in[0], grnd,         cinripple[0]);
fulladder i1(w[1], 1'b1, in[1], cinripple[0], cinripple[1]);
fulladder i2(w[2], grnd, in[2], cinripple[1], cinripple[2]);
fulladder i3(w[3], grnd, in[3], cinripple[2], cinripple[3]);

and and1(wand1, w[3], w[1]);
and and2(wand2, w[3], w[2]);
or or1(wor1 ,wand1, wand2, cinripple[3]);

fulladder i4(w[4], grnd, in[4], wor1,         cinripple[4]);
fulladder i5(w[5], grnd, in[5], cinripple[4], cinripple[5]);
fulladder i6(w[6], grnd, in[6], cinripple[5], cinripple[6]);
fulladder i7(w[7], grnd, grnd , cinripple[6], cinripple[7]);

and and3(wand3, w[7], w[5]);
and and4(wand4, w[7], w[6]);
or or2(wor2 ,wand3, wand4, cinripple[7]);


fulladder o0(out[0], grnd, w[0], grnd,          fcinripple[0]);
fulladder o1(out[1], wor1, w[1], fcinripple[0], fcinripple[1]);
fulladder o2(out[2], wor1, w[2], fcinripple[1], fcinripple[2]);
fulladder o3(out[3], grnd, w[3], fcinripple[2], fcinripple[3]);


fulladder o4(out[4],     grnd, w[4],     grnd,          fcinripple[4]);
fulladder o5(out[5],     wor2, w[5],     fcinripple[4], fcinripple[5]);
fulladder o6(out[6],     wor2, w[6],     fcinripple[5], fcinripple[6]);
fulladder o7(hidden_out, grnd, w[7], fcinripple[6], fcinripple[7]);


endmodule


module sub2(in, out);
input [6:0]in; output[6:0]out;
wire [7:0]w; wire [7:0]cinripple; wire [7:0]fcinripple;
wire grnd, wand1, wand2, wor1, wand3, wand4, wor2, hidden_out, pwr;
assign grnd = 1'b0;
assign pwr = 1'b1;

fulladder i0(w[0], pwr, in[0], pwr,         cinripple[0]);
fulladder i1(w[1], pwr, in[1], cinripple[0], cinripple[1]);
fulladder i2(w[2], pwr, in[2], cinripple[1], cinripple[2]);
fulladder i3(w[3], grnd, in[3],  cinripple[2], cinripple[3]);

and and1(wand1, w[3], w[1]);
and and2(wand2, w[3], w[2]);
or or1(wor1 ,wand1, wand2, cinripple[3]);

fulladder i4(w[4], pwr, in[4], wor1,         cinripple[4]);
fulladder i5(w[5], grnd, in[5], cinripple[4], cinripple[5]);
fulladder i6(w[6], grnd, in[6], cinripple[5], cinripple[6]);
fulladder i7(w[7], pwr, grnd , cinripple[6], cinripple[7]);

and and3(wand3, w[7], w[5]);
and and4(wand4, w[7], w[6]);
or or2(wor2 ,wand3, wand4, cinripple[7]);


fulladder o0(out[0], grnd, w[0], grnd,          fcinripple[0]);
fulladder o1(out[1], wor1, w[1], fcinripple[0], fcinripple[1]);
fulladder o2(out[2], wor1, w[2], fcinripple[1], fcinripple[2]);
fulladder o3(out[3], grnd, w[3], fcinripple[2], fcinripple[3]);


fulladder o4(out[4],     grnd, w[4],     grnd,          fcinripple[4]);
fulladder o5(out[5],     wor2, w[5],     fcinripple[4], fcinripple[5]);
fulladder o6(out[6],     wor2, w[6],     fcinripple[5], fcinripple[6]);
fulladder o7(hidden_out, grnd, w[7], fcinripple[6], fcinripple[7]);


endmodule

module dflipflop (d,set,reset,enable,clk,q,q_bar);
 input d; 
 input set; 
 input reset; 
 input enable; 
 input clk; 
 output q;
 output q_bar;

  reg q_reg, q_bar_reg;

  always_ff @(posedge clk or posedge reset or posedge set) begin
    if (reset) begin
      q_reg <= 0;
      q_bar_reg <= 1;
    end else if (set) begin
      q_reg <= 1;
      q_bar_reg <= 0;
    end else if (enable) begin
      q_reg <= d;
      q_bar_reg <= !d;
    end
  end

  assign q = q_reg;
  assign q_bar = q_bar_reg;

  
endmodule



module bcd_decoder(in, out);
output [6:0]out;
input [0:3]in;
wire [3:0]n_in;
not notin1(n_in[0], in[0]);
not notin2(n_in[1], in[1]);
not notin3(n_in[2], in[2]);
not notin4(n_in[3], in[3]);
wire bd, b_d_, cd, c_d_, cd_, b_c, bc_, bd_, bc_d;

and and1(bd, in[1], in[3]);
and and2(b_d_, n_in[1], n_in[3]);
and and3(cd, in[2], in[3]);
and and4(c_d_,n_in[2] ,n_in[3]);
and and5(cd_,in[2],n_in[3]);
and and6(b_c,n_in[1], in[2]);
and and7(bc_,in[1],n_in[2]);
and and8(bd_,in[1],n_in[3]);
and and9(bc_d,in[1],in[3],n_in[2]);
or ora(out[6],in[0],in[2], b_d_);
or orb(out[5],n_in[1],cd, c_d_);
or orc(out[4],in[1],in[3],n_in[2]);
or ord(out[3],b_d_,cd_,b_c,in[0],bc_d);
or ore(out[2],b_d_,cd_);
or orf(out[1],in[0],c_d_,bc_,bd_);
or org(out[0],in[0],bc_,b_c,cd_);


endmodule


module clk_dvdr (input clk_in, input [1:0]speed , output reg c_out);
reg [25:0] counter;

initial begin 
c_out = 0;
end


// wire cin;
// pll mypll(clk_in, cin);
always @ (posedge clk_in)
begin
counter = counter + 1'b1;

case (speed)
2'b00:
begin 
if (counter == 25000000)
  begin
    c_out = ~c_out;
    counter = 0;
  end
end
2'b01:
begin
 if (counter == 50000000)
  begin
    c_out = ~c_out;
    counter = 0;
  end
end
2'b10:
begin
 if (counter == 12500000)
  begin
    c_out = ~c_out;
    counter = 0;
  end
end
2'b11:
begin 
 if (counter == 50000000)
  begin
    c_out = ~c_out;
    counter = 0;
  end
end
default:
begin
 if (counter == 50000000)
  begin
    c_out = ~c_out;
    counter = 0;
  end
end
 
endcase
end
endmodule



module control_unit(en, dir,times2, div2,add2 ,clk_in, out_clk, out_dir, out_add2, out_en, flshmode);
input en, dir, times2, div2, clk_in, add2;
output out_clk, out_dir, out_add2, out_en, flshmode;
wire w_add2, wn_add2, wn_en, dircheck, n_out_dir;

//add 2 doesnt spam-add when holding btn
dflipflop dffadd2(add2, 1'b0, 1'b0, 1'b1, clk_in, w_add2, wn_add2);
and andadd2(out_add2, wn_add2, add2);


//cant change direction when unpaused
not noten(wn_en, en);
dflipflop changedir(dir, 1'b0, 1'b0, wn_en, clk_in, out_dir, n_out_dir);

//pauses when both up 2x and 0.5x are down
nand nanddir(dircheck, div2, times2);
and and_en_dir(out_en, dircheck, en);

//
wire[1:0] speed;
assign speed[0] = div2;
assign speed[1] = times2;


and andflsh(flshmode, div2, times2);
////////////////////make flash reg
clk_dvdr myclkdiv(clk_in, speed , out_clk);



endmodule


 
module counter(en, rst, clk, direction, add2m, q);
input en, rst, clk, direction, add2m;
output [13:0]q;
wire grnd, pwr, dirgrnd;
assign grnd = 1'b0; assign pwr = 1'b1;
enot enotgrnd(direction, grnd, dirgrnd); //enabled ground for layer1
//the constuction of this code will treat the schemiatic as layers
//the furtherest left components are "layer 1"
////////////////////////////
//first layer enots//
wire [9:0]w_enot_l1;
enot enotL1_0(direction, q[0], w_enot_l1[0]);
enot enotL1_1(direction, q[1], w_enot_l1[1]);
enot enotL1_2(direction, q[2], w_enot_l1[2]);
enot enotL1_3(direction, q[4], w_enot_l1[3]);
enot enotL1_4(direction, q[5], w_enot_l1[4]);
enot enotL1_5(direction, q[7], w_enot_l1[5]);
enot enotL1_6(direction, q[8], w_enot_l1[6]);
enot enotL1_7(direction, q[9], w_enot_l1[7]);
enot enotL1_8(direction, q[11], w_enot_l1[8]);
enot enotL1_9(direction, q[12], w_enot_l1[9]);
//

//last layer, internal resets
wire m0_and, m1_and, m2_and, m3_and, h1_and, h2_and, h3_and;
wire en_q0, en_q3, en_q4, en_q6, en_q7, en_q10, en_q13;
wire n_q1, n_q2, n_q5, n_q8, n_q9, n_q11, n_q12;
enot enotLf_0(direction, q[0], en_q0);
enot enotLf_1(direction, q[3], en_q3);
enot enotLf_2(direction, q[4], en_q4);
enot enotLf_3(direction, q[6], en_q6);
enot enotLf_4(direction, q[7], en_q7);
enot enotLf_5(direction, q[10], en_q10);
enot enotLf_6(direction, q[13], en_q13);
not notLf_0(n_q1, q[1]);
not notLf_1(n_q2, q[2]);
not notLf_2(n_q5, q[5]);
not notLf_3(n_q8, q[8]);
not notLf_4(n_q9, q[9]);
not notLf_5(n_q11, q[11]);
not notLf_6(n_q12, q[12]);

and andLrst_00(m0_and, en_q0, n_q1, n_q2, en_q3);
and andLrst_11(h1_and, en_q4, n_q5, en_q6);
and andLrst_12(h2_and, en_q7, n_q8, n_q9, en_q10);
and andLrst_13(h3_and, n_q11, n_q12, en_q13);

and andLrst_01(m1_and, m0_and, h1_and);
and andLrst_02(m2_and, m1_and, h2_and);
and andLrst_03(m3_and, m2_and, h3_and);
//


//first layer ands//
wire [9:0]w_and_l1;

and andL1_0(w_and_l1[0], w_enot_l1[0], en);
and andL1_1(w_and_l1[1], w_enot_l1[1], w_and_l1[0]);
and andL1_2(w_and_l1[2], w_enot_l1[2], w_and_l1[1]);

and andL1_3(w_and_l1[3], w_enot_l1[3], m0_and);

and andL1_4(w_and_l1[4], w_enot_l1[4], w_and_l1[3]);

and andL1_5(w_and_l1[5], w_enot_l1[5], m1_and);

and andL1_6(w_and_l1[6], w_enot_l1[6], w_and_l1[5]);
and andL1_7(w_and_l1[7], w_enot_l1[7], w_and_l1[6]);

and andL1_8(w_and_l1[8], w_enot_l1[8], m2_and);

and andL1_9(w_and_l1[9], w_enot_l1[9], w_and_l1[8]);
//

//first layer xors
wire [13:0]w_xor_l1;
xor xorL1_0(w_xor_l1[0], q[0],    en);
xor xorL1_1(w_xor_l1[1], q[1],    w_and_l1[0]);
xor xorL1_2(w_xor_l1[2], q[2],    w_and_l1[1]);
xor xorL1_3(w_xor_l1[3], q[3],    w_and_l1[2]);

xor xorL1_4(w_xor_l1[4], q[4],    m0_and);

xor xorL1_5(w_xor_l1[5], q[5],    w_and_l1[3]);
xor xorL1_6(w_xor_l1[6], q[6],    w_and_l1[4]);

xor xorL1_7(w_xor_l1[7], q[7],    m1_and);

xor xorL1_8(w_xor_l1[8], q[8],    w_and_l1[5]);
xor xorL1_9(w_xor_l1[9], q[9],    w_and_l1[6]);
xor xorL1_10(w_xor_l1[10], q[10], w_and_l1[7]);

xor xorL1_11(w_xor_l1[11], q[11], m2_and);

xor xorL1_12(w_xor_l1[12], q[12], w_and_l1[8]);
xor xorL1_13(w_xor_l1[13], q[13], w_and_l1[9]);
//

//first layer muxes
wire [13:0]w_mux_l1;
mux21 mux21L1_0(w_xor_l1[0],   dirgrnd, m0_and, w_mux_l1[0]);
mux21 mux21L1_1(w_xor_l1[1],   grnd,    m0_and, w_mux_l1[1]);
mux21 mux21L1_2(w_xor_l1[2],   grnd,    m0_and, w_mux_l1[2]);
mux21 mux21L1_3(w_xor_l1[3],   dirgrnd, m0_and, w_mux_l1[3]);

mux21 mux21L1_4(w_xor_l1[4],   grnd,    m1_and, w_mux_l1[4]);
mux21 mux21L1_5(w_xor_l1[5],   dirgrnd, m1_and, w_mux_l1[5]);
mux21 mux21L1_6(w_xor_l1[6],   dirgrnd, m1_and, w_mux_l1[6]);

mux21 mux21L1_7(w_xor_l1[7],   dirgrnd, m2_and, w_mux_l1[7]);
mux21 mux21L1_8(w_xor_l1[8],   grnd,    m2_and, w_mux_l1[8]);
mux21 mux21L1_9(w_xor_l1[9],   grnd,    m2_and, w_mux_l1[9]);
mux21 mux21L1_10(w_xor_l1[10], dirgrnd, m2_and, w_mux_l1[10]);

mux21 mux21L1_11(w_xor_l1[11], grnd,    m3_and, w_mux_l1[11]);
mux21 mux21L1_12(w_xor_l1[12], grnd,    m3_and, w_mux_l1[12]);
mux21 mux21L1_13(w_xor_l1[13], dirgrnd, m3_and, w_mux_l1[13]);

//reset comb//////////////////////////////////
wire w_up_ofdet_en, w_down_ofdet_en, notdir, maxofr, minofr;
wire uprst, downrst;
wire t_downrst, t_uprst;
demux21 rstlgc(rst, direction, t_uprst, t_downrst);
or orrstd(downrst, t_downrst, maxofr);
or orrstu(uprst, t_uprst, minofr);

//reset when upof//
/////////////////////////////////////////////

//stop at max/min, stop at overflow
wire minreached, maxreached;
wire n_minreached, n_maxreached;
wire minmaxreached, int_en;
not notmin(n_minreached, minreached);
not notmax(n_maxreached, maxreached);
minVal_comp mxvc(q, minreached);
maxVal_comp mnvc(q, maxreached);
mux21 muxmaxmin(n_maxreached, n_minreached, direction, minmaxreached);
wire n_minofr, n_maxofr;
not nmaxo(n_maxofr, maxofr);
not nmino(n_minofr, minofr);
and andminmax(int_en, minmaxreached, n_maxofr , n_minofr);
//

//second layer muxes
wire [6:0]w_mux_l2; wire [6:0]qp2; wire [6:0]qm2;
wire updownrst, double_en;

mux41 mux41L2_0(w_mux_l1[7],  qp2[0], qm2[0], w_mux_l1[7],  w_up_ofdet_en, w_down_ofdet_en, w_mux_l2[0]);
mux41 mux41L2_1(w_mux_l1[8],  qp2[1], qm2[1], w_mux_l1[8],  w_up_ofdet_en, w_down_ofdet_en, w_mux_l2[1]);
mux41 mux41L2_2(w_mux_l1[9],  qp2[2], qm2[2], w_mux_l1[9],  w_up_ofdet_en, w_down_ofdet_en, w_mux_l2[2]);
mux41 mux41L2_3(w_mux_l1[10], qp2[3], qm2[3], w_mux_l1[10], w_up_ofdet_en, w_down_ofdet_en, w_mux_l2[3]);
mux41 mux41L2_4(w_mux_l1[11], qp2[4], qm2[4], w_mux_l1[11], w_up_ofdet_en, w_down_ofdet_en, w_mux_l2[4]);
mux41 mux41L2_5(w_mux_l1[12], qp2[5], qm2[5], w_mux_l1[12], w_up_ofdet_en, w_down_ofdet_en, w_mux_l2[5]);
mux41 mux41L2_6(w_mux_l1[13], qp2[6], qm2[6], w_mux_l1[13], w_up_ofdet_en, w_down_ofdet_en, w_mux_l2[6]);
//

//d-flipflops

wire qbar[13:0];
wire w_dff1in; 
and (double_en, int_en, en);
//tristate_buf trsb(w_mux_l1[0], en, w_dff1in);
or orrst(updownrst, uprst, downrst);

dflipflop dff1(w_mux_l1[0], grnd, updownrst, double_en, clk, q[0], qbar[0]);
dflipflop dff2(w_mux_l1[1], grnd, updownrst, int_en, clk, q[1], qbar[1]);
dflipflop dff3(w_mux_l1[2], grnd, updownrst, int_en, clk, q[2], qbar[2]);
dflipflop dff4(w_mux_l1[3], grnd, updownrst, int_en, clk, q[3], qbar[3]);
dflipflop dff5(w_mux_l1[4], downrst,  uprst, int_en, clk, q[4], qbar[4]);
dflipflop dff6(w_mux_l1[5], updownrst, grnd, int_en, clk, q[5], qbar[5]);
dflipflop dff7(w_mux_l1[6], grnd, updownrst, int_en, clk, q[6], qbar[6]);
dflipflop dff8(w_mux_l2[0], downrst, uprst,  int_en, clk, q[7], qbar[7]);
dflipflop dff9(w_mux_l2[1], grnd, updownrst, int_en, clk, q[8], qbar[8]);
dflipflop dff10(w_mux_l2[2], grnd, updownrst, int_en, clk, q[9], qbar[9]);
dflipflop dff11(w_mux_l2[3], downrst,  uprst, int_en, clk, q[10], qbar[10]);
dflipflop dff12(w_mux_l2[4], uprst,  downrst, int_en, clk, q[11], qbar[11]);
dflipflop dff13(w_mux_l2[5], grnd, updownrst, int_en, clk, q[12], qbar[12]);
dflipflop dff14(w_mux_l2[6], downrst,  uprst, int_en, clk, q[13], qbar[13]);

//adder



//add2
add2 addtwo(q[13:7] ,qp2);
sub2 subtwo(q[13:7] ,qm2);
not notdirnot(notdir, direction);
and andmin2of(w_down_ofdet_en ,direction,add2m);
and andup2of(w_up_ofdet_en, notdir , add2m);
max_of mxof(w_up_ofdet_en, q[13:11], q[10:7], q[6:4], q[3:0], maxofr);
min_of mnof(w_down_ofdet_en, q[13:11], q[10:7], q[6:4], q[3:0], minofr);
//
// wire [15:0]qdisplay;
// assign qdisplay[0] = q[0];
// assign qdisplay[1] = q[1];
// assign qdisplay[2] = q[2];
// assign qdisplay[3] = q[3];

// assign qdisplay[4] = q[4];
// assign qdisplay[5] = q[5];
// assign qdisplay[6] = q[6];
// assign qdisplay[7] = 1'b0;

// assign qdisplay[8] =  q[7];
// assign qdisplay[9] =  q[8];
// assign qdisplay[10] = q[9];
// assign qdisplay[11] = q[10];

// assign qdisplay[12] = q[11];
// assign qdisplay[13] = q[12];
// assign qdisplay[14] = q[13];
// assign qdisplay[15] = 1'b0;



endmodule

 