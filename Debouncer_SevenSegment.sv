MEMORY SYSTEMVERILOG

`timescale 1ns / 1ps
module Memory #(parameter N = 4, M = 8)( input clk, writeEnable, [N-1:0]writeAddr,
[M-1:0]writeData, [N-1:0]readAddr1, [N-1:0]readAddr2, output [M-1:0]readData1,
[M-1:0]readData2 );
logic [M-1:0]memory[N-1:0];
always_ff @(posedge clk)
begin
if( writeEnable )
memory[writeAddr] <= writeData;
end
assign readData1 = memory[readAddr1];
assign readData2 = memory[readAddr2];
endmodule

MEMORY TESTBENCH

timescale 1ns / 1ps
module Memory_tb;
logic clk, writeEnable;
logic [15:0]writeAddr, readAddr1, readAddr2;
logic [7:0]writeData;
wire [7:0]readData1, readData2;
Memory uut( .clk(clk), .writeEnable(writeEnable), .writeAddr(writeAddr), .writeData(writeData),
.readAddr1(readAddr1), .readAddr2(readAddr2), .readData1(readData1), .readData2(readData2)
);
initial
begin
#300;
writeEnable = 1; writeAddr = 1'h0; writeData = 8'b00000011;
readAddr1 = 1'h3; readAddr2 = 1'h0;
#70; writeEnable = 0; writeAddr = 1'h0; writeData = 8'b10100011;
readAddr1 = 1'h3; readAddr2 = 1'h0;
#70;writeEnable = 1; writeAddr = 1'h3; writeData = 8'b10101011;
readAddr1 = 1'h3; readAddr2 = 1'h0;
#70;writeEnable = 1; writeAddr = 1'h5; writeData = 8'b00000011;
readAddr1 = 1'h5; readAddr2 = 1'h0;
#70;writeEnable = 1; writeAddr = 1'h4; writeData = 8'b00000011;
readAddr1 = 1'h3; readAddr2 = 1'h4;
end
always
begin
#30; clk <= 1;
#30; clk <= 0;
end
endmodule

REDUCESUM SYSTEMVERILOG

`timescale 1ns / 1ps
module ReduceSum#(parameter M = 8, N = 12)(
input clk,
buttonPushed,
input [M-1:0]value,
value1,
value2,
value3,
value4,
value5,
value6,
value7,
value8,
value9,
valueA,
valueB,
valueC,
valueD,
valueE,
valueF,
output logic [N-1:0]sum
);
always_ff @(posedge clk)
begin
if( buttonPushed )
sum <= value + value1 + value2 + value3 + value4 + value5 +
value6 + value7 + value8 + value9 + valueA + valueB + valueC + valueD
+ valueE + valueF;
end
endmodule

REDUCE TESTBENCH

module ReduceSum_tb;
logic clk, buttonPushed;
logic [7:0] value,
value1,
value2,
value3,
value4,
value5,
value6,
value7,
value8,
value9,
valueA,
valueB,
valueC,
valueD,
valueE,
valueF;
wire [11:0]sum;
ReduceSum uut(
.clk(clk),
.buttonPushed(buttonPushed),
.value(value),
.value1(value1),
.value2(value2),
.value3(value3),
.value4(value4),
.value5(value5),
.value6(value6),
.value7(value7),
.value8(value8),
.value9(value9),
.valueA(valueA),
.valueB(valueB),
.valueC(valueC),
.valueD(valueD),
.valueE(valueE),
.valueF(valueF),
.sum(sum)
);
initial
begin
#300;
buttonPushed = 1;
value = 8'b00110011 ; value1 = 8'b00110011 ; value2 =
8'b00110011 ; value3 = 8'b00110011 ;
value4 = 8'b00110011 ; value5 = 8'b00110011 ; value6 =
8'b00110011 ; value7 = 8'b00110011 ;
value8 = 8'b00110011 ; value9 = 8'b00110011 ; valueA =
8'b00110011 ; valueB = 8'b00110011 ;
valueC = 8'b00110011 ; valueD = 8'b00110011 ; valueE =
8'b00110011 ; valueF = 8'b00110011 ;
#70;
buttonPushed = 0;
value = 8'b00000000 ; value1 = 8'b00110011 ; value2 =
8'b00110011 ; value3 = 8'b00110011 ;
value4 = 8'b00110011 ; value5 = 8'b0011000 ; value6 =
8'b01110011 ; value7 = 8'b00110011 ;
value8 = 8'b00110011 ; value9 = 8'b00110011 ; valueA =
8'b10110011 ; valueB = 8'b00110011 ;
valueC = 8'b00111011 ; valueD = 8'b00111011 ; valueE =
8'b00010011 ; valueF = 8'b00110011 ;
#70;
buttonPushed = 1;
value = 8'b11111111 ; value1 = 8'b11111111 ; value2 =
8'b11111111 ; value3 = 8'b11111111 ;
value4 = 8'b11111111 ; value5 = 8'b11111111 ; value6 =
8'b11111111 ; value7 = 8'b11111111 ;
value8 = 8'b11111111 ; value9 = 8'b11111111 ; valueA =
8'b11111111 ; valueB = 8'b11111111 ;
valueC = 8'b11111111 ; valueD = 8'b11111111 ; valueE =
8'b11111111 ; valueF = 8'b11111111 ;
#70;
buttonPushed = 0;
value = 8'b00110011 ; value1 = 8'b00111011 ; value2 =
8'b00010011 ; value3 = 8'b10010011 ;
value4 = 8'b00111011 ; value5 = 8'b00110011 ; value6 =
8'b00000011 ; value7 = 8'b00110011 ;
value8 = 8'b00110111 ; value9 = 8'b01110011 ; valueA =
8'b00110011 ; valueB = 8'b00010011 ;
valueC = 8'b00110111 ; valueD = 8'b10110011 ; valueE =
8'b00110011 ; valueF = 8'b00100011 ;
end
always
begin
#30; clk <= 1;
#30; clk <= 0;
end
endmodule

TOPDESIGN SYSTEMVERILOG

`timescale 1ns / 1ps
module TopDesign(
input clk,
displayPreviousData,
displayNextData,
enterData,
runReduceSum,
reset,
input [2:0]inputAddr,
input [7:0]inputData,
output logic [11:0]summation,
output logic dp,
output logic [6:0] seg,
output logic [3:0] an
);
logic displayPreviousDataD, displayNextDataD, enterDataD,
runReduceSumD;
Debouncer debouncer( clk, displayPreviousData,
displayPreviousDataD );
Debouncer debouncer1( clk, displayNextData, displayNextDataD );
Debouncer debouncer2( clk, enterData, enterDataD );
Debouncer debouncer3( clk, runReduceSum, runReduceSumD );
logic seg, dp, an;
logic [7:0] ramData;
logic [3:0] addr;
logic [7:0] data;
SevenSegment sevenSegment( clk, addr, 16, ramData[7:4],
ramData[3:0], seg, dp, an );
logic [7:0]readData1, readData2;
logic [3:0]readAddr1, readAddr2;
Memory memory( clk, enterDataD, addr, data, readAddr1, readAddr1,
readData1, readData2 );
logic [7:0] value,
value1,
value2,
value3,
value4,
value5,
value6,
value7,
value8,
value9,
valueA,
valueB,
valueC,
valueD,
valueE,
valueF;
ReduceSum reduceSum(
clk,
enterDataD,
value,
value1,
value2,
value3,
value4,
value5,
value6,
value7,
value8,
value9,
valueA,
valueB,
valueC,
valueD,
valueE,
valueF,
summation
);
logic currentAddr;
typedef enum logic [2:0] { init, waitInput, sum, displayPrevious,
displayNext, enteringData } State;
State currentState, nextState;
always_ff @(posedge clk)
if( reset ) currentState <= init;
else currentState <= nextState;
always_comb
case( currentState )
init: nextState = waitInput;
waitInput:
if ( displayPreviousData ) nextState = displayPrevious;
else if ( displayNextData ) nextState = displayNext;
else if ( enterData ) nextState = enteringData;
else if ( runReduceSum ) nextState = sum;
sum: nextState = waitInput;
displayPrevious: nextState = waitInput;
displayNext: nextState = waitInput;
enteringData: nextState = waitInput;
endcase
always_comb
case( currentState )
init:
begin
addr <= 0;
currentAddr <= 0;
end
waitInput:
begin
end
sum:
begin
readAddr1 <= 4'b0000;
readAddr2 <= 4'b0001;
value <= readData1;
value1 <= readData2;
//-----------------------
readAddr1 <= 4'b0010;
readAddr2 <= 4'b0011;
value2 <= readData1;
value3 <= readData2;
//-----------------------
readAddr1 <= 4'b0100;
readAddr2 <= 4'b0101;
value4 <= readData1;
value5 <= readData2;
//-----------------------
readAddr1 <= 4'b0110;
readAddr2 <= 4'b0111;
value6 <= readData1;
value7 <= readData2;
//-----------------------
readAddr1 <= 4'b1000;
readAddr2 <= 4'b1001;
value8 <= readData1;
value9 <= readData2;
//-----------------------
readAddr1 <= 4'b1010;
readAddr2 <= 4'b1011;
valueA <= readData1;
valueB <= readData2;
//-----------------------
readAddr1 <= 4'b1100;
readAddr2 <= 4'b1101;
valueC <= readData1;
valueD <= readData2;
//-----------------------
readAddr1 <= 4'b1110;
readAddr2 <= 4'b1111;
valueE <= readData1;
valueF <= readData2;
end
displayPrevious:
begin
if( addr == 4'b0000 )
addr <= 4'b1101;
else
addr <= addr - 4'b0001;
readAddr1 <= addr;
ramData <= readData1;
end
displayNext:
begin
if( addr == 4'b1101 )
addr <= 4'b0000;
else
addr <= addr + 4'b0001;
readAddr1 <= addr;
ramData <= readData1;
end
enteringData:
begin
addr <= inputAddr;
ramData <= inputData;
end
endcase
endmodule
