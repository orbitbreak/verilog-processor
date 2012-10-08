// Verilog 8-bit processor (four 6-bit registers)
// Supports MOV, EXCG, STA, ADD, SUB, HLT, and NOP
// Utilizes external memory
// Test bench module at end of code

module veriproc(reset,clock);
  input clock, reset;				//cpu clock and reset
  wire read, write;				//control for when to read/write to memory
  reg [3:0] address;				//address register
  wire [3:0] instruction, destination, source;  //IR values after decoded
  reg [5:0] A, B, C, D, temp;			//registers to store
  wire [5:0] bus;				//bus to transfer values
  wire triA, triB, triC, triD, triT;		//controls for tristates
  wire clkA, clkB, clkC, clkD, clkT;		//controls for clocks
  wire addsub,amuxc;				//controls for cpa/mux
  reg [5:0] IR;					//instruction register
  reg [3:0] PC;					//program counter
  reg [1:0] SC;					//step counter
  wire MOV, EXCG, STA, ADD, SUB, HLT, NOP, LDA; //processor commands
  wire [5:0] xorout, cpaout, muxout1;		//xor gate, mux, and cpa outputs
  wire carry;					//carry of cpa
  
  // ram setup
  ram my_ram(address, bus, read, write);

  // reset step counter, program counter, and address register
  always @ (posedge reset)
    begin
      PC <= 4'd0;
      SC <= 2'd0;
      address <= 4'd0;
    end
  
  // increment PC and set address of next instruction
  always @ (negedge clock)
    begin
      if(PC == 4'd15)
          PC <= 4'd0;
      else
        if((SC==2'd1 & ~EXCG)|(SC==2'd3))
          begin
            PC <= PC + 4'd1;
            address <= PC + 4'd1;
          end
    end  

  // reset step counter for each new command  
  always @ (negedge clock)
    begin
      if(reset|(SC==2'd1 & ~EXCG)|(SC==2'd3))
        SC <= 2'd0;
      else
        SC <= SC + 2'd1;
    end
    
  // NOP does nothing, HLT stops processor
  always @ (posedge clock)
    begin
      if (NOP);
      if (HLT) $stop;
    end
    
  // set address if STA/LDA
  always @ (negedge clock)
    begin
        if((SC == 2'd0) & (LDA | STA))
          begin
            address <= IR[3:0];
          end
    end
    
  // clock each register when they must take an input
  always @ (posedge clock)
    begin
      if(SC == 2'd0) IR <= bus;
      if(clkA) A <= muxout1;
      if(clkB) B <= bus;
      if(clkC) C <= bus;
      if(clkD) D <= bus;
      if(clkT) temp <= bus;
    end
    
  // cpa control for add/sub
  assign addsub = SUB;

  // mux control for regA input select
  assign amuxc = ~((SC == 2'd1)&(ADD | SUB));

  // tristate control clocks
  assign triA = (((SC == 2'd1) & (STA | ((source[0]) & (MOV | EXCG | ADD | SUB)))) | ((SC == 2'd2) & (destination[0])));
  assign triB = (((SC == 2'd1) & ((source[1]) & (MOV | EXCG | ADD | SUB))) | ((SC == 2'd2) & (destination[1])));
  assign triC = (((SC == 2'd1) & ((source[2]) & (MOV | EXCG | ADD | SUB))) | ((SC == 2'd2) & (destination[2])));
  assign triD = (((SC == 2'd1) & ((source[3]) & (MOV | EXCG | ADD | SUB))) | ((SC == 2'd2) & (destination[3])));
  assign triT = ((SC == 2'd3) & EXCG);

  // register control clocks (need-based clocking)
  assign clkA = (((SC == 2'd1) & (LDA | ADD | SUB | ((destination[0]) & MOV))) | ((SC == 2'd2) & (source[0])) | ((SC == 2'd3) & (destination[0])));
  assign clkB = (((SC == 2'd1) & ((destination[1]) & MOV)) | ((SC == 2'd2) & (source[1])) | ((SC == 2'd3) & (destination[1])));
  assign clkC = (((SC == 2'd1) & ((destination[2]) & MOV)) | ((SC == 2'd2) & (source[2])) | ((SC == 2'd3) & (destination[2])));
  assign clkD = (((SC == 2'd1) & ((destination[3]) & MOV)) | ((SC == 2'd2) & (source[3])) | ((SC == 2'd3) & (destination[3])));
  assign clkT = ((SC == 2'd1) & EXCG);

  // tell ram when to read/write to bus
  assign read = ~((SC == 2'd0)|LDA);
  assign write = ((SC == 2'd1) & STA & clock);

  // assign op codes to processor commands
  assign MOV = (instruction[1]);
  assign EXCG = (instruction[3]);
  assign LDA = (instruction[2]);
  assign STA = ((instruction[0])&(destination[3]));
  assign ADD = ((instruction[0])&(destination[1]));
  assign SUB = ((instruction[0])&(destination[2]));
  assign HLT = ((instruction[0])&(destination[0])&(source[0]));
  assign NOP = (~MOV & ~EXCG & ~LDA & ~STA & ~ADD & ~SUB & ~HLT);
  
  
  
  mux2 #(6) mux1(muxout1,{bus,cpaout},amuxc);   // mux module 
  xorgate #(6) gate1(xorout,addsub,bus);        // xorgate module to configure cpa for add or subtract
  cpa #(6) cpa1(cpaout,carry,xorout,A,addsub);  // cpa module to add or subtract registers
  tristate #(6) tristateA(bus,A,triA);			// tristate instances to load registers to bus
  tristate #(6) tristateB(bus,B,triB);
  tristate #(6) tristateC(bus,C,triC);
  tristate #(6) tristateD(bus,D,triD);
  tristate #(6) tristateTemp(bus,temp,triT);
  decode #(2) decoder1(instruction,IR[5:4]);
  decode #(2) decoder2(destination,IR[3:2]);
  decode #(2) decoder3(source,IR[1:0]);
  
endmodule

// memory model (16 6-bit words, read/write signals are active low

module ram(address, data, rd, wr);
  input [3:0] address;
  inout [5:0] data;  // bidirectional
  input rd, wr;

  reg [5:0] ram_mem[0:15];

  // output data when rd is low
  tristate #(6) ram_tristate(data, ram_mem[address], ~rd);

  // write data to memory at end of wr low
  always @(posedge wr)
     ram_mem[address] <= data;

  initial
    $readmemh("program.dat", ram_mem);
endmodule



// test bench, reads file input and initiates clock/reset

module test_veriproc;
  wire reset, clock;

  init test_init(reset, clock);
  veriproc my_cpu(reset, clock);
endmodule

