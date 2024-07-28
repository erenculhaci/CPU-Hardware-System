`timescale 1ns / 1ps

module register_n_bit_tb;
    //Inputs
    reg[7:0] data_input;
    reg[1:0] fun_sel;
    reg e, clk;
    
    wire [7:0]data_output;
    
    //  Initial values and clock generation
    initial begin
        data_input = 8'd7; e = 1; clk = 0;
        forever begin
            clk = ~clk; #25;
        end
    end
    
    // UUT initialization
    register_n_bit #(8) uut (
      .data_input(data_input),
      .fun_sel(fun_sel),
      .e(e),
      .clk(clk),
      .data_output(data_output)
    );
    
    initial begin
        fun_sel = 2'b01; #50;  // Load data
        fun_sel = 2'b11; #50;  // Increment data        
        fun_sel = 2'b10; #50;  //  Decrement data
        fun_sel = 2'b00; #50;  
        
          // Reset data
        e=0;
        fun_sel = 2'b01; #50;  // Load data
        fun_sel = 2'b11; #50;  // Increment data        
        fun_sel = 2'b10; #50;  //  Decrement data
        fun_sel = 2'b00; #50;  // Reset data
    end  
endmodule


//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////


module IR_tb;
    //Inputs
    reg[7:0] data_input;
    reg[1:0] fun_sel;
    reg e, clk, lh;
    
    wire [15:0]data_output;
        
    // UUT initialization
    IR uut (
      .data_input(data_input),
      .fun_sel(fun_sel),
      .e(e),
      .clk(clk),
      .lh(lh),
      .data_output(data_output)
    );
    
    // Clock generation
    always #25 clk = ~clk;
    
    //  Initial values
    initial begin
        data_input = 8'b11010010; e = 1; clk = 0; lh = 0;
    end
    
    initial begin
        fun_sel = 2'b00; lh = 0;    #50;  // Reset data
        fun_sel = 2'b01; lh = 1;    #50;  // Load data
        fun_sel = 2'b11; lh = 0;    #50;  // Increment data
        fun_sel = 2'b01; lh = 0;    #50;  // Load data
        fun_sel = 2'b10; lh = 1;    #50;  //  Decrement data
        $finish();
    end    
endmodule

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////


module RF_test;
    reg [7:0] in;
    reg [2:0] O1Sel;
    reg [2:0] O2Sel;
    reg [1:0] fun_sel;
    reg [3:0] RSel;
    reg [3:0] TSel;
    reg clk;
    wire [7:0] O1,O2;
    
    RF uut(
    .in(in), 
    .O1Sel(O1Sel), 
    .O2Sel(O2Sel), 
    .fun_sel(fun_sel), 
    .RSel(RSel), 
    .TSel(TSel), 
    .clk(clk),
    .O1(O1),
    .O2(O2)
  );
  
  //Clock generation
  always #25 clk = ~clk;
  
  //Initial values
  initial begin
    in = 8'b00101101; clk = 0; RSel = 4'b0000; TSel = 4'b0000;
  end
  
  //Stimulus
  initial begin
  fun_sel = 2'b00; O1Sel = 3'b001; O2Sel = 3'b001; RSel = 4'b1111; TSel = 4'b0111; #50;
  fun_sel = 2'b01; O1Sel = 3'b001; O2Sel = 3'b001; RSel = 4'b1101; TSel = 4'b1100; #50;
  fun_sel = 2'b10; O1Sel = 3'b001; O2Sel = 3'b001; RSel = 4'b1011; TSel = 4'b1110; #50;
  fun_sel = 2'b01; O1Sel = 3'b001; O2Sel = 3'b001; RSel = 4'b1011; TSel = 4'b1101; #50;
  fun_sel = 2'b11; O1Sel = 3'b001; O2Sel = 3'b000; RSel = 4'b1000; TSel = 4'b1111; #50;
  fun_sel = 2'b10; O1Sel = 3'b001; O2Sel = 3'b001; RSel = 4'b1110; TSel = 4'b0110; #50;
  fun_sel = 2'b01; O1Sel = 3'b001; O2Sel = 3'b001; RSel = 4'b0111; TSel = 4'b1100; #50;
  fun_sel = 2'b00; O1Sel = 3'b001; O2Sel = 3'b001; RSel = 4'b0001; TSel = 4'b0100; #50;
  $finish();
  end
endmodule

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

module ARF_test;
    reg [7:0] in;
    reg [1:0] OutASel;
    reg [1:0] OutBSel;
    reg [1:0] fun_sel;
    reg [3:0] RSel;
    reg clk;
    wire [7:0] OutA,OutB;
    
    ARF uut(
    .in(in), 
    .OutASel(OutASel), 
    .OutBSel(OutBSel), 
    .fun_sel(fun_sel), 
    .RSel(RSel), 
    .clk(clk),
    .OutA(OutA),
    .OutB(OutB)
  );
  
  //Clock generation
  always #25 clk = ~clk;
  
  //Initial values
  initial begin
    in = 8'b10100101; clk = 0; RSel = 4'b0000;
  end
  
  //Stimulus
  initial begin
  fun_sel = 2'b00; OutASel = 2'b01; OutBSel = 2'b10; RSel = 4'b1111; #50;
  fun_sel = 2'b01; OutASel = 2'b10; OutBSel = 2'b01; RSel = 4'b1111; #50;
  fun_sel = 2'b10; OutASel = 2'b00; OutBSel = 2'b11; RSel = 4'b1000; #50;
  fun_sel = 2'b01; OutASel = 2'b11; OutBSel = 2'b01; RSel = 4'b1011; #50;
  fun_sel = 2'b11; OutASel = 2'b01; OutBSel = 2'b00; RSel = 4'b1000; #50;
  fun_sel = 2'b10; OutASel = 2'b10; OutBSel = 2'b01; RSel = 4'b1110; #50;
  fun_sel = 2'b01; OutASel = 2'b11; OutBSel = 2'b10; RSel = 4'b0111; #50;
  fun_sel = 2'b00; OutASel = 2'b00; OutBSel = 2'b10; RSel = 4'b0001; #50;
  $finish();
  end
endmodule

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

module ALU_test();  
    reg[7:0]A,B;
    reg [3:0]fun_sel;
    wire [3:0]flag;  
    wire [7:0]ALUOut;
    
    ALU uut(.A(A),.B(B),.fun_sel(fun_sel),.flag(flag),.ALUOut(ALUOut));
    
    initial begin
    A=8'b10111010; B=8'b10111111;
    end
    
    initial begin
        fun_sel = 4'b0000; #20;
        fun_sel = 4'b0001; #20;
        fun_sel = 4'b0010; #20;
        fun_sel = 4'b0011; #20;
        fun_sel = 4'b0100; #20;
        fun_sel = 4'b0101; #20;
        fun_sel = 4'b0110; #20;
        fun_sel = 4'b0111; #20;
        fun_sel = 4'b1000; #20;
        fun_sel = 4'b1001; #20;
        fun_sel = 4'b1010; #20;
        fun_sel = 4'b1011; #20;
        fun_sel = 4'b1100; #20;
        fun_sel = 4'b1101; #20;
        fun_sel = 4'b1110; #20;
        fun_sel = 4'b1111; #20;
        $finish();
    end      
endmodule

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

module Project1Test();
    //Input Registers of ALUSystem
    reg[2:0] RF_O1Sel; 
    reg[2:0] RF_O2Sel; 
    reg[1:0] RF_FunSel;
    reg[3:0] RF_RSel;
    reg[3:0] RF_TSel;
    reg[3:0] ALU_FunSel;
    reg[1:0] ARF_OutASel; 
    reg[1:0] ARF_OutBSel; 
    reg[1:0] ARF_FunSel;
    reg[3:0] ARF_RSel;
    reg      IR_LH;
    reg      IR_Enable;
    reg[1:0]      IR_Funsel;
    reg      Mem_WR;
    reg      Mem_CS;
    reg[1:0] MuxASel;
    reg[1:0] MuxBSel;
    reg MuxCSel;
    reg      Clock;
    
    //Test Bench Connection of ALU System
    ALU_System _ALUSystem(
    .RF_OutASel(RF_O1Sel), 
    .RF_OutBSel(RF_O2Sel), 
    .RF_FunSel(RF_FunSel),
    .RF_RSel(RF_RSel),
    .RF_TSel(RF_TSel),
    .ALU_FunSel(ALU_FunSel),
    .ARF_OutCSel(ARF_OutASel), 
    .ARF_OutDSel(ARF_OutBSel), 
    .ARF_FunSel(ARF_FunSel),
    .ARF_RegSel(ARF_RSel),
    .IR_LH(IR_LH),
    .IR_Enable(IR_Enable),
    .IR_Funsel(IR_Funsel),
    .Mem_WR(Mem_WR),
    .Mem_CS(Mem_CS),
    .MuxASel(MuxASel),
    .MuxBSel(MuxBSel),
    .MuxCSel(MuxCSel),
    .Clock(Clock)
    );
    
    //Test Vector Variables
    reg [41:0] VectorNum, Errors, TotalLine; 
    reg [41:0] TestVectors[3:0];
    reg Reset, Operation;
    initial begin
        Reset = 0;
    end
    //Clock Signal Generation
    always 
    begin
        Clock = 1; #5; Clock = 0; #5; // 10ns period
    end
    
    //Read Test Bench Values
    initial begin
        $readmemb("TestBench.mem", TestVectors); // Read vectors
        VectorNum = 0; Errors = 0; TotalLine=0; Reset=0;// Initialize
    end
    
    // Apply test vectors on rising edge of clock
    always @(posedge Clock)
    begin
        {Operation, RF_O1Sel, RF_O2Sel, RF_FunSel, 
        RF_RSel, RF_TSel, ALU_FunSel, ARF_OutASel, ARF_OutBSel, 
        ARF_FunSel, ARF_RSel, IR_LH, IR_Enable, IR_Funsel, 
        Mem_WR, Mem_CS, MuxASel, MuxBSel, MuxCSel} = TestVectors[VectorNum];
    end
    
    // Check results on falling edge of clk
    always @(negedge Clock)
        if (~Reset) // skip during reset
        begin
            $display("Input Values:");
            $display("Operation: %d", Operation);
            $display("Register File: O1Sel: %d, O2Sel: %d, FunSel: %d, RSel: %d, TSel: %d", RF_O1Sel, RF_O2Sel, RF_FunSel, RF_RSel, RF_TSel);            
            $display("ALU FunSel: %d", ALU_FunSel);
            $display("Addres Register File: OutASel: %d, OutBSel: %d, FunSel: %d, Regsel: %d", ARF_OutASel, ARF_OutBSel, ARF_FunSel, ARF_RSel);            
            $display("Instruction Register: LH: %d, Enable: %d, FunSel: %d", IR_LH, IR_Enable, IR_Funsel);            
            $display("Memory: WR: %d, CS: %d", Mem_WR, Mem_CS);
            $display("MuxASel: %d, MuxBSel: %d, MuxCSel: %d", MuxASel, MuxBSel, MuxCSel);
            
            $display("");
            $display("Output Values:");
            $display("Register File: AOut: %d, BOut: %d", _ALUSystem.AOut, _ALUSystem.BOut);            
            $display("ALUOut: %d, ALUOutFlag: %d, ALUOutFlags: Z:%d, C:%d, N:%d, O:%d,", _ALUSystem.ALUOut, _ALUSystem.ALUOutFlag, _ALUSystem.ALUOutFlag[3],_ALUSystem.ALUOutFlag[2],_ALUSystem.ALUOutFlag[1],_ALUSystem.ALUOutFlag[0]);
            $display("Address Register File: AOut: %d, BOut (Address): %d", _ALUSystem.AOut, _ALUSystem.Address);            
            $display("Memory Out: %d", _ALUSystem.MemoryOut);            
            $display("Instruction Register: IROut: %d", _ALUSystem.IROut);            
            $display("MuxAOut: %d, MuxBOut: %d, MuxCOut: %d", _ALUSystem.MuxAOut, _ALUSystem.MuxBOut, _ALUSystem.MuxCOut);
            
            // increment array index and read next testvector
            VectorNum = VectorNum + 1;
            if (TestVectors[VectorNum] === 42'bx)
            begin
                $display("%d tests completed.",
                VectorNum);
                $finish; // End simulation
            end
        end
endmodule

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

module Project2Test();
    reg Clock; 
    wire Reset;
    wire [7:0] T;
    always 
    begin
        Clock = 1; #5; Clock = 0; #5; // 10ns period
    end
    CPUSystem _CPUSystem( 
            .Clock(Clock),
            .Reset(Reset),
            .T(T)    
        );
endmodule

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////

module CPUSystem_TB;

  reg Clock;
  wire Reset;
  wire [7:0] T;

  CPUSystem dut (
      .Clock(Clock),
      .Reset(Reset),
      .T(T)
  );

  always #5 Clock = ~Clock;

  initial begin
    Clock = 0;
    repeat (100) @(posedge Clock);

    $display("RF_OutASel: %b", dut.RF_OutASel);
    $display("RF_OutBSel: %b", dut.RF_OutBSel);
    $display("RF_FunSel: %b", dut.RF_FunSel);
    $display("RF_RSel: %b", dut.RF_RSel);
    $display("ALU_FunSel: %b", dut.ALU_FunSel);
    $display("ARF_OutCSel: %b", dut.ARF_OutCSel);
    $display("ARF_OutDSel: %b", dut.ARF_OutDSel);
    $display("ARF_FunSel: %b", dut.ARF_FunSel);
    $display("ARF_RegSel: %b", dut.ARF_RegSel);
    $display("IR_LH: %b", dut.IR_LH);
    $display("IR_Enable: %b", dut.IR_Enable);
    $display("IR_Funsel: %b", dut.IR_Funsel);
    $display("Mem_WR: %b", dut.Mem_WR);
    $display("Mem_CS: %b", dut.Mem_CS);
    $display("MuxASel: %b", dut.MuxASel);
    $display("MuxBSel: %b", dut.MuxBSel);
    $display("MuxCSel: %b", dut.MuxCSel);
    $display("SeqCtr_T: %b", dut.SeqCtr_T);
    $display("opcode: %b", dut.opcode);
    $display("z: %b", dut.z);
    $display("addressingMode: %b", dut.addressingMode);
    $display("RSel: %b", dut.RSel);
    $display("address: %b", dut.address);
    $display("dstReg: %b", dut.dstReg);
    $display("SReg1: %b", dut.SReg1);
    $display("SReg2: %b", dut.SReg2);
    $display("ARFValue: %b", dut.ARFValue);
    $display("RFValue: %b", dut.RFValue);
    $finish;
  end

endmodule


//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////


module SequentialCounter_test();

reg clock;
reg reset;

wire [2:0]T;

SequentialCounter uut(.clock(clock), .reset(reset), .T(T));

always #25 clock = ~clock;

initial begin
clock = 1;
reset = 1; #50;
reset = 0; #250;
reset = 1; #50;
reset = 0; #250;
$finish();
end
endmodule

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////