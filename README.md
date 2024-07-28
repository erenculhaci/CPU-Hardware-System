# CPU System in Verilog

This project implements a CPU hardware system using Verilog. The system is composed of several modules that work together to perform various computational tasks. Each module is designed to handle specific functions, contributing to the overall operation of the CPU.

## Project Structure

The CPU System consists of the following modules:

1. **register_n_bit**: 
   - This module represents a general-purpose register of n bits. It stores data and supports basic operations like loading and resetting values.

2. **IR (Instruction Register)**:
   - The Instruction Register holds the current instruction being executed by the CPU. It plays a crucial role in decoding and executing instructions.

3. **Memory**:
   - The Memory module simulates the main memory of the CPU. It stores data and instructions, allowing read and write operations.

4. **RF (Register File)**:
   - The Register File is a collection of registers used for storing temporary data. It facilitates quick access and manipulation of data by the CPU.

5. **ARF (Address Register File)**:
   - The Address Register File stores addresses for memory access operations. It helps manage memory addressing efficiently.

6. **ALU (Arithmetic Logic Unit)**:
   - The ALU performs arithmetic and logical operations on data. It is a fundamental component of the CPU, handling operations like addition, subtraction, bitwise operations, and more.

7. **ALU_System**:
   - This module integrates the ALU with other components, managing the control signals and data flow for executing ALU operations.

8. **CPU_System**:
   - The CPU_System module is the central control unit, orchestrating the operation of all other modules. It manages instruction fetch, decode, execute, and write-back cycles.

9. **Sequential Counter**:
   - The Sequential Counter is used for timing and control purposes. It helps synchronize operations within the CPU.

## Simulations

Simulations were conducted for all the modules to verify their functionality and performance. The simulations include:

- **Module-Level Testing**: Each module was tested individually to ensure it operates as expected. This includes verifying data storage, retrieval, arithmetic operations, and control logic.
- **Integration Testing**: Modules were integrated and tested together to ensure seamless communication and correct data flow. The CPU_System was tested to verify the proper execution of instructions.
- **Performance Testing**: The system's performance was evaluated under various scenarios to assess its efficiency and correctness.

## How to Use

1. **Clone the Repository**: Clone the project repository to your local machine.
2. **Simulation**: Use a Verilog simulator to run the provided testbenches and verify the system's functionality.
3. **Modify and Extend**: Feel free to modify the modules or add new features to enhance the system's capabilities.
