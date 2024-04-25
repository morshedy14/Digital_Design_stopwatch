# Digital_Design_stopwatch
# OurProject

Welcome to OurProject! This repository contains a collection of Verilog hardware modules designed for various digital logic operations, including multiplexers, adders, counters, and more. The modules are designed to be flexible and can be used in a variety of applications. 

## Project Structure

- **`OurProject.v`**: This is the main module that integrates several smaller modules to create a complex system.
- **Multiplexers**:
  - `mux21`: A 2-to-1 multiplexer module.
  - `mux41`: A 4-to-1 multiplexer module.
- **Counter**:
  - `counter`: A basic counter module.
- **Control Logic**:
  - `control_unit`: A control unit to manage signals for the rest of the system.
- **Adders and Subtractors**:
  - `add2`: A 2-bit adder module.
  - `sub2`: A 2-bit subtractor module.
- **Buffer and Demultiplexer**:
  - `tristate_buf`: A tristate buffer module.
  - `demux21`: A 2-to-1 demultiplexer module.
- **Comparators**:
  - `minVal_comp`: Compares a 14-bit input to a predefined minimum value.
  - `maxVal_comp`: Compares a 14-bit input to a predefined maximum value.

## Getting Started

To work with this project, you'll need a Verilog simulator or a suitable FPGA development environment. Here's how to get started:

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/OurProject.git
   ```

2. **Open the Project**:
   Open your Verilog development environment and import the project files from the cloned repository.

3. **Compile the Project**:
   Compile the Verilog files to check for any errors or warnings.

4. **Run Simulations**:
   Use testbenches or simulation tools to test individual modules or the complete system.

5. **Implement on FPGA**:
   If you have an FPGA development board, you can implement the design to see it in action.

## Contributing

Contributions are welcome! If you'd like to contribute to this project, please follow these steps:

1. Fork the repository.
2. Create a new branch for your changes.
3. Make your changes and commit them with descriptive messages.
4. Submit a pull request to the main repository.

## Contact

If you have any questions or need further assistance, feel free to contact the project maintainer at:
- [Mohamed Morshedy](morshedy149@gmail.com)
- [Aser Osama](s-aser.ibrahim@zewailcity.edu.eg)






---

Thank you for using OurProject! We hope you find it useful for your Verilog and FPGA development needs. If you have any feedback or suggestions, we'd love to hear from you.
