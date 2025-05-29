# Tomasulo Architecture Implementation

This project implements a RISC-V processor using the Tomasulo algorithm for dynamic instruction scheduling and out-of-order execution.

## Project Structure

- `core/`: Contains the main processor components including functional units and reservation stations
- `common/`: Common modules and utilities
- `auxillary/`: Supporting modules and components
- `sim/`: Simulation files
- `test.s`: Assembly test code

## Features

- Out-of-order execution with in-order commit
- Reservation stations for different functional units
- Common Data Bus (CDB) for result broadcasting
- Support for ALU, Load/Store, Multiplication, and Division operations

## Implementation Details

The processor follows the Tomasulo algorithm with reservation stations for each functional unit. The division unit contains three reservation stations that can hold division operations, a functional unit controller that schedules operations, and a CDB interface for broadcasting results. Similar structures are implemented for other functional units like ALU and multiplication.
