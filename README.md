## Overview

This project provides a standalone diagnostic robot program for FIRST Robotics Competition (FRC) robots, focused on hardware and wiring validation.

The diagnostic program runs on the roboRIO instead of the competition robot program and is used to test motors, sensors, encoders, CAN devices, and other hardware components in isolation. It supports early hardware bring-up, incremental validation, and hardware troubleshooting in a controlled and predictable environment.

The diagnostic system is intentionally limited to hardware-level behavior and does not include competition logic, autonomous routines, or driver controls.

## Motivation

Early hardware bring-up in FRC is frequently complicated by incomplete wiring, partially configured devices, and unfinished robot software architecture.

Even when robot code deploys successfully, hardware devices may not respond, may respond incorrectly, or may fail intermittently. Existing tools such as vendor dashboards and console logs expose raw values and error messages but do not provide a structured, repeatable mechanism for isolating individual devices and verifying correct hardware behavior.

This project addresses that gap by providing a dedicated diagnostic robot program that enables device-level testing, explicit status reporting, and safe operation when hardware is missing or incomplete.

## Non-Goals

This project does not attempt to:

* Debug autonomous routines, state machines, or competition logic
* Replace competition robot code or command-based architecture
* Tune closed-loop control, motion profiling, or drivetrain performance
* Provide live diagnostics during a match
* Act as a simulator or physics model

The diagnostic system is intended to validate hardware and wiring only.

## Architecture Overview

The diagnostic system is organized around device-level diagnostic objects.

Each hardware component under test is represented by a diagnostic device that encapsulates:

* Hardware initialization
* One or more diagnostic tests
* Explicit status reporting
* Safe failure behavior

Diagnostic devices are registered with a central diagnostic runner that:

* Initializes devices
* Executes tests on demand
* Publishes status and controls to the dashboard
* Ensures failures are contained and non-fatal

Missing, unplugged, or misconfigured hardware is reported as a diagnostic condition rather than causing the robot program to crash.

There are no subsystems, commands, schedulers, or operator interfaces. All robot behavior is intentional and test-driven.

### Data Flow (Conceptual)

```
[ Diagnostic Device ]
        |
        v
[ Diagnostic Runner ]
        |
        v
[ NetworkTables / Dashboard ]
```

* Devices expose tests and status
* The runner coordinates execution and lifecycle
* The dashboard provides control and visibility
* No competition code participates in this flow

## Usage Workflow

### Early Bring-Up

The diagnostic program is typically used before competition robot code exists.

Once the roboRIO is powered, the diagnostic program is deployed. Hardware can then be wired and validated incrementally:

* Motors can be spun and direction-checked
* Limit switches and sensors can be exercised manually
* Encoders can be verified for signal presence and polarity
* CAN devices can be validated by ID and responsiveness

Mechanical, electrical, and software work can proceed in parallel instead of serially.

### Incremental Validation

Partial configurations are fully supported.

Each diagnostic device reports status independently. A missing or miswired device does not block testing of other hardware.

This allows teams to validate and sign off hardware one component at a time.

### Competition Pits (Hardware Only)

At events, the diagnostic program can be temporarily deployed in the pits to diagnose hardware and wiring issues between matches.

Typical use cases include:

* Verifying wiring after a collision
* Confirming CAN devices after a brownout
* Validating replacement motors or sensors
* Checking encoders or limit switches that may have been disturbed

The diagnostic program does not debug autonomous or competition logic. Once hardware is confirmed or repaired, the competition robot program is redeployed unchanged.

## Getting Started

A minimal diagnostic setup requires only a few steps:

1. Create a new robot project dedicated to diagnostics
2. Add the diagnostic framework to the project
3. Instantiate diagnostic devices in `Robot.java`
4. Deploy the diagnostic program to the roboRIO
5. Use the dashboard to run tests and observe status

No subsystems, commands, or operator interfaces are required.

The recommended workflow is to maintain diagnostics as a separate robot project from competition code.

## Extending the System

The diagnostic system is intentionally extensible.

Support for new hardware is added by implementing a diagnostic device that conforms to the existing device interface and registering it with the diagnostic runner. Modifying the core framework is not required.

This supports:

* Team-specific custom hardware
* One-off sensors or offseason experiments
* Community-developed diagnostic modules for shared devices

Community modules can coexist with core devices without forcing adoption or destabilizing the framework.

## Safety and Liability Disclaimer

This software controls real robot hardware, including motors, actuators, and
mechanisms that can cause property damage or personal injury if used improperly.

This diagnostic program is intended for supervised testing and educational
use only. Users are responsible for ensuring that:

* The robot is physically secured during testing
* Adequate clearance is maintained around moving mechanisms
* Tests are conducted at appropriate power levels
* Standard FRC safety practices are followed at all times

This software is provided “as is”, without warranty of any kind. The authors
assume no responsibility for damage, injury, or loss resulting from the use
or misuse of this software.

This diagnostic system is not a safety system and does not replace mechanical
safeguards, electrical protection, limit switches, interlocks, or responsible
human supervision.
