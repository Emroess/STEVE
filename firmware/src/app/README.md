# Application Interfaces Subsystem

This directory contains high-level application interfaces and utilities.

## Files

- **`cli.c`** - Command-line interface
  - Local command processing
  - Parameter validation
  - Help system and auto-completion

- **`perfmon.c`** - Performance monitoring
  - System resource tracking
  - Performance metrics collection
  - Diagnostic data export

## Architecture

Application interfaces provide:
- User interaction layers
- System monitoring and diagnostics
- Configuration management
- Data export capabilities

## Dependencies

- Hardware drivers (UART for CLI)
- System services
- Valve subsystem for status data