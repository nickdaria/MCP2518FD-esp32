# MCP2518FD Library for ESP32

This is a library for interfacing with the [MCP2518FD CAN FD Controller](https://www.microchip.com/en-us/product/mcp2518fd).

## Setup
This library was written to be used in esp-idf projects. It may work in PlatformIO, but this has not been tested.

To use in an existing esp-idf project, simply add the `components/MCP2518FD` directory to your project's `components` directory, run `idf.py reconfigure`, and optionally, `idf.py menuconfig`.

## Configuration
There are several configuration options that can be accessed via `idf.py menuconfig`. They can be found under `Component config` > `MCP2518FD Configuration`. Changing these results in a complete rebuild.

Configuration of the MCP2518FD itself can be found in `components/MCP2518FD/config/MCP2518Config.hpp`. Here you can set the oscillator frequency, FIFO sizes, etc. During compilation, these constant expressions are used to generate the MCP2518's memory map, and so is able to determine whether the given config is valid.

## Example
The repository itself is an example project that can be built using `idf.py build`. The example application simply initializes the SPI bus, initializes the MCP2518FD, then enters a infinite loop listening for messages and echoing them.

