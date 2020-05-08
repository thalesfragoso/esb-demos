# This is an example GDB script for flashing the demos using OpenOCD. To use it,
# copy it into the demo directory and rename it `.gdbinit`. Then, running
# `cargo run` should automatically make use of it.
# Note that you might need to build a recent OpenOCD version from source.

# disable "are you sure you want to quit?"
define hook-quit
    set confirm off
end

target remote :3333

# detect unhandled exceptions, hard faults and panics
break DefaultHandler
break HardFault
break rust_begin_unwind

# *try* to stop at the user entry point (it might be gone due to inlining)
break main

# print demangled symbols by default
set print asm-demangle on

monitor arm semihosting enable
load
cont
