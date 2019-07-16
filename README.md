# helios

So far only tested things on [STM32L0 Discovery kit](https://www.st.com/en/evaluation-tools/b-l072z-lrwan1.html), which features the [STM32L072CZ](https://www.st.com/en/microcontrollers-microprocessors/stm32l072cz.html).

First off, you can use the built-in debugger better if you [turn it into a J-Link](https://www.segger.com/products/debug-probes/j-link/models/other-j-links/st-link-on-board/). The only downside is you lose the virtual UART feature from ST-Link.

Run JLink server:
`JLinkGDBServer -device STM32L072CZ -speed 4000 -if jtag -AutoConnect -1 -port 3333`

If for some reason, you really want to use openocd/ST-Link

`sudo openocd -f ./openocd.cfg`

At the moment, you need to restart the OpenOCD server every time you upload. The reason seems to be that if you try to flash without explicitly erasing, the flash fails. Starting the server clears the flash. JLink doesn't have this problem.

Use a .cargo/config as such:

```
[target.thumbv6m-none-eabi]
# uncomment this to make `cargo run` execute programs on QEMU
# runner = "qemu-system-arm -cpu cortex-m3 -machine lm3s6965evb -nographic -semihosting-config enable=on,target=native -kernel"

[target.'cfg(all(target_arch = "arm", target_os = "none"))']
# uncomment ONE of these three option to make `cargo run` start a GDB session
# which option to pick depends on your system
# runner = "arm-none-eabi-gdb -q -x openocd.gdb"
# runner = "gdb-multiarch -q -x openocd.gdb"c
# runner = "gdb -q -x openocd.gdb"
runner = "gdb-multiarch -tui -q -x openocd.gdb"

rustflags = [
  # LLD (shipped with the Rust toolchain) is used as the default linker
  "-C", "link-arg=-Tlink.x",

  # if you run into problems with LLD switch to the GNU linker by commenting out
  # this line
  #"-C", "linker=arm-none-eabi-ld",

  # if you need to link to pre-compiled C libraries provided by a C toolchain
  # use GCC as the linker by commenting out both lines above and then
  # uncommenting the three lines below
  # "-C", "linker=arm-none-eabi-gcc",
  # "-C", "link-arg=-Wl,-Tlink.x",
  # "-C", "link-arg=-nostartfiles",
]

[build]
# Pick ONE of these compilation targets
target = "thumbv6m-none-eabi"    # Cortex-M0 and Cortex-M0+
# target = "thumbv7m-none-eabi"    # Cortex-M3
# target = "thumbv7em-none-eabi"   # Cortex-M4 and Cortex-M7 (no FPU)
# target = "thumbv7em-none-eabihf" # Cortex-M4F and Cortex-M7F (with FPU)
```

Creating a DFU:

You'll need to turn the cargo ouput into intel-hex for the DFU utility:
`arm-none-eabi-objcopy -O ihex target/thumbv6m-none-eabi/release/helios helios.hex`

