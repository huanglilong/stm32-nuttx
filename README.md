# stm32-nuttx
  [Nuttx notes](https://huanglilong.gitbooks.io/nuttx-notes/content/)

# Environment
  1. arm-none-eabi-gcc version 4.9.3 and 5.4.1 has been tested.

  2. Ubuntu14.04 / Ubuntu16.04

  3. [flash tools](https://github.com/texane/stlink)

# Building and exporting nuttx
$ git clone https://github.com/huanglilong/stm32-nuttx

$ cd stm32-nuttx

$ git submodule update --init

$ make archive

# Building firmware
$ make firmware

# Flashing firmware
$ make upload
