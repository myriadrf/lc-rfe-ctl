# rfe-mini-test

This folder contains test firmware for the Raspberry Pi Pico MCU board, running the LibreCellular RFE-Mini board.

Full GUI control not yet implemented, purpose of this test firmware is to validate the board design.

## Setup
  - Put the Pico in UF2 mode, and drag and drop the [CircuitPython blob](https://circuitpython.org/board/raspberry_pi_pico/) into the drive.
  - With CircuitPython flashed, copy contents of this folder into the drive
  - Edit line ~165 in code.py with the test number you'd like to run. Saving the file should reboot the Pico and your test will run
  - Use [Mu](https://codewith.mu/) as it is the simplest editor, click "Serial" to get REPL