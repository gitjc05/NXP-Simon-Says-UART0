# Simon Says Game Project

![NXP Freedom Board Example](Screenshot 2024-11-16 204118.png)

## Overview
This project implements a Simon Says game on the NXP Freedom board. The game displays a sequence of colored lights, and the player must enter the corresponding letters via UART0 terminal before time runs out. As the game progresses, the sequences get faster, increasing the difficulty.

## Technologies Used
- Assembly language for board-specific operations
- C language for game logic and control flow
- UART0 for serial communication with the terminal

## Setup
### Requirements
- NXP Freedom board (MKLL05Z)
- Development environment setup (program board with Keil)
- UART0 connection for input and output (Used Putty in this case)
