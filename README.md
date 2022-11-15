In this implementation we try to interface a character LCD 16x2 in stm32 microcontroller using Arm Keil MDK in two following ways:
1. Displaying the number of seconds that have passed since the start of the program on the character LCD. (Image is included)
2. Implementing tic-tac-toe game, using two external keys for moving around 9 squares.
By pressing the blue button on the board, the square value changes between empty, X and O, and after one second passed since the last change, the selected value is recorded.
At the end of the game, "X/O won"  or "tied" will be displayed. (using Custom character generator)