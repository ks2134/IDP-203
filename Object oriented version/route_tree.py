tree = [["TL", "R","B"], #0 start route to 16
        ["RL", "TL"], #1 16 to Y/R
        ["RR","S","TR"], #2 16 to G/B
        ["SL","S","R", "S", "R","B"], #3 Y/R to 19
        ["SR", "S", "L", "L","B"], #4 G/B to 19
        ["RR","S","TL","S"], #5 19 to Y/R
        ["RL","TR","S"], #6 19 to G/B
        ["SL","S","R","L","L","B"], #7 Y/R to 17
        ["SR","S","L","S","R","L","B"], #8 G/B to 17
        ["RL","TR","TL","S"], #9 17 to Y/R
        ["RL","TL","S","TR","S"], #10 17 to G/B
        ["SL","S","S","CR","S","R","B"],#11 Y/R to 18
        ["SR","S","S","CL","L","B"],#12 G/B to 18
        ["RR","S","CL","S","S"],#13 18 to Y/R
        ["RL","CR","S","S"],#14 18 to G/B
        ["SL","R","S","R","FIN"],#15 Y/R to finish
        ["SR","L","L","FIN"]] #16 G/B to finish
#box_num initially 0
#inc 1 or 2 ......... 4*box_num + inc when collecting 4*box_num + 2*inc when delivering
#4*box_num + 1 to go to Y/R
#4*box_num + 2 to go to B/G
#4*box_num + 3 to go from Y/R
#4*box_num + 4 to go from B/G