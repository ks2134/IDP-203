route_tree = [["TL", "R"], #0 start route to 16
              ["RL", "TL"], #1 16 to Y/R
              ["RR","S","TR"], #2 16 to G/B
              ["SL","S","R", "S", "R"], #3 Y/R to 19
              ["SR", "S", "L", "L"], #4 G/B to 19
              ["RR","S","TL","S"], #5 19 to Y/R
              ["RL","TR","S"], #6 19 to G/B
              ["SL","S","R","L","L"], #7 Y/R to 17
              ["SR","S","L","S","R","L"], #8 G/B to 17
              ["RL","TR","TL","S"], #9 17 to Y/R
              ["RL","TL","S","TR","S"], #10 17 to G/B
              ["SL","S","S","CR","S","R"],#11 Y/R to 18
              ["SR","S","S","CL","L"],#12 G/B to 18
              ["RR","S","CL","S","S"],#13 18 to Y/R
              ["RL","CR","S","S"],#14 18 to G/B
              ["SL","R","S","R","FIN"],#15 Y/R to finish
              ["SR","L","L","FIN"]] #16 G/B to finish


