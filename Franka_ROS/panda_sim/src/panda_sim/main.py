#!/usr/bin/env python

from board_with_ai import * # ovo ti tu ni ne treba...
import pygame
from alpha_beta_pruning import GLOBAL_alfa_cnt, GLOBAL_beta_cnt, GLOBAL_node_max_cnt, GLOBAL_node_min_cnt


global_depth_variable = 0

if __name__ == "__main__":
    
    global global_depth_variable
    global_depth_variable = 0
    global GLOBAL_alfa_cnt, GLOBAL_beta_cnt, GLOBAL_node_max_cnt, GLOBAL_node_min_cnt
    GLOBAL_alfa_cnt, GLOBAL_beta_cnt, GLOBAL_node_max_cnt, GLOBAL_node_min_cnt = 0,0,0,0
                    
    try:
        game_depth = 4
        print('Start')
        mode = 1
        #print('Heuristic functions.\n  number_of_pieces      [1]\n  try_2_block_pieces    [2]\n  number_of_double_mill [3]\n  advanced_heuristic    [4]')
        choose_heuristic = 4 #input('Choose heuristic function: ')
        choose_heuristic2 = 0
        
        global_depth_variable = game_depth
        
        start_game = Game_Board(mode, game_depth, choose_heuristic, choose_heuristic2)
        start_game.run()

    except pygame.error:
        import sys
        sys.exit(0)