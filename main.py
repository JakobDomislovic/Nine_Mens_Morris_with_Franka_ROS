from board_with_ai import *
import pygame

global_depth_variable = 0

if __name__ == "__main__":
    
    global global_depth_variable
    
    try:
        print('Game modes:\n  Player vs. AI     [1]\n  Player vs. Player [2]\n  AI vs. AI         [3]')
        mode = input('Choose game mode: ')
       
        if mode==2:
            game_depth = 0
            choose_heuristic = 0
            choose_heuristic2 = 0
        else:
            game_depth = input('Choose depth: ')
            print('Heuristic functions.\n  number_of_pieces      [1]\n  try_2_block_pieces    [2]\n  number_of_double_mill [3]\n  advanced_heuristic    [4]')
            if mode==1: 
                choose_heuristic = input('Choose heuristic function: ')
                choose_heuristic2 = 0
            else:
                choose_heuristic = input('Choose heuristic function for AI_1: ')
                choose_heuristic2 = input('Choose heuristic function for AI_2: ')
        
        global_depth_variable = game_depth
        
        start_game = Game_Board(mode, game_depth, choose_heuristic, choose_heuristic2)
        start_game.run()
    
    except pygame.error:
        import sys
        sys.exit(0)