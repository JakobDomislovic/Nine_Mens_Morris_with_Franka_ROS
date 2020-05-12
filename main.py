from board_with_ai import *
import pygame

global_depth_variable = 0

if __name__ == "__main__":
    global global_depth_variable
    try:
        mode = input('[1] for player vs. AI, [2] for player vs. player: ')
        game_depth = input('Choose depth: ')
        print('Heuristic functions.\n  number_of_pieces      [1]\n  try_2_block_pieces    [2]\n  number_of_double_mill [3]')
        choose_heuristic = input('Choose heuristic function: ')
        global_depth_variable = game_depth
        start_game = Game_Board(mode, game_depth, choose_heuristic)
        start_game.run()
    except pygame.error:
        import sys
        sys.exit(0)