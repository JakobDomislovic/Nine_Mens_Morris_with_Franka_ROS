from board_with_ai import *
import pygame

if __name__ == "__main__":
    try:
        mode = input('[1] for player vs. AI, [2] for player vs. player: ')
        start_game = Game_Board(mode)
        start_game.run()
    except pygame.error:
        import sys
        sys.exit(0)