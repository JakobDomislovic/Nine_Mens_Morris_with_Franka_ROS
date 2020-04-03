from board_oop import *
import pygame

if __name__ == "__main__":
    try:
        start_game = Game_Board()
        start_game.run()
    except pygame.error:
        import sys
        sys.exit(0)