from state_space import *
import numpy as np
import pygame


# GUI settings
WIDTH  = 900
HEIGHT = 900
BACKGROUND = (255, 249, 174) # background color
WHITE = (250, 250, 250)      # white player
BLACK = (20, 20, 20)         # black player
LINES = (245, 182, 84)       # line color 
THICK = 5                    # line thickness
PIECE_SIZE = 25              # piece radius

# Global variables
mouse_click = False
mouse_click_X = 0
mouse_click_Y = 0




class Game_Board():
    
    def __init__(self):
        pygame.init()

        # Set up the drawing window
        self.screen = pygame.display.set_mode([WIDTH, HEIGHT])
        pygame.display.set_caption("Nine Men's Morris")
        
        # Fill the background with white
        self.screen.fill(BACKGROUND)
        # upper horizontal, left vertical
        self.draw_lines_for_board([100, 100], [800, 100])
        self.draw_lines_for_board([225, 225], [675, 225])
        self.draw_lines_for_board([350, 350], [550, 350])
        # lower horizontal, right vertical
        self.draw_lines_for_board([100, 800], [800, 800])
        self.draw_lines_for_board([225, 675], [675, 675])
        self.draw_lines_for_board([350, 550], [550, 550])
        # middle horizontal and vertical
        self.draw_lines_for_board([800, 450], [550, 450])
        self.draw_lines_for_board([100, 450], [350, 450])
        # update changes on gui
        pygame.display.flip()
        
        self.update_gui()

    def update_gui(self):
        self.running = True
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    self.mx, self.my = pygame.mouse.get_pos()
                    print self.mx, self.my
                    mouse_click = True
        
            #if mouse_click:
            #    mouse_click = False
            #    draw_piece([mx, my], 0)
            #    pygame.display.flip()

        pygame.quit()
    
    def draw_lines_for_board(self, start, end):
        pygame.draw.line(self.screen, LINES, start, end, THICK)
        pygame.draw.line(self.screen, LINES, start[::-1], end[::-1], THICK)
        pygame.draw.circle(self.screen, LINES, start, 10)
        pygame.draw.circle(self.screen, LINES, end, 10)
        pygame.draw.circle(self.screen, LINES, [(start[0]+end[0])/2, (start[1]+end[1])/2], 10)


class Piece(pygame.sprite.Sprite):

    def __init__(self, position, color1, color2):
        pygame.sprite.Sprite.__init__(self)

        self.image = self.draw_piece(position, color1, color2)

    
    def draw_piece(self, position, color1, color2):
            pygame.draw.circle(screen, color1, position, PIECE_SIZE, 0)
            pygame.draw.circle(screen, color2, position, PIECE_SIZE, 1)
            pygame.draw.circle(screen, color2, position, PIECE_SIZE/2, 1)
