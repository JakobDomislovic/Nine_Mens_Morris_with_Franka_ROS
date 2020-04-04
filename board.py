# Import and initialize the pygame library
from state_space import *
from threading import Event
import numpy as np
import pygame
pygame.init()

# Set up the drawing window
screen = pygame.display.set_mode([900, 900])
pygame.display.set_caption("Nine Men's Morris")

# GUI settings
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


# funkcija koja gleda koje smo polje htjeli oznaciti i je li to polje slobodno
def check_position(x, y):
    for k in position_on_board:
        d = (position_on_board[k][0] - x)**2 + (position_on_board[k][1] - y)**2
        if np.sqrt(d) <= PIECE_SIZE and not in closed_positions:
            closed_positions[k] = position_on_board[k]
            return position_on_board[k], True
    return [0,0], False


# Only for drawing lines on board
def draw_lines_for_board(start, end):
    pygame.draw.line(screen, LINES, start, end, THICK)
    pygame.draw.line(screen, LINES, start[::-1], end[::-1], THICK)
    pygame.draw.circle(screen, LINES, start, 10)
    pygame.draw.circle(screen, LINES, end, 10)
    pygame.draw.circle(screen, LINES, [(start[0]+end[0])/2, (start[1]+end[1])/2], 10)

# 
def draw_piece(position, depth):
    if depth == 0:
        pygame.draw.circle(screen, WHITE, position, PIECE_SIZE, 0)
        pygame.draw.circle(screen, BLACK, position, PIECE_SIZE, 1)
        pygame.draw.circle(screen, BLACK, position, PIECE_SIZE/2, 1)
    else:
        pygame.draw.circle(screen, BLACK, position, PIECE_SIZE, 0)
        pygame.draw.circle(screen, WHITE, position, PIECE_SIZE, 1)
        pygame.draw.circle(screen, WHITE, position, PIECE_SIZE/2, 1)


# Run until the user asks to quit
running = True
board_is_made = False
while running:

    # Did the user click the window close button?
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            mx, my = pygame.mouse.get_pos()
            print mx, my
            mouse_click = True
    if not board_is_made:
        # Fill the background with white
        screen.fill(BACKGROUND)
        # upper horizontal, left vertical
        draw_lines_for_board([100, 100], [800, 100])
        draw_lines_for_board([225, 225], [675, 225])
        draw_lines_for_board([350, 350], [550, 350])
        # lower horizontal, right vertical
        draw_lines_for_board([100, 800], [800, 800])
        draw_lines_for_board([225, 675], [675, 675])
        draw_lines_for_board([350, 550], [550, 550])
        # middle horizontal and vertical
        draw_lines_for_board([800, 450], [550, 450])
        draw_lines_for_board([100, 450], [350, 450])

        pygame.display.flip()
        board_is_made = True
    
    if mouse_click:
        mouse_click = False
        draw_piece([mx, my], 0)
        pygame.display.flip()

    # Flip the display
    
# Done! Time to quit.
pygame.quit()


