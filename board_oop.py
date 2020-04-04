from state_space import *
import numpy as np
import pygame
import pygame.gfxdraw
PLAYER = 'WHITE'

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

# dict for closed positions 
closed_positions = {}
# za lakse nalazenje moguceg mlina
white_positions  = {}
black_positions  = {}
WHITE_PIECES = 9
BLACK_PIECES = 9

all_pieces_list = pygame.sprite.Group() # pozivas kad oces sve uploadati na ekran
white_pieces_list = pygame.sprite.Group()
black_pieces_list = pygame.sprite.Group()

# # funkcija koja gleda koje smo polje htjeli oznaciti i je li to polje slobodno
# def check_position(x, y):
#     for k in position_on_board:
#         d = (position_on_board[k][0] - x)**2 + (position_on_board[k][y] - y)**2
#         if d <= PIECE_SIZE**2 and not in closed_positions:
#             closed_positions[k] = position_on_board[k]
#             return position_on_board[k], True
#     return [0,0], False



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
        
        self.update_gui() # pozivam update gui fju koja je u stavri petlja do kad ne iskljucimo gui

    def update_gui(self):
        global mouse_click
        self.running = True
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    self.mx, self.my = pygame.mouse.get_pos()
                    print self.mx, self.my
                    mouse_click = True

            all_pieces_list.draw(self.screen)

            if PLAYER == 'WHITE' and mouse_click:
                PLAYER == 'BLACK'
                mouse_click = False
                self.pos_for_piece, self.empty_spot = self.check_position(self.mx, self.my)
                print(self.empty_spot)
                if self.empty_spot:
                    white = Piece(self.pos_for_piece, WHITE, BLACK)
                    all_pieces_list.add(white)
            #else:
            # inace ide racunalo
               #pygame.display.flip()
        pygame.display.flip()
        pygame.quit()
    
    def draw_lines_for_board(self, start, end):
        pygame.draw.line(self.screen, LINES, start, end, THICK)
        pygame.draw.line(self.screen, LINES, start[::-1], end[::-1], THICK)
        pygame.draw.circle(self.screen, LINES, start, 10)
        pygame.draw.circle(self.screen, LINES, end, 10)
        pygame.draw.circle(self.screen, LINES, [(start[0]+end[0])/2, (start[1]+end[1])/2], 10)


    # funkcija koja gleda koje smo polje htjeli oznaciti i je li to polje slobodno
    def check_position(self, x, y):
        for self.k in position_on_board:
            print position_on_board[self.k], x, y
            self.d = (position_on_board[self.k][0] - x)**2 + (position_on_board[self.k][0] - y)**2
            print int(np.sqrt(self.d)), PIECE_SIZE
            print(closed_positions)
            if int(np.sqrt(self.d)) <= PIECE_SIZE and self.k not in closed_positions:
                closed_positions[self.k] = position_on_board[self.k]
                return position_on_board[self.k], True
        return [0,0], False


class Piece(pygame.sprite.Sprite):

    def __init__(self, position, color1, color2):
        pygame.sprite.Sprite.__init__(self)

        self.image = pygame.Surface((PIECE_SIZE, PIECE_SIZE), pygame.SRCALPHA)
        pygame.gfxdraw.aacircle(self.image, position[0], position[1], PIECE_SIZE, color1)
        self.rect = self.image.get_rect()

    def update(self):
        # ovdje stavi naredbu kill u slucaju da trebas ubiti igraca
        pass

    def draw_piece(self, position, color1, color2):
            pygame.circle(screen, color1, position, PIECE_SIZE, 0)
            pygame.draw.circle(screen, color2, position, PIECE_SIZE, 1)
            pygame.draw.circle(screen, color2, position, PIECE_SIZE/2, 1)
