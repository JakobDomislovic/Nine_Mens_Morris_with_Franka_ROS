from state_space_descriptor import *
import numpy as np
import pygame
import random

# kasnije budes random biral ko prvi pocinje, uvijek prvi pocinje s bijelom bojom
#if random.randrange(100) % 2:
#    ON_MOVE = 'CPU'
#else:
#    ON_MOVE = 'HUMAN'

#PLAYER_ON_MOVE = 'WHITE' # globalna varijabla ko je sljedeci na redu
                         # nazovi ju ON_MOVE =
                         # i mijenjaj u svakom koraku


# GUI settings
WIDTH  = 900
HEIGHT = 900
BACKGROUND = (255, 249, 174) # background color
FPS = 60                     # frames per sec
WHITE = (240, 240, 240)      # white player
BLACK = (20, 20, 20)         # black player
LINES = (245, 182, 84)       # line color 
THICK = 5                    # line thickness
PIECE_SIZE = 25              # piece radius

# Global variables
mouse_click = False

# dict for closed positions
closed_positions = {}

# za lakse nalazenje moguceg mlina
white_positions_taken  = {}
black_positions_taken  = {}
white_mill_dict = {}
white_mill_dict_helper = {}
black_mill_dict = {}
black_mill_dict_helper = {}

# globalno upravljaj brojem figura pojedinih boja
WHITE_PIECES = 9
BLACK_PIECES = 9

# grupe spriteova
all_pieces_list = pygame.sprite.Group() # pozivas kad oces sve uploadati na ekran
white_pieces_list = pygame.sprite.Group()
black_pieces_list = pygame.sprite.Group()


class Game_Board():
    
    def __init__(self):
        pygame.init()
        # Set up the drawing window
        self.screen = pygame.display.set_mode([WIDTH, HEIGHT])
        pygame.display.set_caption("Nine Men's Morris")
        pygame.display.flip()
        self.PLAYER_ON_MOVE = WHITE
        self.NUMBER_OF_PIECES = 18

        self.clock = pygame.time.Clock()

        self.update_gui() # pozivam update gui fju koja je u stavri petlja do kad ne iskljucimo gui

    def update_gui(self):
        global mouse_click
        self.running = True
        print(self.PLAYER_ON_MOVE)
        while self.running:
            # clock for updating gui
            self.clock.tick(FPS)
            
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
            
            # check for events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    self.mx, self.my = pygame.mouse.get_pos()
                    mouse_click = True   # mozes ju napraviti lokalnom varijablom

            if self.PLAYER_ON_MOVE == WHITE and mouse_click:
                mouse_click = False
                is_legal_move_made = self.make_move(self.PLAYER_ON_MOVE, [self.mx, self.my])
                if is_legal_move_made:
                    print('LEGALNO')
                    if self.is_mill(WHITE):
                        print('MLIN MLIN NMLI MLIN  MLIN')
                    self.PLAYER_ON_MOVE = BLACK
            elif self.PLAYER_ON_MOVE == BLACK:
                choose_place = input('Upisi poziciju za crnog: ')
                if choose_place in position_on_board:
                    is_legal_move_made = self.make_move(self.PLAYER_ON_MOVE, position_on_board[choose_place])
                    if is_legal_move_made:
                        # provjeri je li doslo do mlina
                        self.PLAYER_ON_MOVE = WHITE

            all_pieces_list.update()
            all_pieces_list.draw(self.screen)

            pygame.display.flip()
        pygame.quit()
    
    def draw_lines_for_board(self, start, end):
        pygame.draw.line(self.screen, LINES, start, end, THICK)
        pygame.draw.line(self.screen, LINES, start[::-1], end[::-1], THICK)
        pygame.draw.circle(self.screen, LINES, start, 10)
        pygame.draw.circle(self.screen, LINES, end, 10)
        pygame.draw.circle(self.screen, LINES, [(start[0]+end[0])/2, (start[1]+end[1])/2], 10)


    # funkcija koja gleda koje smo polje htjeli oznaciti i je li to polje slobodno
    # dodaj da u closed_positions imas i boju igraca
    # taj closed budes slal u minimax
    def check_position(self, x, y):
        '''
        prolazi kroz cijeli state space i trazi tocku najblizu onoj koju si kliknuo na ekranu
        ovo se radi i kada damo tocnu tocku, sto nije najbolje, ali za sada necu raditi promjene
        gubitak vremena je minimalni prostor stanja nije veliki
        '''
        for self.k in position_on_board:
            self.d = (position_on_board[self.k][0] - x)**2 + (position_on_board[self.k][1] - y)**2
            if ( int(np.sqrt(self.d)) <= PIECE_SIZE ) and ( self.k not in closed_positions ):
                closed_positions[self.k] = position_on_board[self.k]
                print(closed_positions)
                return self.k, True
        return [0,0], False

    def make_move(self, player_color, position): # u class Piece isto imas position, NEMAJU VEZE JEDAN S DRUGIM
        '''
        radis poteze za svaku fazu
        '''
        meta = True # varijabla za 
        if self.NUMBER_OF_PIECES:  
            '''
            ako nemas zatvoreno barem 18 polja znaci da nisu svi igraci na ploci,
            a to znaci da si u PRVOJ FAZI IGRE --> stavljaj figure gdje mozes.
            Trebas provjeriti igra li WHITE(u ovom slucaju je to HUMAN) ili BLACK (CPU)
            '''
            if player_color == WHITE:
                self.key, self.empty_spot = self.check_position(position[0], position[1])
                if self.empty_spot:
                    white = White_Piece(position_on_board[self.key])
                    white_positions_taken[self.key] = position_on_board[self.key]
                    all_pieces_list.add(white)
                    white_pieces_list.add(white)
                    self.PLAYER_ON_MOVE = BLACK
                    self.NUMBER_OF_PIECES -= 1
                    return True # ako si napravio legalan potez vrati 'True'
                else:
                    return False

            else:
                self.key, self.empty_spot = self.check_position(position[0], position[1])
                if self.empty_spot:
                    black = Black_Piece(position_on_board[self.key])
                    black_positions_taken[self.key] = position_on_board[self.key]
                    all_pieces_list.add(black)
                    black_pieces_list.add(black)
                    self.PLAYER_ON_MOVE = WHITE
                    self.NUMBER_OF_PIECES -= 1
                    return True # ako si napravio legalan potez vrati 'True'
                else:
                    return False

        elif player_color == WHITE and WHITE_PIECES > 3 and not NUMBER_OF_PIECES:
            '''
            druga faza igrem samo za bijelog igraca
            '''
            print('slozi mlin majmune')
        
        elif player_color == BLACK and BLACK_PIECES > 3 and not NUMBER_OF_PIECES:
            '''
            druga faza samo za crnog igraca
            '''
            pass
        else:
            '''
            treca faza moze biti samo ako nije ni jedna druga
            '''
            pass


    def is_mill(self, player_color):
        print('usao')
        counter = {}
        cnt_d_low   = 0
        cnt_d_high  = 0
        cnt_4_left  = 0
        cnt_4_right = 0
        new_mill = False
        counter['4_left'], counter['4_right'] = 0, 0

        if player_color == WHITE:
            for k in white_positions_taken:
                if k[0] == 'd':
                    if k == 'd1' or k == 'd2' or k == 'd3':
                        if 'd_low' not in counter: counter['d_low'] = 1
                        else:
                            counter['d_low'] += 1
                            if counter['d_low'] == 3 and 'd_low' not in white_mill_dict_helper:
                                white_mill_dict_helper['d_low'] = True
                                new_mill = True
                    
                    if k == 'd5' or k == 'd6' or k == 'd7':
                        if 'd_high' not in counter: counter['d_high'] = 1
                        else:
                            counter['d_high'] += 1
                            if counter['d_high'] == 3 and 'd_high' not in white_mill_dict_helper:
                                white_mill_dict_helper['d_high'] = True
                                new_mill = True
                
                elif k[1] == 4:
                    if k == 'a4' or k == 'b4' or k == 'c4':
                        if '4_left' not in counter: counter['4_left'] = 1
                        else:
                            counter['4_left'] += 1
                            if counter['4_left'] == 3 and '4_left' not in white_mill_dict_helper:
                                white_mill_dict_helper['4_left'] = True
                                new_mill = True
                    
                    if k == 'e4' or k == 'f4' or k == 'g4':
                        if 1 == 1: print(1)
                        else:
                            counter['4_right'] += 1
                            if counter['4_right'] == 3 and '4_right' not in white_mill_dict_helper:
                                white_mill_dict_helper['4_right'] = True
                                new_mill = True
                else:
                    if k[0] not in counter:
                        counter[k[0]] = 1
                    else:
                        counter[k[0]] += 1
                        if counter[k[0]] == 3 and k[0] not in white_mill_dict_helper: 
                            white_mill_dict_helper[k[0]] = True
                            new_mill = True
                    if k[1] not in counter:
                        counter[k[1]] = 1
                    else:
                        counter[k[1]] += 1
                        if counter[k[1]] == 3 and k[1] not in white_mill_dict_helper:
                            white_mill_dict_helper[k[1]] = True
                            new_mill == True

            for k in white_mill_dict_helper:   # prolazimo kroz sve mlinove i gledamoo koji vise nisu mlinovi
                if counter[k] < 3:
                    del white_mill_dict_helper[k]
            
            for k in white_positions_taken:
                if k[0] == 'd':
                    if 'd_low' in white_mill_dict_helper:
                        white_mill_dict['d1'] = True
                        white_mill_dict['d2'] = True
                        white_mill_dict['d3'] = True
                    elif 'd_high' in white_mill_dict_helper:
                        white_mill_dict['d5'] = True
                        white_mill_dict['d6'] = True
                        white_mill_dict['d7'] = True
                
                elif k[1] == '4':
                    if '4_left' in white_mill_dict_helper:
                        white_mill_dict['a4'] = True
                        white_mill_dict['b4'] = True
                        white_mill_dict['c4'] = True
                    elif '4_right' in white_mill_dict_helper:
                        white_mill_dict['e4'] = True
                        white_mill_dict['f4'] = True
                        white_mill_dict['g4'] = True

                elif k[0] in white_mill_dict_helper:
                    white_mill_dict[k] = True
                elif k[1] in white_mill_dict_helper:
                    white_mill_dict[k] = True

            print(new_mill)
            print(counter)
            return new_mill





class White_Piece(pygame.sprite.Sprite):

    def __init__(self, position):
        super(White_Piece, self).__init__()

        self.image = pygame.Surface((PIECE_SIZE*2, PIECE_SIZE*2))#, pygame.SRCALPHA)
        self.image.fill(BACKGROUND)
        self.image.set_colorkey(BACKGROUND)

        self.position = position

        self.rect = self.image.get_rect()
        self.rect.center = position

    def update(self):
            pygame.draw.circle(self.image, WHITE, (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE, 0)
            pygame.draw.circle(self.image, (15,15,15), (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE, 1)
            pygame.draw.circle(self.image, (15,15,15), (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE/2, 1)

    def set_position(self, position_XY):
        self.rect.center = position_XY


class Black_Piece(pygame.sprite.Sprite):

    def __init__(self, position):
        #pygame.sprite.Sprite.__init__(self)
        super(Black_Piece, self).__init__()

        self.image = pygame.Surface((PIECE_SIZE*2, PIECE_SIZE*2))#, pygame.SRCALPHA)
        self.image.fill(BACKGROUND)
        self.image.set_colorkey(BACKGROUND)

        self.position = position

        self.rect = self.image.get_rect()
        self.rect.center = position

    def update(self):
            pygame.draw.circle(self.image, BLACK, (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE, 0)
            pygame.draw.circle(self.image, (230,230,230), (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE, 1)
            pygame.draw.circle(self.image, (230,230,230), (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE/2, 1)

    def set_position(self, position_XY):
        self.rect.center = position_XY

