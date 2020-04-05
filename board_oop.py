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
white_mill_dict = {}
white_mill_dict_helper = {}
black_positions_taken  = {}
black_mill_dict = {}
black_mill_dict_helper = {}
# ovo su kratice gordnjih imena


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
                    if self.is_mill(WHITE):
                        print('MLIN MLIN NMLI MLIN  MLIN')
                    self.PLAYER_ON_MOVE = BLACK
                    print('Sljedeci na redu: {}'.format(self.PLAYER_ON_MOVE))
                    print('Trenutni u bijelom mlinu: {}'.format(white_mill_dict))
            elif self.PLAYER_ON_MOVE == BLACK:
                choose_place = input('Upisi poziciju za crnog: ')
                if choose_place in position_on_board:
                    is_legal_move_made = self.make_move(self.PLAYER_ON_MOVE, position_on_board[choose_place])
                    if is_legal_move_made:
                        if self.is_mill(BLACK):
                            print('MLIN MLIN CRNI CRNI')
                        self.PLAYER_ON_MOVE = WHITE
                    print('Sljedeci na redu: {}'.format(self.PLAYER_ON_MOVE))
                    print('Trenutni u crnom mlinu: {}'.format(black_mill_dict))

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
                print('Zauzeta mjesta: {}'.format(closed_positions.keys()))
                return self.k, True
        return [0,0], False


    def make_move(self, player_color, position): # u class Piece isto imas position, NEMAJU VEZE JEDAN S DRUGIM
        '''
        radis poteze za svaku fazu
        '''
        global WHITE_PIECES, BLACK_PIECES
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
                    #self.PLAYER_ON_MOVE = BLACK
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
                    #self.PLAYER_ON_MOVE = WHITE
                    self.NUMBER_OF_PIECES -= 1
                    return True # ako si napravio legalan potez vrati 'True'
                else:
                    return False

        elif player_color == WHITE and WHITE_PIECES > 3 and not self.NUMBER_OF_PIECES:
            '''
            druga faza igrem samo za bijelog igraca
            '''
            print('slozi mlin majmune')
        
        elif player_color == BLACK and BLACK_PIECES > 3 and not self.NUMBER_OF_PIECES:
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
        '''
        mlin javljas samo ako je doslo do NOVO NAPRAVLJENOG MLINA
        zato cim naides na novi odma vracaj True jer nije moguce vise od jednog novog mlina 
        '''
        print('usao')
        global white_mill_dict, white_mill_dict_helper
        global black_mill_dict, black_mill_dict_helper

        new_mill = False
        
        if player_color == WHITE:
            wpt = white_positions_taken
            wmh = white_mill_dict_helper
            wmd = white_mill_dict
        else:
            wpt = black_positions_taken
            wmd = black_mill_dict
            wmh = black_mill_dict_helper


        for i in mill_combinations:
            if i[0] in wpt and i[1] in wpt and i[2] in wpt:
                if i not in wmh:
                    wmh[i] = True
                    new_mill = True
            elif i in wmh: del wmh[i]

        for k in wmh:
            for i in k:
                wmd[i] = True


        if player_color == WHITE:
            white_mill_dict = wmd
            white_mill_dict_helper = wmh
        else:
            black_mill_dict = wmd
            black_mill_dict_helper = wmh


        print(new_mill)
        return new_mill




# mozda da svakom spriteu saljes poziciju na polju tipa 'a1' iz ne izvuces koordinate, 
# a svaki sprite ti onda ima i poziciju na polju

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

