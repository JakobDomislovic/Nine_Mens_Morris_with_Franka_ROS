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
white_mill_dict = {}            # sve mill_dictove napravi kao set, u ovom slucaju ti bolje
white_mill_dict_helper = {}
black_positions_taken  = {}
black_mill_dict = {}
black_mill_dict_helper = {}
# ovo su kratice gordnjih imena


# globalno upravljaj brojem figura pojedinih boja
WHITE_PIECES = 9
BLACK_PIECES = 9
NUMBER_OF_PIECES = 18

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

        self.clock = pygame.time.Clock()
        self.make_mill_move = False
        self.update_gui() # pozivam update gui fju koja je u stavri petlja do kad ne iskljucimo gui


    def update_gui(self):
        global mouse_click, WHITE_PIECES, BLACK_PIECES
        self.running = True
        self.make_mill_move_white = False
        self.make_mill_move_black = False
        while self.running:

            self.gui_settings()

            # check for events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    self.mx, self.my = pygame.mouse.get_pos()
                    mouse_click = True   # mozes ju napraviti lokalnom varijablom


            if self.PLAYER_ON_MOVE == WHITE and mouse_click:
                if NUMBER_OF_PIECES: print('PRVA FAZA BIJELI:')
                elif not NUMBER_OF_PIECES and WHITE_PIECES > 3: print('DRUGA FAZA BIJELI. upisi prvo poziciju koju zelis pomaknuti onda kamo pomaknuti')
                else: print('Treca faza, klinki prvo poziciju kojeg zelis maknuti onda kamo.')

                mouse_click = False
                is_legal_move_made = self.make_move(self.PLAYER_ON_MOVE, [self.mx, self.my])
                if is_legal_move_made:
                    print('Trenutni u bijelom mlinu: {}'.format(white_mill_dict.keys()))
                    if self.is_mill(WHITE):
                        self.make_mill_move_white = True
                        self.PLAYER_ON_MOVE = 0
                    else: self.PLAYER_ON_MOVE = BLACK
                    print('Sljedeci na redu: {}'.format(self.PLAYER_ON_MOVE))
                    print('Trenutni u bijelom mlinu: {}'.format(white_mill_dict.keys()))


            elif self.make_mill_move_white:
                print('Klikni misom kojeg crnog zelis ubiti.')
                print('Crne figure prije mlina: {}'.format(black_positions_taken.keys()))
                if self.kill_piece(WHITE):
                    self.PLAYER_ON_MOVE = BLACK
                    self.make_mill_move_white = False
                    print('Crne figure nakon mlina: {}'.format(black_positions_taken.keys()))


            elif self.PLAYER_ON_MOVE == BLACK:
                if NUMBER_OF_PIECES: print('PRVA FAZA CRNI:')
                elif not NUMBER_OF_PIECES and BLACK_PIECES > 3: print('DRUGA FAZA CRNI. upisi prvo poziciju koju zelis pomaknuti onda kamo pomaknuti')
                else: print('Treca faza, upisi prvo poziciju kojeg zelis maknuti onda kamo.')

                choose_place = input('Upisi poziciju za crnog: ')
                if choose_place in position_on_board:
                    is_legal_move_made = self.make_move(self.PLAYER_ON_MOVE, position_on_board[choose_place])
                    if is_legal_move_made:
                        print('Trenutno u crnom mlinu: {}'.format(black_mill_dict.keys()))
                        if self.is_mill(BLACK):
                            self.make_mill_move_black = True
                            self.PLAYER_ON_MOVE = 0
                        else: self.PLAYER_ON_MOVE = WHITE
                    print('Sljedeci na redu: {}'.format(self.PLAYER_ON_MOVE))
                    print('Trenutni u crnom mlinu: {}'.format(black_mill_dict.keys()))


            elif self.make_mill_move_black:
                print('Bijele figure prije mlina: {}'.format(white_positions_taken.keys()))
                if self.kill_piece(BLACK):
                    self.make_mill_move_black = False
                    self.PLAYER_ON_MOVE = WHITE
                    print('Bijele figure poslije mlina: {}'.format(white_positions_taken.keys()))



            all_pieces_list.update()
            all_pieces_list.draw(self.screen)
            pygame.display.flip()

            # provjera je li doslo do kraja igre
            if white_positions_taken and self.no_legal_moves_game_loss(WHITE) or WHITE_PIECES < 3:
                print('Crni je pobjedio.')
                pygame.quit()
            if black_positions_taken and self.no_legal_moves_game_loss(BLACK) or BLACK_PIECES < 3:
                print('Bijeli je pobjedio.')
                pygame.quit()
            # nerijeseno ispitaj jesi li u fazi dva  ili tri i sve figure koje postoje su u mlinu
            # TODO: check_draw

        pygame.quit()
        print(closed_positions.keys())

    def gui_settings(self):
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


    def draw_lines_for_board(self, start, end):
        pygame.draw.line(self.screen, LINES, start, end, THICK)
        pygame.draw.line(self.screen, LINES, start[::-1], end[::-1], THICK)
        pygame.draw.circle(self.screen, LINES, start, 10)
        pygame.draw.circle(self.screen, LINES, end, 10)
        pygame.draw.circle(self.screen, LINES, [(start[0]+end[0])/2, (start[1]+end[1])/2], 10)


    def check_position(self, x, y):
        global closed_positions
        '''
        prolazi kroz cijeli state space i trazi tocku najblizu onoj koju si kliknuo na ekranu
        ovo se radi i kada damo tocnu tocku, sto nije najbolje, ali za sada necu raditi promjene
        '''
        for self.k in position_on_board:
            self.d = (position_on_board[self.k][0] - x)**2 + (position_on_board[self.k][1] - y)**2
            if ( int(np.sqrt(self.d)) <= PIECE_SIZE ) and ( self.k not in closed_positions ):
                return self.k, True
        return None, False


    def make_move(self, player_color, position): # u class Piece isto imas position, NEMAJU VEZE JEDAN S DRUGIM
        '''
        radis poteze za svaku fazu
        '''
        global WHITE_PIECES, BLACK_PIECES, NUMBER_OF_PIECES, closed_positions
        global white_positions_taken, black_positions_taken

        if NUMBER_OF_PIECES:
            '''
            prva faza igre u kojoj si tako dugo dok ti NUMBER_OF_PIECES ne padne na nulu
            '''
            if player_color == WHITE:
                self.key, self.empty_spot = self.check_position(position[0], position[1])
                if self.empty_spot:
                    closed_positions[self.k] = position_on_board[self.k]
                    white = White_Piece(self.key)
                    white_positions_taken[self.key] = position_on_board[self.key]
                    all_pieces_list.add(white)
                    white_pieces_list.add(white)
                    NUMBER_OF_PIECES -= 1
                    return True # ako si napravio legalan potez vrati 'True'
                else:
                    return False

            else:
                self.key, self.empty_spot = self.check_position(position[0], position[1])
                if self.empty_spot:
                    closed_positions[self.k] = position_on_board[self.k]
                    black = Black_Piece(self.key)
                    black_positions_taken[self.key] = position_on_board[self.key]
                    all_pieces_list.add(black)
                    black_pieces_list.add(black)
                    NUMBER_OF_PIECES -= 1
                    return True # ako si napravio legalan potez vrati 'True'
                else:
                    return False


        elif player_color == WHITE and WHITE_PIECES > 3 and not NUMBER_OF_PIECES:
            '''
            mid-game covek
            '''
            there_are_legal_moves = False
            for white_piece in white_pieces_list:
                if white_piece.rect.collidepoint(position[0], position[1]):
                    # ako si kliknuo na bijelu figuru provjeri ima li legalnih poteza
                    # dakle pogledaj je li BAREM JEDAN SUSJED SLOBODAN
                    for i in state_space[white_piece.key]:
                        if i not in closed_positions:
                            there_are_legal_moves = True
                            break
                if there_are_legal_moves: break

            if not there_are_legal_moves:
                print('Figura koja je odabrana se ne moze nikamo pomaknuti.')
                return False  # ako nema legalnih poteza vrati false i ponovo odaberi figuru
            
            white_piece.tag()
            all_pieces_list.draw(self.screen)
            pygame.display.flip()
            
            # ako je odabir figure dobar, odabrati mjesto za figuru
            print('Odaberi novu lokaciju... mora biti susjedna')
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.MOUSEBUTTONDOWN:
                        new_x, new_y = pygame.mouse.get_pos()
                        key, flag = self.check_position(new_x, new_y)
                        if flag:
                            for i in state_space[white_piece.key]:
                                if i == key:
                                    print('Nova odabrana pozicija je: {}'.format(key))
                                    # dodavanje novog spritea
                                    white_piece.kill_it()
                                    new_white = White_Piece(key)
                                    white_pieces_list.add(new_white)
                                    all_pieces_list.add(new_white)
                                    
                                    all_pieces_list.update()
                                    all_pieces_list.draw(self.screen)
                                    self.gui_settings()
                                    pygame.display.flip()
                                    
                                    del closed_positions[white_piece.key]
                                    del white_positions_taken[white_piece.key]
                                    closed_positions[key] = position_on_board[key]
                                    white_positions_taken[key] = position_on_board[key]
                                    print('Bijele zauzete pozicije: {}'.format(white_positions_taken.keys()))
                                    return True

            return False


        elif player_color == BLACK and BLACK_PIECES > 3 and not NUMBER_OF_PIECES:
            '''
            druga faza samo za crnog igraca
            '''
            there_are_legal_moves = False
            for black_piece in black_pieces_list:
                if black_piece.rect.collidepoint(position[0], position[1]):
                    for i in state_space[black_piece.key]:
                        if i not in closed_positions:
                            there_are_legal_moves = True
                            break
                    if there_are_legal_moves: break

            if not there_are_legal_moves:
                print('Figura koja je odabrana se ne moze nikamo pomaknuti.')
                return False
            
            black_piece.tag()
            all_pieces_list.draw(self.screen)
            pygame.display.flip()
            
            print('Odaberi lokaciju kamo ces premjestiti figuru.')
            while True:
                new_location = input('Upisi novu lokaciju: ')
                key, flag = self.check_position(position_on_board[new_location][0], position_on_board[new_location][1])
                if flag:
                    for i in state_space[black_piece.key]:
                        if i == key:
                            print('Nova odabrana pozicija je: {}'.format(key))
                            black_piece.kill_it()
                            new_black = Black_Piece(key)
                            black_pieces_list.add(new_black)
                            all_pieces_list.add(new_black)

                            all_pieces_list.update()
                            all_pieces_list.draw(self.screen)
                            self.gui_settings()
                            pygame.display.flip()

                            del closed_positions[black_piece.key]
                            del black_positions_taken[black_piece.key]
                            closed_positions[key] = position_on_board[key]
                            black_positions_taken[key] = position_on_board[key]
                            print('Crne zauzete pozicije: {}'.format(black_positions_taken.keys()))
                            return True
            
            return False


        elif player_color == WHITE and WHITE_PIECES == 3 and not NUMBER_OF_PIECES:
            '''
            treca faza za bijelu figuricu
            '''
            for white_piece in white_pieces_list:
                if white_piece.rect.collidepoint(position[0], position[1]):
                    break

            white_piece.tag()
            all_pieces_list.draw(self.screen)
            pygame.display.flip()

            # ako je odabir figure dobar, odabrati mjesto za figuru
            print('Odaberi novu lokaciju... bilo koju osim one na kojima je vec figurica')
            while True:
                for event in pygame.event.get():
                    if event.type == pygame.MOUSEBUTTONDOWN:
                        new_x, new_y = pygame.mouse.get_pos()
                        key, flag = self.check_position(new_x, new_y)
                        if flag and key not in closed_positions:
                                print('Nova odabrana pozicija je: {}'.format(key))
                                # dodavanje novog spritea
                                white_piece.kill_it()
                                new_white = White_Piece(key)
                                white_pieces_list.add(new_white)
                                all_pieces_list.add(new_white)
                                
                                all_pieces_list.update()
                                all_pieces_list.draw(self.screen)
                                self.gui_settings()
                                pygame.display.flip()
                                
                                del closed_positions[white_piece.key]
                                del white_positions_taken[white_piece.key]
                                closed_positions[key] = position_on_board[key]
                                white_positions_taken[key] = position_on_board[key]
                                print('Bijele zauzete pozicije: {}'.format(white_positions_taken.keys()))
                                return True

            return False

        else:
            for black_piece in black_pieces_list:
                if black_piece.rect.collidepoint(position[0], position[1]):
                    break

            black_piece.tag()
            all_pieces_list.draw(self.screen)
            pygame.display.flip()

            print('Odaberi lokaciju kamo ces premjestiti figuru.')
            while True:
                new_location = input('Upisi novu lokaciju: ')
                key, flag = self.check_position(position_on_board[new_location][0], position_on_board[new_location][1])
                if flag and key not in closed_positions:
                    print('Nova odabrana pozicija je: {}'.format(key))
                    black_piece.kill_it()
                    new_black = Black_Piece(key)
                    black_pieces_list.add(new_black)
                    all_pieces_list.add(new_black)

                    all_pieces_list.update()
                    all_pieces_list.draw(self.screen)
                    self.gui_settings()
                    pygame.display.flip()

                    del closed_positions[black_piece.key]
                    del black_positions_taken[black_piece.key]
                    closed_positions[key] = position_on_board[key]
                    black_positions_taken[key] = position_on_board[key]
                    print('Crne zauzete pozicije: {}'.format(black_positions_taken.keys()))
                    return True
            
            return False


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


    def no_legal_moves_game_loss(self, col):
        '''
        ako nemas legalnih poteza gubis vracas True, ako imas False
        '''
        if col == WHITE:
            for k in white_positions_taken:
                for i in state_space[k]:
                    if i not in closed_positions: return False

        else:
            for k in black_positions_taken:
                for i in state_space[k]:
                    if i not in closed_positions: return False

        # ako su sva mjesta zauzeta neces vratiti false, nego ovaj true na kraju
        return True


    def kill_piece(self, color):
        # trebas koristiti 'sprite.kill()' jer ga tako mices iz svih grupa u kojima postoji
        global BLACK_PIECES, WHITE_PIECES
        is_killed = False
        all_in_mill = True
        if color == WHITE:
            # prvo provjeravas ima li uopce figura koje nisu u mlinu
            # ako su sve protivnicke u mlinu mozes ubiti bilo koju
            for k in black_positions_taken:
                if k not in black_mill_dict:
                    all_in_mill = False
                    break

            while not is_killed:
                for event in pygame.event.get():
                    if event.type == pygame.MOUSEBUTTONDOWN:
                        x, y = pygame.mouse.get_pos()
                        for black_piece in black_pieces_list:
                            if black_piece.rect.collidepoint(x,y):
                               if all_in_mill:
                                    black_piece.kill_it()
                                    del closed_positions[black_piece.key]
                                    del black_positions_taken[black_piece.key]
                                    BLACK_PIECES -= 1
                                    print('Preostalo crnih figura: {}'.format(BLACK_PIECES))
                                    is_killed = True
                                    break

                               elif black_piece.key not in black_mill_dict:
                                    black_piece.kill_it()
                                    del closed_positions[black_piece.key]
                                    del black_positions_taken[black_piece.key]
                                    BLACK_PIECES -= 1
                                    print('Preostalo crnih figura: {}'.format(BLACK_PIECES))
                                    is_killed = True
                                    break

        else:
            for k in white_positions_taken:
                if k not in white_mill_dict:
                    all_in_mill = False
                    break

            while not is_killed:
                key = input('Upisite poziciju bijelog kojeg zelite ubiti: ')
                for white_piece in white_pieces_list:
                    if white_piece.key == key:
                        if all_in_mill:
                            white_piece.kill_it()
                            del closed_positions[white_piece.key]
                            del white_positions_taken[white_piece.key]
                            WHITE_PIECES -= 1
                            print('Preostalo bijelih figura: {}'.format(WHITE_PIECES))
                            is_killed = True
                            break

                        elif key not in white_mill_dict:
                            white_piece.kill_it()
                            del closed_positions[white_piece.key]
                            del white_positions_taken[white_piece.key]
                            WHITE_PIECES -= 1
                            print('Preostalo bijelih figura: {}'.format(WHITE_PIECES))
                            is_killed = True
                            break

        return is_killed


# mozda da svakom spriteu saljes poziciju na polju tipa 'a1' iz ne izvuces koordinate,
# a svaki sprite ti onda ima i poziciju na polju

class White_Piece(pygame.sprite.Sprite):

    def __init__(self, key):
        super(White_Piece, self).__init__()

        self.image = pygame.Surface((PIECE_SIZE*2, PIECE_SIZE*2))#, pygame.SRCALPHA)
        self.image.fill(BACKGROUND)
        self.image.set_colorkey(BACKGROUND)

        self.key = key
        self.position = position_on_board[key]

        self.rect = self.image.get_rect()
        self.rect.center = self.position

    def update(self):
            pygame.draw.circle(self.image, WHITE, (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE, 0)
            pygame.draw.circle(self.image, (15,15,15), (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE, 1)
            pygame.draw.circle(self.image, (15,15,15), (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE/2, 1)

    def set_position(self, position_XY):
        self.rect.center = position_XY

    def kill_it(self):
        self.kill()

    def tag(self):
        pygame.draw.circle(self.image, (128,0,128), (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE, 5)


class Black_Piece(pygame.sprite.Sprite):

    def __init__(self, key):
        #pygame.sprite.Sprite.__init__(self)
        super(Black_Piece, self).__init__()

        self.image = pygame.Surface((PIECE_SIZE*2, PIECE_SIZE*2))#, pygame.SRCALPHA)
        self.image.fill(BACKGROUND)
        self.image.set_colorkey(BACKGROUND)

        self.key = key
        self.position = position_on_board[self.key]

        self.rect = self.image.get_rect()
        self.rect.center = self.position

    def update(self):
            pygame.draw.circle(self.image, BLACK, (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE, 0)
            pygame.draw.circle(self.image, (230,230,230), (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE, 1)
            pygame.draw.circle(self.image, (230,230,230), (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE/2, 1)

    def set_position(self, position_XY):
        self.rect.center = position_XY

    def kill_it(self):
        self.kill()

    def tag(self):
        pygame.draw.circle(self.image, (128,0,128), (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE, 5)
