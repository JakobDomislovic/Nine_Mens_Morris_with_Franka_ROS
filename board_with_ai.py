from state_space_descriptor import state_space, position_on_board, mill_combinations

from alpha_beta_pruning import alpha_beta, mini_max
import alpha_beta_pruning as abp
#import GLOBAL_alfa_cnt, GLOBAL_beta_cnt, GLOBAL_node_max_cnt, GLOBAL_node_min_cnt

import numpy as np
import pygame
import random
import copy
import math
import time

########################################### GUI settings ##################################################
WIDTH  = 900
HEIGHT = 900
BACKGROUND = (255, 249, 174) # background color
FPS = 60                     # frames per sec
WHITE = (240, 240, 240)      # white player
BLACK = (20, 20, 20)         # black player
LINES = (245, 182, 84)       # line color
THICK = 5                    # line thickness
PIECE_SIZE = 25              # piece radius
########################################### GUI settings ##################################################

GLOBAL_search_depth = 0
GLOBAL_heur_choice = 0
GLOBAL_last_move = []


# group of sprites
all_pieces_list = pygame.sprite.Group() # pozivas kad oces sve uploadati na ekran
white_pieces_list = pygame.sprite.Group()
black_pieces_list = pygame.sprite.Group()


class Game_Board():

    def __init__(self, game_mode, depth, heuristic_choice, heuristic_choice2):
        
        self.global_move_counter = 0
        
        self.game_mode = game_mode
        
        self.depth = depth

        self.heur_choice1 = heuristic_choice
        self.heur_choice2 = heuristic_choice2

        global GLOBAL_search_depth
        GLOBAL_search_depth = depth
        
        global GLOBAL_heur_choice
        GLOBAL_heur_choice = heuristic_choice

        global GLOBAL_last_move
        GLOBAL_last_move = []

        self.history_white = []
        self.history_black = []
        
        global GLOBAL_alfa_cnt, GLOBAL_beta_cnt, GLOBAL_node_max_cnt, GLOBAL_node_min_cnt
        GLOBAL_alfa_cnt, GLOBAL_beta_cnt, GLOBAL_node_max_cnt, GLOBAL_node_min_cnt = 10,10,10,10
        
        self.A_prop_WHITE = []
        self.B_prop_WHITE = []
        self.time_WHITE = []
        self.time_mini_WHITE = []
        self.A_prop_BLACK = []
        self.B_prop_BLACK = []
        self.time_BLACK = []
        self.time_mini_BLACK = []

        self.mouse_click = False

        self.closed_positions = {}


        self.white_positions_taken  = {}
        self.white_mill_dict = {}
        self.white_mill_dict_helper = {}
        self.black_positions_taken  = {}
        self.black_mill_dict = {}
        self.black_mill_dict_helper = {}

        self.WHITE_PIECES = 9
        self.BLACK_PIECES = 9
        self.NUMBER_OF_PIECES = 18

        pygame.init()
        # Set up the drawing window
        self.screen = pygame.display.set_mode([WIDTH, HEIGHT])
        pygame.display.set_caption("Nine Men's Morris")
        pygame.display.flip()

        # kasnije dodaj jos da mozes birati ko je prvi na potezu 
        self.PLAYER_ON_MOVE = WHITE

        self.clock = pygame.time.Clock()
        self.make_mill_move = False

        if self.game_mode == 1: self.update_gui_AI() 
        elif self.game_mode == 2: self.update_gui_PvP()
        elif self.game_mode == 3: self.update_gui_AIvsAI()
    
    
    def update_gui_PvP(self):
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
                    self.mouse_click = True   # mozes ju napraviti lokalnom varijablom


            if self.PLAYER_ON_MOVE == WHITE and self.mouse_click:
                #if self.NUMBER_OF_PIECES: print('PRVA FAZA BIJELI:')
                #elif not self.NUMBER_OF_PIECES and self.WHITE_PIECES > 3: print('DRUGA FAZA BIJELI. upisi prvo poziciju koju zelis pomaknuti onda kamo pomaknuti')
                #else: print('Treca faza, klinki prvo poziciju kojeg zelis maknuti onda kamo.')

                self.mouse_click = False
                is_legal_move_made = self.make_move(self.PLAYER_ON_MOVE, [self.mx, self.my], 2, None)
                if is_legal_move_made:
                    #print('Trenutni u bijelom mlinu: {}'.format(self.white_mill_dict.keys()))
                    if self.is_mill(WHITE):
                        self.make_mill_move_white = True
                        self.PLAYER_ON_MOVE = 0
                    else: 
                        self.PLAYER_ON_MOVE = BLACK
                        

            elif self.make_mill_move_white:
                print('Kill black piece.')
                if self.kill_piece(WHITE, None):
                    self.PLAYER_ON_MOVE = BLACK
                    self.make_mill_move_white = False
                    self.trash = self.is_mill(BLACK)
                    #print('Crne figure nakon mlina: {}'.format(self.black_positions_taken.keys()))
                    

            elif self.PLAYER_ON_MOVE == BLACK:
                print('\nBlack player on turn.')
                if self.NUMBER_OF_PIECES: self.choose_place = input('Choose one of the empty fields: ')
                else: self.choose_place = input('Choose piece to move: ')
                if self.choose_place in position_on_board:
                    is_legal_move_made = self.make_move(self.PLAYER_ON_MOVE, position_on_board[self.choose_place], 2, None)
                    if is_legal_move_made:
                        if self.is_mill(BLACK):
                            self.make_mill_move_black = True
                            self.PLAYER_ON_MOVE = 0
                        else: 
                            self.PLAYER_ON_MOVE = WHITE
                            print('\nWhite player on turn.')

            elif self.make_mill_move_black:
                #print('Bijele figure prije mlina: {}'.format(self.white_positions_taken.keys()))
                if self.kill_piece(BLACK, None):
                    self.make_mill_move_black = False
                    self.PLAYER_ON_MOVE = WHITE
                    self.trash = self.is_mill(WHITE)
                    #print('Bijele figure poslije mlina: {}'.format(self.white_positions_taken.keys()))
                    print('\nWhite player on turn.')

            all_pieces_list.update()
            all_pieces_list.draw(self.screen)
            pygame.display.flip()

            # provjera je li doslo do kraja igre
            if self.white_positions_taken and not self.NUMBER_OF_PIECES and self.no_legal_moves_game_loss(WHITE) or self.WHITE_PIECES < 3:
                print('Black wins.')
                pygame.quit()
            if self.black_positions_taken and not self.NUMBER_OF_PIECES and self.no_legal_moves_game_loss(BLACK) or self.BLACK_PIECES < 3:
                print('White wins.')
                pygame.quit()
            # nerijeseno ispitaj jesi li u fazi dva  ili tri i sve figure koje postoje su u mlinu
            # TODO: check_draw

        pygame.quit()
        print(self.closed_positions.keys())


    def update_gui_AI(self):
        
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
                    self.mouse_click = True   # mozes ju napraviti lokalnom varijablom


            if self.PLAYER_ON_MOVE == WHITE and self.mouse_click:
               
                self.mouse_click = False
                is_legal_move_made = self.make_move(self.PLAYER_ON_MOVE, [self.mx, self.my], 1, None)
                if is_legal_move_made:
                    self.global_move_counter += 1
                    #print('Trenutni u bijelom mlinu: {}'.format(self.white_mill_dict.keys()))
                    if self.is_mill(WHITE):
                        self.make_mill_move_white = True
                        self.PLAYER_ON_MOVE = 0
                    else: 
                        self.PLAYER_ON_MOVE = BLACK
                        

            elif self.make_mill_move_white:
                print('Kill black piece.')
                if self.kill_piece(WHITE, None):
                    self.PLAYER_ON_MOVE = BLACK
                    self.make_mill_move_white = False
                    self.trash = self.is_mill(BLACK)
                    #print('Crne figure nakon mlina: {}'.format(self.black_positions_taken.keys()))
            
            elif self.PLAYER_ON_MOVE == BLACK:
                print('\nAI on turn.')
                
                board = set(self.closed_positions.keys())
                black_pieces = set(self.black_positions_taken.keys())
                white_pieces = set(self.white_positions_taken.keys())
                
                #print(list(board))
                #print(list(black_pieces))
                #print(list(white_pieces))

                if self.NUMBER_OF_PIECES > 0:
                    self.global_move_counter += 1
                    # in first stage we only put pieces down, we are never moving them
                    print('Prva faza crni')
                    new_move, evaluation_value, figure_to_kill, _ = alpha_beta(board, self.depth, True, float('-inf'),float('inf'),
                                                                                           white_pieces, black_pieces, self.NUMBER_OF_PIECES, None)
                    
                    print('Move: {}\nFigure to kill: {}'.format(new_move, figure_to_kill))

                    self.trash = self.make_move(self.PLAYER_ON_MOVE, position_on_board[new_move], 1, None)

                    time.sleep(1) # pauzira se program na sekundu da se bolje vidi stavljanje figure i ubijanje

                else:
                    if self.BLACK_PIECES > 3: print('Druga faza crni')
                    else: print('Treca faza crni')
                    self.global_move_counter += 1
                    new_move, evaluation_value, figure_to_kill, new_place = alpha_beta(board, self.depth, True, float('-inf'),float('inf'),
                                                                                           white_pieces, black_pieces, self.NUMBER_OF_PIECES, None)
                    
                    if not new_move:
                        all_pieces_list.update()
                        all_pieces_list.draw(self.screen)
                        pygame.display.flip()
                        print('White wins. There are no legal moves for Black player.')
                        print('Moves counter: {}'.format(self.global_move_counter))
                        time.sleep(10)
                        self.running = False
                        self.PLAYER_ON_MOVE = None
                        continue

                    print('Move: {}\nNew place: {}\nFigure to kill: {}'.format(new_move, new_place, figure_to_kill))
                    
                    self.trash = self.make_move(self.PLAYER_ON_MOVE, position_on_board[new_move], 1, new_place)

                    if new_move not in self.black_mill_dict:
                        global GLOBAL_last_move
                        GLOBAL_last_move.append((new_move, new_place))

                if figure_to_kill:
                    self.kill_piece(BLACK, figure_to_kill)
                
                self.PLAYER_ON_MOVE = WHITE
                self.trash = self.is_mill(WHITE)
                self.trash = self.is_mill(BLACK)
                # na kraju postavljas da igra bijeli igrac
                
                print('\nWhite player on turn.')
                
                if self.NUMBER_OF_PIECES: print('PRVA FAZA BIJELI:')
                elif not self.NUMBER_OF_PIECES and self.WHITE_PIECES > 3: print('DRUGA FAZA BIJELI. upisi prvo poziciju koju zelis pomaknuti onda kamo pomaknuti')
                else: print('Treca faza, klinki prvo poziciju kojeg zelis maknuti onda kamo.')



            all_pieces_list.update()
            all_pieces_list.draw(self.screen)
            pygame.display.flip()

            if self.global_move_counter == 100:
                print('Tie game.')
                time.sleep(5)
                pygame.quit()
            # provjera je li doslo do kraja igre
            
            if (self.white_positions_taken and not self.NUMBER_OF_PIECES and self.no_legal_moves_game_loss(WHITE)) or self.WHITE_PIECES < 3:
                print('Black wins.')
                print('Moves counter: {}'.format(self.global_move_counter))
                time.sleep(5)
                pygame.quit()
            if (self.black_positions_taken and not self.NUMBER_OF_PIECES and self.no_legal_moves_game_loss(BLACK)) or self.BLACK_PIECES < 3:
                print('White wins.')
                print('Moves counter: {}'.format(self.global_move_counter))
                time.sleep(5)
                pygame.quit()
            # nerijeseno ispitaj jesi li u fazi dva  ili tri i sve figure koje postoje su u mlinu
            # TODO: check_draw
        
        time.sleep(5)
        pygame.quit()


    def update_gui_AIvsAI(self):
        
        self.running = True
        self.make_mill_move_white = False
        self.make_mill_move_black = False

        self.gui_settings()
       
        
        if self.PLAYER_ON_MOVE == WHITE:
            # first move in ai_vs_ai is random
            print('\nAI white on turn.')
            print('First stage white.')

            first_move = random.choice(state_space.keys())
            print('Move: {}\nFigure to kill: None'.format(first_move))

            self.trash = self.make_move(self.PLAYER_ON_MOVE, position_on_board[first_move], 3, None)

            self.PLAYER_ON_MOVE = BLACK
        
        elif self.PLAYER_ON_MOVE == BLACK:
            # first move in ai_vs_ai is random
            print('\nAI black on turn.')
            print('First stage black.')

            first_move = random.choice(state_space.keys())
            print('Move: {}\nFigure to kill: None'.format(first_move))

            self.trash = self.make_move(self.PLAYER_ON_MOVE, position_on_board[first_move], 3, None)

            self.PLAYER_ON_MOVE = WHITE

        all_pieces_list.update()
        all_pieces_list.draw(self.screen)
        pygame.display.flip()

        while self.running:

            self.gui_settings()

            # check for events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
            
            if self.PLAYER_ON_MOVE == WHITE:
                
                global GLOBAL_heur_choice
                GLOBAL_heur_choice = self.heur_choice1

                print('\nAI white on turn.')

                board = set(self.closed_positions.keys())
                black_pieces = set(self.black_positions_taken.keys())
                white_pieces = set(self.white_positions_taken.keys())

                if self.NUMBER_OF_PIECES > 0:
                    self.global_move_counter += 1
                    # in first stage we only put pieces down, we are never moving them
                    print('First stage white.')

                    # ######################## MINIMAX - NEMA PODREZIVANJA #########################
                    # start = time.time()
                    # new_move, evaluation_value, figure_to_kill, _ = mini_max(board, self.depth, True, float('-inf'),float('inf'),
                    #                                                                        black_pieces, white_pieces, self.NUMBER_OF_PIECES, None)
                    # end = time.time()
                    # self.time_mini_WHITE.append(end-start)
                    # self.B_prop_WHITE.append(abp.GLOBAL_node_min_cnt)
                    # abp.GLOBAL_alfa_cnt, abp.GLOBAL_beta_cnt, abp.GLOBAL_node_max_cnt, abp.GLOBAL_node_min_cnt = 0,0,0,0
                    # ####################################################################

                    
                    start = time.time()
                    new_move, evaluation_value, figure_to_kill, _ = alpha_beta(board, self.depth, True, float('-inf'),float('inf'),
                                                                                           black_pieces, white_pieces, self.NUMBER_OF_PIECES, None)
                    end = time.time()
                    
                    # ####################################################################
                    # self.time_WHITE.append(end-start)
                    # self.A_prop_WHITE.append(abp.GLOBAL_node_max_cnt)
                    # abp.GLOBAL_alfa_cnt, abp.GLOBAL_beta_cnt, abp.GLOBAL_node_max_cnt, abp.GLOBAL_node_min_cnt = 0,0,0,0
                    # ####################################################################

                    print('Move: {}\nFigure to kill: {}'.format(new_move, figure_to_kill))

                    self.trash = self.make_move(self.PLAYER_ON_MOVE, position_on_board[new_move], 3, None)
                    
                    
                else:

                    if self.WHITE_PIECES > 3: print('Second stage white.')
                    else: print('Third stage white')
                    self.global_move_counter += 1
                    

                    # ######################## MINIMAX - NEMA PODREZIVANJA #########################
                    # start = time.time()
                    # new_move, evaluation_value, figure_to_kill, _ = mini_max(board, self.depth, True, float('-inf'),float('inf'),
                    #                                                                        black_pieces, white_pieces, self.NUMBER_OF_PIECES, None)
                    # end = time.time()
                    # self.time_mini_WHITE.append(end-start)
                    # self.B_prop_WHITE.append(abp.GLOBAL_node_min_cnt)
                    # abp.GLOBAL_alfa_cnt, abp.GLOBAL_beta_cnt, abp.GLOBAL_node_max_cnt, abp.GLOBAL_node_min_cnt = 0,0,0,0
                    # ####################################################################


                    start = time.time()
                    new_move, evaluation_value, figure_to_kill, new_place = alpha_beta(board, self.depth, True, float('-inf'),float('inf'),
                                                                                           black_pieces, white_pieces, self.NUMBER_OF_PIECES, None)
                    end = time.time()
                    
                    # ####################################################################
                    # self.time_WHITE.append(end-start)
                    # self.A_prop_WHITE.append(abp.GLOBAL_node_max_cnt)
                    # abp.GLOBAL_alfa_cnt, abp.GLOBAL_beta_cnt, abp.GLOBAL_node_max_cnt, abp.GLOBAL_node_min_cnt = 0,0,0,0
                    # ####################################################################
                    
                    if not new_move:
                        all_pieces_list.update()
                        all_pieces_list.draw(self.screen)
                        pygame.display.flip()
                        print('Black wins. There are no legal moves for White player.')
                        print('Moves counter: {}'.format(self.global_move_counter))                        
                        print('Depth: {}'.format(self.depth))
                        #self.results(self.time_mini_WHITE, self.time_WHITE, self.B_prop_WHITE, self.A_prop_WHITE, self.time_mini_BLACK, self.time_BLACK, self.B_prop_BLACK, self.A_prop_BLACK)
                        time.sleep(20)
                        exit()

                    if new_move not in self.white_mill_dict.keys():
                        global GLOBAL_last_move
                        GLOBAL_last_move = copy.deepcopy([(new_move, new_place)])
                        #print('WHITE: {}\n{}\n{}'.format(GLOBAL_last_move, self.history_white, self.history_black))
                        self.history_white = copy.deepcopy(GLOBAL_last_move)
                        GLOBAL_last_move = copy.deepcopy(self.history_black)
                    
                    print('Move: {}\nNew place: {}\nFigure to kill: {}'.format(new_move, new_place, figure_to_kill))
                    self.trash = self.make_move(self.PLAYER_ON_MOVE, position_on_board[new_move], 3, new_place)


                if figure_to_kill:
                    self.kill_piece(WHITE, figure_to_kill)
                
                self.PLAYER_ON_MOVE = BLACK
                self.trash = self.is_mill(BLACK)
                self.trash = self.is_mill(WHITE)
                # na kraju postavljas da igra bijeli igrac
                
                #time.sleep(1) # pauzira se program na sekundu da se bolje vidi stavljanje figure i ubijanje


            elif self.PLAYER_ON_MOVE == BLACK:
                
                global GLOBAL_heur_choice
                GLOBAL_heur_choice = self.heur_choice2

                print('\nAI black on turn.')
                
                board = set(self.closed_positions.keys())
                black_pieces = set(self.black_positions_taken.keys())
                white_pieces = set(self.white_positions_taken.keys())
                
                if self.NUMBER_OF_PIECES > 0:
                    self.global_move_counter += 1
                    # in first stage we only put pieces down, we are never moving them
                    print('First stage black.')
                    
                    # ######################## MINIMAX - NEMA PODREZIVANJA #########################
                    # start = time.time()
                    # new_move, evaluation_value, figure_to_kill, _ = mini_max(board, self.depth, True, float('-inf'),float('inf'),
                    #                                                                        black_pieces, white_pieces, self.NUMBER_OF_PIECES, None)
                    # end = time.time()                    
                    # self.time_mini_BLACK.append(end-start)
                    # self.B_prop_BLACK.append(abp.GLOBAL_node_min_cnt)
                    # abp.GLOBAL_alfa_cnt, abp.GLOBAL_beta_cnt, abp.GLOBAL_node_max_cnt, abp.GLOBAL_node_min_cnt = 0,0,0,0
                    # ####################################################################


                    start = time.time()                    
                    new_move, evaluation_value, figure_to_kill, _ = alpha_beta(board, self.depth, True, float('-inf'),float('inf'),
                                                                                           white_pieces, black_pieces, self.NUMBER_OF_PIECES, None)
                    end = time.time()
                    # ####################################################################
                    # self.time_BLACK.append(end-start)
                    # self.A_prop_BLACK.append(abp.GLOBAL_node_max_cnt)
                    # abp.GLOBAL_alfa_cnt, abp.GLOBAL_beta_cnt, abp.GLOBAL_node_max_cnt, abp.GLOBAL_node_min_cnt = 0,0,0,0
                    # ####################################################################


                    print('Move: {}\nFigure to kill: {}'.format(new_move, figure_to_kill))

                    self.trash = self.make_move(self.PLAYER_ON_MOVE, position_on_board[new_move], 1, None)

                    
                    time.sleep(1) # pauzira se program na sekundu da se bolje vidi stavljanje figure i ubijanje

                else:
                    if self.BLACK_PIECES > 3: print('Second stage black.')
                    else: print('Third stage black.')
                    self.global_move_counter += 1
                    
                    # ######################## MINIMAX - NEMA PODREZIVANJA #########################
                    # start = time.time()
                    # new_move, evaluation_value, figure_to_kill, _ = mini_max(board, self.depth, True, float('-inf'),float('inf'),
                    #                                                                        black_pieces, white_pieces, self.NUMBER_OF_PIECES, None)
                    # end = time.time()
                    # self.time_mini_BLACK.append(end-start)
                    # self.B_prop_BLACK.append(abp.GLOBAL_node_min_cnt)
                    # abp.GLOBAL_alfa_cnt, abp.GLOBAL_beta_cnt, abp.GLOBAL_node_max_cnt, abp.GLOBAL_node_min_cnt = 0,0,0,0
                    # ####################################################################

                    
                    
                    start = time.time()
                    new_move, evaluation_value, figure_to_kill, new_place = alpha_beta(board, self.depth, True, float('-inf'),float('inf'),
                                                                                           white_pieces, black_pieces, self.NUMBER_OF_PIECES, None)
                    end = time.time()
                    # ####################################################################
                    # self.time_BLACK.append(end-start)
                    # self.A_prop_BLACK.append(abp.GLOBAL_node_max_cnt)
                    # abp.GLOBAL_alfa_cnt, abp.GLOBAL_beta_cnt, abp.GLOBAL_node_max_cnt, abp.GLOBAL_node_min_cnt = 0,0,0,0
                    # ####################################################################


                    if not new_move:
                        all_pieces_list.update()
                        all_pieces_list.draw(self.screen)
                        pygame.display.flip()
                        print('White wins. There are no legal moves for Black player.')
                        print('Moves counter: {}'.format(self.global_move_counter))
                        print('Depth: {}'.format(self.depth))
                        #self.results(self.time_mini_WHITE, self.time_WHITE, self.B_prop_WHITE, self.A_prop_WHITE, self.time_mini_BLACK, self.time_BLACK, self.B_prop_BLACK, self.A_prop_BLACK)
                        time.sleep(20)
                        exit()

                    if new_move not in self.black_mill_dict.keys():
                        global GLOBAL_last_move
                        GLOBAL_last_move = copy.deepcopy([(new_move, new_place)])
                        #print('BLACK: {}\n{}\n{}'.format(GLOBAL_last_move, self.history_black, self.history_white))
                        self.history_black = copy.deepcopy(GLOBAL_last_move)
                        GLOBAL_last_move = copy.deepcopy(self.history_white)

                    print('Move: {}\nNew place: {}\nFigure to kill: {}'.format(new_move, new_place, figure_to_kill))
                    self.trash = self.make_move(self.PLAYER_ON_MOVE, position_on_board[new_move], 1, new_place)

                    
                if figure_to_kill:
                    self.kill_piece(BLACK, figure_to_kill)
                
                self.PLAYER_ON_MOVE = WHITE
                self.trash = self.is_mill(WHITE)
                self.trash = self.is_mill(BLACK)


            all_pieces_list.update()
            all_pieces_list.draw(self.screen)
            pygame.display.flip()
            time.sleep(1)

            if self.global_move_counter >= 75:
                print('Tie game.')
                print('Moves counter: {}'.format(self.global_move_counter))
                print('Depth: {}'.format(self.depth))
                # self.results(self.time_mini_WHITE, self.time_WHITE, self.B_prop_WHITE, self.A_prop_WHITE, self.time_mini_BLACK, self.time_BLACK, self.B_prop_BLACK, self.A_prop_BLACK)         
                time.sleep(5)
                exit()

            # provjera je li doslo do kraja igre
            if (self.white_positions_taken and not self.NUMBER_OF_PIECES and self.no_legal_moves_game_loss(WHITE)) or self.WHITE_PIECES < 3:
                print('\nBlack wins.')
                print('Moves counter: {}'.format(self.global_move_counter))
                print('Depth: {}'.format(self.depth))
                #self.results(self.time_mini_WHITE, self.time_WHITE, self.B_prop_WHITE, self.A_prop_WHITE, self.time_mini_BLACK, self.time_BLACK, self.B_prop_BLACK, self.A_prop_BLACK)         
                time.sleep(5)
                exit()

            if (self.black_positions_taken and not self.NUMBER_OF_PIECES and self.no_legal_moves_game_loss(BLACK)) or self.BLACK_PIECES < 3:
                print('White wins.')
                print('Moves counter: {}'.format(self.global_move_counter))
                print('Depth: {}'.format(self.depth))
                #self.results(self.time_mini_WHITE, self.time_WHITE, self.B_prop_WHITE, self.A_prop_WHITE, self.time_mini_BLACK, self.time_BLACK, self.B_prop_BLACK, self.A_prop_BLACK) 
                time.sleep(5)
                exit()
        
        time.sleep(5)
        pygame.quit()


    def results(self, time_mini_white, time_white, nodes_white, cut_white, time_mini_black, time_black, nodes_black, cut_black):
        time_mini_white = np.mean(time_mini_white)
        time_white = np.mean(time_white)
        ab_white = np.mean([(1-float(i)/j) for i,j in zip(cut_white, nodes_white)])

        time_mini_black = np.mean(time_mini_black)
        time_black = np.mean(time_black)
        ab_black = np.mean([(1-float(i)/j) for i,j in zip(cut_black, nodes_black)])
        
        print('WHITE: {}, {}, {}'.format(time_mini_white, time_white, ab_white))
        print('BLACK: {}, {}, {}'.format(time_mini_black, time_black, ab_black))


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
        '''
        prolazi kroz cijeli state space i trazi tocku najblizu onoj koju si kliknuo na ekranu
        ovo se radi i kada damo tocnu tocku, sto nije najbolje, ali za sada necu raditi promjene
        '''
        for self.k in position_on_board:
            self.d = (position_on_board[self.k][0] - x)**2 + (position_on_board[self.k][1] - y)**2
            if ( int(np.sqrt(self.d)) <= PIECE_SIZE ) and ( self.k not in self.closed_positions ):
                return self.k, True
        return None, False


    def make_move(self, player_color, position, mode, new_loc): # u class Piece isto imas position, NEMAJU VEZE JEDAN S DRUGIM
        '''
        radis poteze za svaku fazu
        '''

        if self.NUMBER_OF_PIECES:
            '''
            prva faza igre u kojoj si tako dugo dok ti self.NUMBER_OF_PIECES ne padne na nulu
            '''
            if player_color == WHITE:
                self.key, self.empty_spot = self.check_position(position[0], position[1])
                if self.empty_spot:
                    self.closed_positions[self.k] = position_on_board[self.k]
                    white = White_Piece(self.key)
                    self.white_positions_taken[self.key] = position_on_board[self.key]
                    all_pieces_list.add(white)
                    white_pieces_list.add(white)
                    self.NUMBER_OF_PIECES -= 1
                    return True # ako si napravio legalan potez vrati 'True'
                else:
                    return False

            else:
                self.key, self.empty_spot = self.check_position(position[0], position[1])
                if self.empty_spot:
                    self.closed_positions[self.k] = position_on_board[self.k]
                    black = Black_Piece(self.key)
                    self.black_positions_taken[self.key] = position_on_board[self.key]
                    all_pieces_list.add(black)
                    black_pieces_list.add(black)
                    self.NUMBER_OF_PIECES -= 1
                    return True # ako si napravio legalan potez vrati 'True'
                else:
                    return False


        elif player_color == WHITE and self.WHITE_PIECES > 3 and not self.NUMBER_OF_PIECES:
            '''
            mid-game human
            '''
            there_are_legal_moves = False
            for white_piece in white_pieces_list:
                if white_piece.rect.collidepoint(position[0], position[1]):
                    # ako si kliknuo na bijelu figuru provjeri ima li legalnih poteza
                    # dakle pogledaj je li BAREM JEDAN SUSJED SLOBODAN
                    for i in state_space[white_piece.key]:
                        if i not in self.closed_positions:
                            there_are_legal_moves = True
                            break
                if there_are_legal_moves: break

            if not there_are_legal_moves:
                print('There are no free adjacent fields for the chosen figure.')
                return False  # ako nema legalnih poteza vrati false i ponovo odaberi figuru
            
            white_piece.tag()
            all_pieces_list.draw(self.screen)
            pygame.display.flip()
            
            # ako je odabir figure dobar, odabrati mjesto za figuru
            #print('Odaberi novu lokaciju... mora biti susjedna')
            if mode == 3:
                new_location = new_loc
                time.sleep(1)

                key, flag = self.check_position(position_on_board[new_location][0], position_on_board[new_location][1])
                
                if flag:
                    for i in state_space[white_piece.key]:
                        if i == key:
                            #print('Nova odabrana pozicija je: {}'.format(key))
                            # dodavanje novog spritea
                            white_piece.kill_it()
                            new_white = White_Piece(key)
                            white_pieces_list.add(new_white)
                            all_pieces_list.add(new_white)

                            all_pieces_list.update()
                            all_pieces_list.draw(self.screen)
                            self.gui_settings()
                            pygame.display.flip()

                            del self.closed_positions[white_piece.key]
                            del self.white_positions_taken[white_piece.key]
                            self.closed_positions[key] = position_on_board[key]
                            self.white_positions_taken[key] = position_on_board[key]
                            #print('Bijele zauzete pozicije: {}'.format(self.white_positions_taken.keys()))
                            return True

            else:
                while True:
                    for event in pygame.event.get():
                        if event.type == pygame.MOUSEBUTTONDOWN:
                            new_x, new_y = pygame.mouse.get_pos()
                            key, flag = self.check_position(new_x, new_y)
                            if flag:
                                for i in state_space[white_piece.key]:
                                    if i == key:
                                        #print('Nova odabrana pozicija je: {}'.format(key))
                                        # dodavanje novog spritea
                                        white_piece.kill_it()
                                        new_white = White_Piece(key)
                                        white_pieces_list.add(new_white)
                                        all_pieces_list.add(new_white)

                                        all_pieces_list.update()
                                        all_pieces_list.draw(self.screen)
                                        self.gui_settings()
                                        pygame.display.flip()

                                        del self.closed_positions[white_piece.key]
                                        del self.white_positions_taken[white_piece.key]
                                        self.closed_positions[key] = position_on_board[key]
                                        self.white_positions_taken[key] = position_on_board[key]
                                        #print('Bijele zauzete pozicije: {}'.format(self.white_positions_taken.keys()))
                                        return True

            return False


        elif player_color == BLACK and self.BLACK_PIECES > 3 and not self.NUMBER_OF_PIECES:
            '''
            druga faza samo za crnog igraca
            '''
            there_are_legal_moves = False
            for black_piece in black_pieces_list:
                if black_piece.rect.collidepoint(position[0], position[1]):
                    for i in state_space[black_piece.key]:
                        if i not in self.closed_positions:
                            there_are_legal_moves = True
                            break
                    if there_are_legal_moves: break

            if not there_are_legal_moves:
                print('There are no free adjacent fields for the chosen figure.')
                return False
            
            black_piece.tag()
            all_pieces_list.draw(self.screen)
            pygame.display.flip()
            
            #print('Odaberi lokaciju kamo ces premjestiti figuru.')
            while True:
                if mode == 2: new_location = input('New location: ')
                else: 
                    new_location = new_loc
                    time.sleep(1) # pauzira se program na sekundu da se bolje vidi stavljanje figure i ubijanje

                key, flag = self.check_position(position_on_board[new_location][0], position_on_board[new_location][1])
                
                if flag:
                    for i in state_space[black_piece.key]:
                        if i == key:
                            #print('Nova odabrana pozicija je: {}'.format(key))
                            black_piece.kill_it()
                            new_black = Black_Piece(key)
                            black_pieces_list.add(new_black)
                            all_pieces_list.add(new_black)

                            all_pieces_list.update()
                            all_pieces_list.draw(self.screen)
                            self.gui_settings()
                            pygame.display.flip()

                            del self.closed_positions[black_piece.key]
                            del self.black_positions_taken[black_piece.key]
                            self.closed_positions[key] = position_on_board[key]
                            self.black_positions_taken[key] = position_on_board[key]
                            #print('Crne zauzete pozicije: {}'.format(self.black_positions_taken.keys()))
                            return True
            
            return False


        elif player_color == WHITE and self.WHITE_PIECES == 3 and not self.NUMBER_OF_PIECES:
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
            print('New location in third stage.')
            
            if mode == 3:
                new_location = new_loc
                time.sleep(1)
                
                key, flag = self.check_position(position_on_board[new_location][0], position_on_board[new_location][1])
                
                if flag and key not in self.closed_positions:
                    #print('Nova odabrana pozicija je: {}'.format(key))
                    # dodavanje novog spritea
                    white_piece.kill_it()
                    new_white = White_Piece(key)
                    white_pieces_list.add(new_white)
                    all_pieces_list.add(new_white)
                    all_pieces_list.update()
                    all_pieces_list.draw(self.screen)
                    self.gui_settings()
                    pygame.display.flip()
                    del self.closed_positions[white_piece.key]
                    del self.white_positions_taken[white_piece.key]
                    self.closed_positions[key] = position_on_board[key]
                    self.white_positions_taken[key] = position_on_board[key]
                    #print('Bijele zauzete pozicije: {}'.format(self.white_positions_taken.keys()))
                    return True

            else:
                while True:
                    for event in pygame.event.get():
                        if event.type == pygame.MOUSEBUTTONDOWN:
                            new_x, new_y = pygame.mouse.get_pos()
                            key, flag = self.check_position(new_x, new_y)
                            if flag and key not in self.closed_positions:
                                #print('Nova odabrana pozicija je: {}'.format(key))
                                # dodavanje novog spritea
                                white_piece.kill_it()
                                new_white = White_Piece(key)
                                white_pieces_list.add(new_white)
                                all_pieces_list.add(new_white)

                                all_pieces_list.update()
                                all_pieces_list.draw(self.screen)
                                self.gui_settings()
                                pygame.display.flip()

                                del self.closed_positions[white_piece.key]
                                del self.white_positions_taken[white_piece.key]
                                self.closed_positions[key] = position_on_board[key]
                                self.white_positions_taken[key] = position_on_board[key]
                                #print('Bijele zauzete pozicije: {}'.format(self.white_positions_taken.keys()))
                                return True

            return False

        else:
            for black_piece in black_pieces_list:
                if black_piece.rect.collidepoint(position[0], position[1]):
                    break

            black_piece.tag()
            all_pieces_list.draw(self.screen)
            pygame.display.flip()

            #print('Choose new location.')
            while True:
                if mode == 2: new_location = input('Write new location: ')
                else:
                    new_location = new_loc
                    time.sleep(1)

                key, flag = self.check_position(position_on_board[new_location][0], position_on_board[new_location][1])
                
                if flag and key not in self.closed_positions:
                    #print('Nova odabrana pozicija je: {}'.format(key))
                    black_piece.kill_it()
                    new_black = Black_Piece(key)
                    black_pieces_list.add(new_black)
                    all_pieces_list.add(new_black)

                    all_pieces_list.update()
                    all_pieces_list.draw(self.screen)
                    self.gui_settings()
                    pygame.display.flip()

                    del self.closed_positions[black_piece.key]
                    del self.black_positions_taken[black_piece.key]
                    self.closed_positions[key] = position_on_board[key]
                    self.black_positions_taken[key] = position_on_board[key]
                    #print('Crne zauzete pozicije: {}'.format(self.black_positions_taken.keys()))
                    return True
            
            return False


    def is_mill(self, player_color):
        '''
        mlin javljas samo ako je doslo do NOVO NAPRAVLJENOG MLINA
        zato cim naides na novi odma vracaj True jer nije moguce vise od jednog novog mlina
        '''
        #print('usao')
        #print(self.white_mill_dict_helper)
        #print(self.black_mill_dict_helper)
       
        new_mill = False

        if player_color == WHITE:
            wpt = self.white_positions_taken
            wmh = self.white_mill_dict_helper
        else:
            wpt = self.black_positions_taken
            wmh = self.black_mill_dict_helper

        wmd = {}

        for i in mill_combinations:
            if i[0] in wpt and i[1] in wpt and i[2] in wpt:
                if i not in wmh:
                    wmh[i] = True
                    new_mill = True
            elif i in wmh:
                del wmh[i]

        for k in wmh:
            for i in k:
                wmd[i] = True

        if player_color == WHITE:
            self.white_mill_dict = wmd
            self.white_mill_dict_helper = wmh
        else:
            self.black_mill_dict = wmd
            self.black_mill_dict_helper = wmh

        #print('Is there new mill? {}'.format(new_mill))
        return new_mill


    def no_legal_moves_game_loss(self, col):
        '''
        ako nemas legalnih poteza gubis vracas True, ako imas False
        '''
        
        if self.BLACK_PIECES == 3 or self.WHITE_PIECES  == 3: return False

        if col == WHITE:
            for k in self.white_positions_taken:
                for i in state_space[k]:
                    if i not in self.closed_positions: return False

        else:
            for k in self.black_positions_taken:
                for i in state_space[k]:
                    if i not in self.closed_positions: return False

        # ako su sva mjesta zauzeta neces vratiti false, nego ovaj true na kraju
        return True


    def kill_piece(self, color, figure_to_kill):
        # trebas koristiti 'sprite.kill()' jer ga tako mices iz svih grupa u kojima postoji
        is_killed = False
        all_in_mill = True
        if color == WHITE:
            # prvo provjeravas ima li uopce figura koje nisu u mlinu
            # ako su sve protivnicke u mlinu mozes ubiti bilo koju
            for k in self.black_positions_taken:
                if k not in self.black_mill_dict:
                    all_in_mill = False
                    break

            if not figure_to_kill:
                while not is_killed:
                    for event in pygame.event.get():
                        if event.type == pygame.MOUSEBUTTONDOWN:
                            x, y = pygame.mouse.get_pos()
                            for black_piece in black_pieces_list:
                                if black_piece.rect.collidepoint(x,y):
                                   if all_in_mill:
                                        black_piece.kill_it()
                                        del self.closed_positions[black_piece.key]
                                        del self.black_positions_taken[black_piece.key]
                                        self.BLACK_PIECES -= 1
                                        #print('Preostalo crnih figura: {}'.format(self.BLACK_PIECES))
                                        is_killed = True
                                        break

                                   elif black_piece.key not in self.black_mill_dict:
                                        black_piece.kill_it()
                                        del self.closed_positions[black_piece.key]
                                        del self.black_positions_taken[black_piece.key]
                                        self.BLACK_PIECES -= 1
                                        #print('Preostalo crnih figura: {}'.format(self.BLACK_PIECES))
                                        is_killed = True
                                        break

            elif figure_to_kill:
                key = figure_to_kill
                for black_piece in black_pieces_list:
                    if black_piece.key == key:
                       
                        if all_in_mill:
                            
                            black_piece.kill_it()
                            del self.closed_positions[black_piece.key]
                            del self.black_positions_taken[black_piece.key]
                            self.BLACK_PIECES -= 1
                            is_killed = True
                            break

                        elif key not in self.black_mill_dict:
                            
                            black_piece.kill_it()
                            del self.closed_positions[black_piece.key]
                            del self.black_positions_taken[black_piece.key]
                            self.BLACK_PIECES -= 1
                            #print('Preostalo crnih figura: {}'.format(self.BLACK_PIECES))
                            is_killed = True
                            break


        else:
            for k in self.white_positions_taken:
                if k not in self.white_mill_dict:
                    all_in_mill = False
                    break

            while not is_killed:
                
                if not figure_to_kill: key = input('Choose which white piece you want to kill: ')
                else: key = figure_to_kill

                for white_piece in white_pieces_list:
                    if white_piece.key == key:
                        if all_in_mill:          # ?????????????????????????????????
                            white_piece.kill_it()
                            del self.closed_positions[white_piece.key]
                            del self.white_positions_taken[white_piece.key]
                            self.WHITE_PIECES -= 1
                            #print('Preostalo bijelih figura: {}'.format(self.WHITE_PIECES))
                            is_killed = True
                            break

                        elif key not in self.white_mill_dict:
                            white_piece.kill_it()
                            del self.closed_positions[white_piece.key]
                            del self.white_positions_taken[white_piece.key]
                            self.WHITE_PIECES -= 1
                            #print('Preostalo bijelih figura: {}'.format(self.WHITE_PIECES))
                            is_killed = True
                            break

        return is_killed




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
        pygame.draw.circle(self.image, (170,0,170), (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE, 5)


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
        pygame.draw.circle(self.image, (170,0,170), (PIECE_SIZE,PIECE_SIZE), PIECE_SIZE, 5)
