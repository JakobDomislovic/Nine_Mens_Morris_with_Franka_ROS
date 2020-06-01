#!/usr/bin/env python

import rospy
from panda_sim.srv import CartesianGoal, CartesianGoalResponse, NamedGoal, \
                          NamedGoalResponse, VacuumGripperControl, VacuumGripperControlRequest

from geometry_msgs.msg import Pose 

import numpy as np
import pygame
import random
import copy
import math
import time

from alpha_beta_pruning import alpha_beta, mini_max
import alpha_beta_pruning as abp
from state_space_descriptor import state_space, position_on_board, mill_combinations

from franka_positons import franka_board_postions, franka_garbage_can, \
                            franka_white_pieces, franka_black_pieces, franka_board_center


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

        self.first_stage_white_frk = 0
        self.first_stage_black_frk = 0

        pygame.init()
        # Set up the drawing window
        self.screen = pygame.display.set_mode([WIDTH, HEIGHT])
        pygame.display.set_caption("Nine Men's Morris")
        pygame.display.flip()

        # kasnije dodaj jos da mozes birati ko je prvi na potezu 
        self.PLAYER_ON_MOVE = WHITE

        self.clock = pygame.time.Clock()
        self.make_mill_move = False
        
        # odma ga postavi u stow poziciju i iskljuci gripper
        #ret = self.named_positions('stow')
        ret = self.named_positions('home')
        ret = self.vac_gripper(False)

        self.gripper_safety_height = 0.08
        self.gripper_piece_height = 0.024

        self.first_stage_white_move_F = 0
        self.white_PICK_Franka = 0
        self.white_PLACE_Franka = 0
        self.white_franka_kill_move = 0

        self.update_gui_AI() 


    def update_gui_AI(self):
        
        self.running = True
        self.make_mill_move_white = False
        self.make_mill_move_black = False
        
        font = pygame.font.Font('freesansbold.ttf', 30)

        text_white = font.render('White', True, (245, 245, 245), (245, 182, 84))
        textRect_white = text_white.get_rect()
        textRect_white.center = (450, 30)

        text_white_kill = font.render('Kill black figure', True, (245, 245, 245), (209, 0, 28))
        textRect_white_kill = text_white_kill.get_rect()
        textRect_white_kill.center = (450, 30)
        
        text_black = font.render('Black', True, (20, 20, 20), (245, 182, 84))
        textRect_black = text_black.get_rect()
        textRect_black.center = (450, 30)

        text_black_kill = font.render('Kill white figure', True, (20, 20, 20), (209, 0, 28))
        textRect_black_kill = text_black_kill.get_rect()
        textRect_black_kill.center = (450, 30)

        text_white_win = font.render('White wins!', True, (245, 245, 245), (245, 182, 84))
        textRect_white_win = text_white_win.get_rect()
        textRect_white_win.center = (450, 30)

        text_black_win = font.render('Black wins!', True, (20, 20, 20), (245, 182, 84))
        textRect_black_win = text_black_win.get_rect()
        textRect_black_win.center = (450, 30)

        text_tie = font.render('Tie game', True, (74, 201, 37), (245, 182, 84))
        textRect_tie = text_tie.get_rect()
        textRect_tie.center = (450, 30)

        
        white_first_stage_franka_flag = False
        white_secthird_stage_franka_flag = False
        white_franka_mill_move = False

        black_first_stage_franka_flag = False
        black_secthird_stage_franka_flag = False
        black_franka_mill_move = False
    
        while self.running:

            self.gui_settings()

            if self.PLAYER_ON_MOVE == WHITE: self.screen.blit(text_white, textRect_white)
            if self.PLAYER_ON_MOVE == BLACK: self.screen.blit(text_black, textRect_black)
            if self.make_mill_move_white: self.screen.blit(text_white_kill, textRect_white_kill)

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
                    if self.is_mill(WHITE):
                        self.make_mill_move_white = True
                        self.PLAYER_ON_MOVE = 0
                    else:
                        # ako nema mlina posalji ruku u centar
                        #self.franka_2_center()
                        self.PLAYER_ON_MOVE = BLACK
                    # ako je bio legalni potez pokreni franku
                    if self.NUMBER_OF_PIECES: 
                        white_first_stage_franka_flag = True
                    else:
                        white_secthird_stage_franka_flag = True
                        

            elif self.make_mill_move_white:
                print('Kill black piece.')
                if self.kill_piece(WHITE, None):
                    white_franka_mill_move = True
                    self.PLAYER_ON_MOVE = BLACK
                    self.make_mill_move_white = False
                    self.trash = self.is_mill(BLACK)
                    self.kill_with_franka(self.black_franka_mill_move)
                    # ovdje je dobro mjesto da stavis da bijeli ubija
                    

            elif self.PLAYER_ON_MOVE == BLACK:
                print('\nAI on turn.')
                
                board = set(self.closed_positions.keys())
                black_pieces = set(self.black_positions_taken.keys())
                white_pieces = set(self.white_positions_taken.keys())
                

                if self.NUMBER_OF_PIECES > 0:
                    self.global_move_counter += 1
                    # in first stage we only put pieces down, we are never moving them
                    print('First stage black')
                    
                    new_move, evaluation_value, figure_to_kill, _ = alpha_beta(board, self.depth, True, float('-inf'),float('inf'),
                                                                                           white_pieces, black_pieces, self.NUMBER_OF_PIECES, None)

                    print('Move: {}\nFigure to kill: {}'.format(new_move, figure_to_kill))

                    self.trash = self.make_move(self.PLAYER_ON_MOVE, position_on_board[new_move], 1, None)

                    # posalji ruku da postavi figuru, a ako nema nikog za ubiti stavi ju na centar
                    black_first_stage_franka_flag = True
                    
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
                        self.screen.blit(text_white_win, textRect_white_win)
                        all_pieces_list.update()
                        all_pieces_list.draw(self.screen)
                        pygame.display.flip()
                        self.named_positions('stow')
                        time.sleep(10)
                        self.running = False
                        self.PLAYER_ON_MOVE = None
                        continue

                    print('Move: {}\nNew place: {}\nFigure to kill: {}'.format(new_move, new_place, figure_to_kill))
                    
                    self.trash = self.make_move(self.PLAYER_ON_MOVE, position_on_board[new_move], 1, new_place)

                    if new_move not in self.black_mill_dict:
                        global GLOBAL_last_move
                        GLOBAL_last_move.append((new_move, new_place))

                    black_secthird_stage_franka_flag = True
                    
                if figure_to_kill:
                    self.screen.blit(text_black_kill, textRect_black_kill)
                    all_pieces_list.update()
                    all_pieces_list.draw(self.screen)
                    pygame.display.flip()
                    self.kill_piece(BLACK, figure_to_kill)
                    black_franka_mill_move = True
                
                if new_move:
                    self.PLAYER_ON_MOVE = WHITE
                    self.trash = self.is_mill(WHITE)
                    self.trash = self.is_mill(BLACK)
                # na kraju postavljas da igra bijeli igrac
                
                print('\nWhite player on turn.')
                
                if self.NUMBER_OF_PIECES: print('First stage white.')
                elif not self.NUMBER_OF_PIECES and self.WHITE_PIECES > 3: print('Second stage white.')
                else: print('Third stage white.')


            all_pieces_list.update()
            all_pieces_list.draw(self.screen)
            pygame.display.flip()

            #### FRANKA START
            if white_first_stage_franka_flag:
                white_first_stage_franka_flag = False
                self.first_stage_franka_WHITE(self.first_stage_white_move_F)
            
            elif white_secthird_stage_franka_flag:
                white_secthird_stage_franka_flag = False
                self.second_and_third_stage_franka(self.white_PICK_Franka, self.white_PLACE_Franka)
            
            elif black_first_stage_franka_flag:
                black_first_stage_franka_flag = False
                self.first_stage_franka_BLACK(new_move)

            elif black_secthird_stage_franka_flag: 
                black_secthird_stage_franka_flag = False
                self.second_and_third_stage_franka(new_move, new_place)

            if white_franka_mill_move:
                white_franka_mill_move = False
                self.kill_with_franka(self.white_franka_mill_move)
            
            if black_franka_mill_move:
                black_franka_mill_move = False
                self.kill_with_franka(figure_to_kill)
            #### FRANKA END


            if self.global_move_counter == 50:
                print('Tie game.')
                self.screen.blit(text_tie, textRect_tie)
                all_pieces_list.update()
                all_pieces_list.draw(self.screen)
                pygame.display.flip()
                self.named_positions('stow')
                time.sleep(5)
                pygame.quit()
            # provjera je li doslo do kraja igre
            
            if (self.white_positions_taken and not self.NUMBER_OF_PIECES and self.no_legal_moves_game_loss(WHITE)) or self.WHITE_PIECES < 3:
                print('Black wins.')
                print('Moves counter: {}'.format(self.global_move_counter))
                self.screen.blit(text_black_win, textRect_black_win)
                all_pieces_list.update()
                all_pieces_list.draw(self.screen)
                pygame.display.flip()
                self.named_positions('home')
                time.sleep(5)
                pygame.quit()
            if (self.black_positions_taken and not self.NUMBER_OF_PIECES and self.no_legal_moves_game_loss(BLACK)) or self.BLACK_PIECES < 3:
                print('White wins.')
                print('Moves counter: {}'.format(self.global_move_counter))
                self.screen.blit(text_white_win, textRect_white_win)
                all_pieces_list.update()
                all_pieces_list.draw(self.screen)
                pygame.display.flip()
                self.named_positions('stow')
                time.sleep(5)
                pygame.quit()
            # nerijeseno ispitaj jesi li u fazi dva  ili tri i sve figure koje postoje su u mlinu
            # TODO: check_draw
        
        # ako je igra gotova posalji servis da ide u HOME
        time.sleep(5)
        pygame.quit()


    # start franka movement
    def first_stage_franka_WHITE(self, place):

        self.first_stage_white_frk += 1
        p = Pose()
        
        ##################### ovaj dio je uvijek isti ne diraj ga 
        p.orientation.x = 0.0 
        p.orientation.y = 1.0 
        p.orientation.z = 0.0 
        p.orientation.w = 0.0
        #####################
        # 1.) otidi do mjesta di su figure, ali malo iznad -- nek to uvijek bude z=0.4
        d = franka_white_pieces[self.first_stage_white_frk]
        p.position.x = d[0]
        p.position.y = d[1]
        #p.position.z = self.gripper_safety_height
        ## gripper dolazi malo iznad figura
        #ret = self.move_gripper(p)
        # gripper se spusta do figura
        p.position.z = d[2]
        ret = self.move_gripper(p)
        # pali se gripper
        ret = self.vac_gripper(True)
        # gipper se postavlja ponovo na visinu z=0.5 da se izbjegnu kolizije
        time.sleep(0.5)
        ret = self.named_positions('home')
        #p.position.z = self.gripper_safety_height
        #ret = self.move_gripper(p)

        # 2.) gripper se salje na poziciju na ploci, ali ponovo malo iznad --> z=0.4
        d = franka_board_postions[place]
        p.position.x = d[0]
        p.position.y = d[1]
        p.position.z = self.gripper_safety_height
        ret = self.move_gripper(p)
        # gripper se stavlja na visinu z=0.024 na kojoj i ispusta figure
        p.position.z = self.gripper_piece_height
        ret = self.move_gripper(p)
        # gasi se gripper da se ispusti figura
        ret = self.vac_gripper(False)
        # ponovo postavi griper na sigurnosnu visinu
        p.position.z = self.gripper_safety_height
        ret = self.move_gripper(p)
        ret = self.named_positions('home')


    def first_stage_franka_BLACK(self, place):

        self.first_stage_black_frk += 1
        p = Pose()
        
        ##################### ovaj dio je uvijek isti ne diraj ga 
        p.orientation.x = 0.0 
        p.orientation.y = 1.0 
        p.orientation.z = 0.0 
        p.orientation.w = 0.0
        #####################

        # 1.) otidi do mjesta di su figure, ali malo iznad -- nek to uvijek bude z=0.4
        d = franka_black_pieces[self.first_stage_black_frk]
        p.position.x = d[0]
        p.position.y = d[1]
        #p.position.z = self.gripper_safety_height
        ## gripper dolazi malo iznad figura
        #ret = self.move_gripper(p)
        # gripper se spusta do figura
        p.position.z = d[2]
        ret = self.move_gripper(p)
        # pali se gripper
        ret = self.vac_gripper(True)
        # gipper se postavlja ponovo na visinu z=0.5 da se izbjegnu kolizije
        time.sleep(0.5)
        ret = self.named_positions('home')
        #p.position.z = self.gripper_safety_height
        #ret = self.move_gripper(p)

        # 2.) gripper se salje na poziciju na ploci, ali ponovo malo iznad --> z=0.4
        d = franka_board_postions[place]
        p.position.x = d[0]
        p.position.y = d[1]
        p.position.z = self.gripper_safety_height
        ret = self.move_gripper(p)
        # gripper se stavlja na visinu z=0.024 na kojoj i ispusta figure
        p.position.z = self.gripper_piece_height
        ret = self.move_gripper(p)
        # gasi se gripper da se ispusti figura
        ret = self.vac_gripper(False)
        # ponovo postavi griper na sigurnosnu visinu
        p.position.z = self.gripper_safety_height
        ret = self.move_gripper(p)
        ret = self.named_positions('home')

    def second_and_third_stage_franka(self, pick, place):
        
        p = Pose()
        
        ##################### ovaj dio je uvijek isti ne diraj ga 
        p.orientation.x = 0.0 
        p.orientation.y = 1.0 
        p.orientation.z = 0.0 
        p.orientation.w = 0.0
        #####################

        # 1.) otidi do mjesta gdje je figura ali na malo sigurnosnu visinu
        d = franka_board_postions[pick]
        p.position.x = d[0]
        p.position.y = d[1]
        p.position.z = self.gripper_safety_height
        ## gripper dolazi iznad figure koju zeli preuzeti
        ret = self.move_gripper(p)
        # spustamo gripper do figure
        p.position.z = d[2]
        ret = self.move_gripper(p)
        # pali se gripper
        time.sleep(0.5)
        ret = self.vac_gripper(True)
        # pomicemo gripper opet malo iznad te pozicije
        #ret = self.named_positions('ready')
        p.position.z = self.gripper_safety_height
        ret = self.move_gripper(p)

        # 2.) postavljanje uzete figure na drugo mjesto
        d = franka_board_postions[place]
        p.position.x = d[0]
        p.position.y = d[1]
        p.position.z = self.gripper_safety_height
        ## gripper dolazi iznad figure koju zeli preuzeti
        ret = self.move_gripper(p)
        # spustamo gripper do figure
        p.position.z = d[2]
        ret = self.move_gripper(p)
        # gasi se gripper
        ret = self.vac_gripper(False)
        # pomicemo gripper opet malo iznad te pozicije --> sigurnosna visina
        p.position.z = self.gripper_safety_height
        ret = self.move_gripper(p)
        ret = self.named_positions('home')

    def kill_with_franka(self, kill):
        
        p = Pose()
        
        ##################### ovaj dio je uvijek isti ne diraj ga 
        p.orientation.x = 0.0 
        p.orientation.y = 1.0 
        p.orientation.z = 0.0 
        p.orientation.w = 0.0
        #####################

        # 1.) otidi do mjesta gdje je figura ali na malo sigurnosnu visinu
        d = franka_board_postions[kill]
        p.position.x = d[0]
        p.position.y = d[1]
        p.position.z = self.gripper_safety_height
        ## gripper dolazi iznad figure koju zeli preuzeti
        ret = self.move_gripper(p)
        # spustamo gripper do figure
        p.position.z = d[2]
        ret = self.move_gripper(p)
        # pali se gripper
        ret = self.vac_gripper(True)
        # pomicemo gripper opet malo iznad te pozicije
        time.sleep(0.5)
        self.named_positions('home')
        p.position.z = self.gripper_safety_height+0.2   # ovdje ides na malo vise da ne lupis u kutiju za smece
        ret = self.move_gripper(p)

        # 2.) postavljanje uzete figure u smece/groblje
        d = franka_garbage_can
        p.position.x = d[0]
        p.position.y = d[1]
        p.position.z = d[2]
        # gripper dolazi iznad figure koju zeli preuzeti
        ret = self.move_gripper(p)
        # gasi se gripper
        ret = self.vac_gripper(False)
        # vracanje grippera na sredinu ploce
        self.named_positions('home')
        

    def franka_2_center(self):
        
        p = Pose()
        
        ##################### ovaj dio je uvijek isti ne diraj ga 
        p.orientation.x = 0.0 
        p.orientation.y = 1.0 
        p.orientation.z = 0.0 
        p.orientation.w = 0.0
        #####################
        d = franka_board_center
        p.position.x = d[0]
        p.position.y = d[1]
        p.position.z = d[2]
        ret = self.move_gripper(p)
    # end franka movement


    # start calling services
    def named_positions(self, value):  #--> kod poziva dajes samo string
        rospy.wait_for_service('move_arm_named_pose')
        try:
            ret = rospy.ServiceProxy('move_arm_named_pose', NamedGoal)
            resp1 = ret(value)
            return True
        except rospy.ServiceException as e:
            print('Service call failed: %s'%e)


    def move_gripper(self, position):  #--> pos ti je poruka tipa Pose, a frm ti je bas frame --> uvijek bude Local
        frame = 'local'
        rospy.wait_for_service('move_arm_cartesian_pose')
        try:
            ret = rospy.ServiceProxy('move_arm_cartesian_pose', CartesianGoal)
            resp1 = ret(position, frame)
            return True
        except rospy.ServiceException as e:
            print('Service call failed: %s'%e)


    def vac_gripper(self, enable):
        try:
            rospy.wait_for_service('gripper/control', 1)
            control = rospy.ServiceProxy('gripper/control', VacuumGripperControl)
            req = VacuumGripperControlRequest()
            req.enable = enable
            control(req)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logerr('Service call failed with error: %s', e)
    # end calling services


    # start gui setting
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
    # end gui settings


    # start board logic
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
                    # OVDJE SELF.KEY TI JE POTEZ U PRVOJ FAZI --- ZA BIJELOG
                    self.first_stage_white_move_F = self.key
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
                    # OVDJE SELF.KEY TI JE POTEZ U PRVOJ FAZI --- ZA BIJELOG
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
                            # OVAJ 'i' BI TI MOGAO BITI ZA DRUGU FAZU PRVA FIGURA
                            there_are_legal_moves = True
                            break
                if there_are_legal_moves: break

            if not there_are_legal_moves:
                print('There are no free adjacent fields for the chosen figure.')
                return False  # ako nema legalnih poteza vrati false i ponovo odaberi figuru
            
            self.white_PICK_Franka = white_piece.key
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
                                        # OVAJ 'i' BI TI MOGAO BITI ZA DRUGU FAZU PRVA FIGURA
                                        # dodavanje novog spritea
                                        self.white_PLACE_Franka = key
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
            
            self.white_PICK_Franka = white_piece.key
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
                    self.white_PLACE_Franka = key
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
                key, flag = self.check_position(position_on_board[new_location][0], position_on_board[new_location][1])
                # ovdje bi mogao napraviti check position da dobijes pozu bijele koju trebas pomaknuti
                while True:
                    for event in pygame.event.get():
                        if event.type == pygame.MOUSEBUTTONDOWN:
                            new_x, new_y = pygame.mouse.get_pos()
                            key, flag = self.check_position(new_x, new_y)
                            if flag and key not in self.closed_positions:
                                #print('Nova odabrana pozicija je: {}'.format(key))
                                # dodavanje novog spritea
                                print('Third stage white, new location: {}'.format(key))
                                self.white_PLACE_Franka = key
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
                                        self.white_franka_kill_move = black_piece.key
                                        black_piece.kill_it()
                                        del self.closed_positions[black_piece.key]
                                        del self.black_positions_taken[black_piece.key]
                                        self.BLACK_PIECES -= 1
                                        #print('Preostalo crnih figura: {}'.format(self.BLACK_PIECES))
                                        is_killed = True
                                        break

                                    elif black_piece.key not in self.black_mill_dict:
                                        self.white_franka_kill_move = black_piece.key
                                        black_piece.kill_it()
                                        del self.closed_positions[black_piece.key]
                                        del self.black_positions_taken[black_piece.key]
                                        self.BLACK_PIECES -= 1
                                        #print('Preostalo crnih figura: {}'.format(self.BLACK_PIECES))
                                        is_killed = True
                                        break

            # ovo dalje je ako imas AI vs. AI
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
    # end board logic



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
