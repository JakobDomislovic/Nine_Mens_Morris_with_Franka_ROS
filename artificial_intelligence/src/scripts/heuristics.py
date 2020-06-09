from state_space_descriptor import state_space, super_move_formations, mill_combinations
import board_with_ai # global variable GLOBAL_search_depth

import copy

def number_of_pieces_heuristic(black, white, number_of_pieces, mill_move_flag, player, depth):
    '''
    comparing number of pieces
    '''
    evaluation = 0

    if number_of_pieces:
        # stage 1
        evaluation += (len(black) - len(white)) * 10
    
    else:
        # if stage 2 or 3 value mill more

        evaluation += (len(black) - len(white)) * 30
        
        if mill_move_flag:
                #print('Mill move flag TRUE')
            if player: 
                evaluation += -100
                if depth == board_with_ai.GLOBAL_search_depth - 1: 
                    evaluation += -100
            else: 
                evaluation += 100
                if depth == board_with_ai.GLOBAL_search_depth - 1: 
                    evaluation += 100
        
    return evaluation


def first_stage_heuristics(black, white, max_player):
    '''
    just for fun to see if AI is controllable
    '''
    sum_black = 0
    sum_white = 0

    for i in black:
        if i == 'a1' or i == 'a4' or i == 'a7' or i == 'd1' or i == 'd7' or i == 'g1' or i == 'g4' or i == 'g7':
            sum_black += 50

        elif i == 'b2' or i == 'b4' or i == 'b6' or i == 'd2' or i == 'd6' or i == 'f2' or i == 'f4' or i == 'f6':
            sum_black += 25
        
        elif i == 'c3' or i == 'c4' or i == 'c5' or i == 'd3' or i == 'd5' or i == 'e3' or i == 'e4' or i == 'e5':
            sum_black += 15

    for i in white:
        if i == 'a1' or i == 'a4' or i == 'a7' or i == 'd1' or i == 'd7' or i == 'g1' or i == 'g4' or i == 'g7':
            sum_white -= 50

        elif i == 'b2' or i == 'b4' or i == 'b6' or i == 'd2' or i == 'd6' or i == 'f2' or i == 'f4' or i == 'f6':
            sum_white -= 25
        
        elif i == 'c3' or i == 'c4' or i == 'c5' or i == 'd3' or i == 'd5' or i == 'e3' or i == 'e4' or i == 'e5':
            sum_white -= 15

    return (sum_black + sum_white) * 5


def try_2_block_pieces(black, white, number_of_pieces, mill_move_flag, player, depth):
    '''
    heuristic is leading AI to block all enemy figures, while also trying to form mill
    '''

    evaluation = 0

    white_blocking, black_blocking = blocked_by_piece(black, white)
    length_black = len(black)
    length_white = len(white)

    if number_of_pieces:
        if not player: evaluation -= (black_blocking) * 50
        else: evaluation += (white_blocking) * 50
        evaluation += (length_black - length_white) * 25 

    elif not player and length_black > 3:
        # u drugoj i trecoj fazi vise cijeni mlinove nego u prvoj
        evaluation += (black_blocking) * 30
        evaluation += (length_black - length_white) * 50
    
    elif player and length_white > 3:
        evaluation -= (white_blocking) * 30
        evaluation += (length_black - length_white) * 50
    
    else:
        evaluation += (length_black - length_white) * 100
        # if depth == board_with_ai.GLOBAL_search_depth - 1: 
        #     if player: evaluation += -1000
        #     else: evaluation += 1000 

    if mill_move_flag:
        if player: evaluation += -10000
        else: evaluation += 10000
   
    return evaluation


def number_of_blocked_pieces(black, white, number_of_pieces, mill_move_flag, player, depth):
    '''
    looks for number of blocked pieces and at number of pieces (if you can make mill and block somebody why not...)
    '''

    evaluation = 0

    if depth == 4: print('Depth 0 heuristic')

    no_adjacent_white, no_adjacent_black = no_adjacent(black, white)
    length_black = len(black)
    length_white = len(white)

    if number_of_pieces:
        evaluation += (no_adjacent_white - no_adjacent_black) * 50
        evaluation += (length_black - length_white) * 10 

    elif not player and length_black > 3:
        # u drugoj i trecoj fazi vise cijeni mlinove nego u prvoj
        evaluation += (no_adjacent_white - no_adjacent_black) * 50
        evaluation += (length_black - length_white) * 100
    
    elif player and length_white > 3:
        # u drugoj i trecoj fazi vise cijeni mlinove nego u prvoj
        evaluation += (no_adjacent_white - no_adjacent_black) * 50
        evaluation += (length_black - length_white) * 100
    
    else:
        evaluation += (length_black - length_white) * 1000
        if depth == board_with_ai.GLOBAL_search_depth - 1: 
            if player: evaluation += -1000
            else: evaluation += 1000 
    
    if mill_move_flag:
        if player: evaluation += -1000
        else: evaluation += 1000
    

    return evaluation


def try_different_mills(black, white, number_of_pieces, mill_move_flag, player, depth):
    '''
    different mill, and possible mills configs
    '''

    evaluation = 0

    length_black = len(black)
    length_white = len(white)

    white_two_piece, black_two_piece = two_piece_config(black, white)
    white_three_piece, black_three_piece = three_piece_config(black, white)
    white_super_mill, black_super_mill = super_move_formation(black, white)
    
    if number_of_pieces:
        # first stage
        if mill_move_flag: evaluation += 20
        evaluation += (length_black - length_white) * 50
        evaluation += (black_two_piece - white_two_piece) * 10
        evaluation += (black_three_piece - white_three_piece) * 7

    elif not player and length_black > 3:
        # u drugoj i trecoj fazi vise cijeni mlinove nego u prvoj
        if mill_move_flag: evaluation += 90
        evaluation += (length_black - length_white) * 50
        evaluation += (black_three_piece - white_three_piece) * 5
        evaluation += (black_super_mill - white_super_mill) * 10
    
    elif player and length_white > 3:
        # u drugoj i trecoj fazi vise cijeni mlinove nego u prvoj
        if mill_move_flag: evaluation += -90
        evaluation += (length_black - length_white) * 15
        evaluation += (black_two_piece - white_two_piece) * 5
        evaluation += (black_three_piece - white_three_piece) * 5
        evaluation += (black_super_mill - white_super_mill) * 10
    
    else:
        if player:
            if mill_move_flag: evaluation += -100
        else:
            evaluation += 100
        evaluation += (length_black - length_white) * 100
        evaluation += (black_two_piece - white_two_piece) * 50
        evaluation += (black_three_piece - white_three_piece) * 0

    if depth == board_with_ai.GLOBAL_search_depth - 1:
        if mill_move_flag:
            if player:
                evaluation += -1000
            else:
                evaluation += 1000

    return evaluation


def advanced_heuristic(black, white, number_of_pieces, mill_move_flag, player, depth):
    
    evaluation = 0

    # if number_of_pieces:
    #     black_branch_factor = branch_factor(black.union(white), black, white, not player, 1, depth)
    #     white_branch_factor = branch_factor(black.union(white), black, white, player, 1, depth)

    # else:
    #     if len(black) > 3:
    #         black_branch_factor = branch_factor(black.union(white), black, white, not player, 2, depth)
    #     else:
    #         black_branch_factor = branch_factor(black.union(white), black, white, not player, 3, depth)
        
    #     if len(white) > 3:
    #         white_branch_factor = branch_factor(black.union(white), black, white, player, 2, depth)
    #     else:
    #         white_branch_factor = branch_factor(black.union(white), black, white, player, 3, depth)
        
    
    
    # branch_factor_diff = black_branch_factor - white_branch_factor

    length_black = len(black)
    length_white = len(white)
    length_diff = length_black - length_white

    white_num_morris, black_num_morris = number_of_morrises(black, white)
    morrises_diff = black_num_morris - white_num_morris

    white_no_adj, black_no_adj = no_adjacent(black, white)
    no_adj_diff = white_no_adj - black_no_adj # pazi ovdje oduzimas protivnik - igrac jer ti je cilj da protivnik ima sto manje susjednih mjesta slobodno

    white_two_piece, black_two_piece = two_piece_config(black, white)
    two_piece_diff = black_two_piece - white_two_piece

    white_three_piece, black_three_piece = three_piece_config(black, white)
    three_piece_diff = black_three_piece - white_three_piece

    white_super_mill, black_super_mill = super_move_formation(black, white)
    super_mill_diff =  black_super_mill - white_super_mill

    if number_of_pieces:
        # first stage
        if mill_move_flag: evaluation += 20
        evaluation += morrises_diff * 30
        evaluation += no_adj_diff * 10
        evaluation += length_diff * 30
        evaluation += two_piece_diff * 15
        evaluation += three_piece_diff * 7
        #evaluation += branch_factor_diff * 5


    elif not player and length_black > 3:
        # u drugoj i trecoj fazi vise cijeni mlinove nego u prvoj
        if mill_move_flag: evaluation += 30
        evaluation += morrises_diff * 40
        evaluation += no_adj_diff * 50
        evaluation += length_diff * 50
        evaluation += super_mill_diff * 30
        #evaluation += branch_factor_diff * 30

    elif player and length_white > 3:
        # u drugoj i trecoj fazi vise cijeni mlinove nego u prvoj
        if mill_move_flag: evaluation += -30
        evaluation += morrises_diff * 40
        evaluation += no_adj_diff * 50
        evaluation += length_diff * 50
        evaluation += super_mill_diff * 30
        #evaluation += branch_factor_diff * 30

    else:
        if player:
            if mill_move_flag: evaluation += -50
        else:
            evaluation += 50
        evaluation += length_diff * 50
        evaluation += two_piece_diff * 5
        evaluation += three_piece_diff * 0
        #evaluation += branch_factor_diff * 1
        
        
    # if depth == board_with_ai.GLOBAL_search_depth - 1:
    #     if mill_move_flag:
    #         if player:
    #             evaluation += -1000
    #         else:
    #             evaluation += 1000

    return evaluation


def branch_factor_heuristic(black, white, number_of_pieces, mill_move_flag, player, depth):
    
    evaluation = 0
    
    if number_of_pieces:
        black_branch_factor = branch_factor(black.union(white), black, white, not player, 1, depth)
        white_branch_factor = branch_factor(black.union(white), black, white, player, 1, depth)

    if len(black) > 3:
        black_branch_factor = branch_factor(black.union(white), black, white, not player, 2, depth)
    
    if len(white) > 3:
        white_branch_factor = branch_factor(black.union(white), black, white, player, 2, depth)
    
    if len(black) == 3:
        black_branch_factor = branch_factor(black.union(white), black, white, not player, 3, depth)
        
    if len(white) == 3:
        white_branch_factor = branch_factor(black.union(white), black, white, player, 3, depth)
    
    branch_factor_diff = black_branch_factor - white_branch_factor

    if number_of_pieces:
        return branch_factor_diff * 10
    else:
        return branch_factor_diff * 30


########################################      HELPER FUNCTIONS      ########################################

def blocked_by_piece(black, white):
    
    # pomocna fja koja vraca koliko figura odredena boja blokira

    number_of_blocked_black = 0
    number_of_blocked_white = 0

    for wh in white:
        for spot in state_space[wh]:
            if spot in black:
                number_of_blocked_black += 1

    for bl in black:
        for spot in state_space[bl]:
            if spot in white:
                number_of_blocked_white += 1

    return number_of_blocked_white, number_of_blocked_black


def no_adjacent(black, white):
    
    # pomocna fja koja vraca koliko figura odredena boja blokira

    no_adjacent_white = 0
    no_adjacent_black = 0

    for wh in white:
        t = True
        for spot in state_space[wh]:
            if spot not in black or spot not in white:
                t = False
                break
        if t: no_adjacent_white += 1

    for bl in black:
        t = True
        for spot in state_space[bl]:
            if spot not in white or spot not in black:
                t = False
                break
        if t: no_adjacent_black += 1

    return  no_adjacent_white, no_adjacent_black


def super_move_formation(black, white):
    
    count_super_black = 0
    count_super_white = 0
   
    for super_move in super_move_formations:
        if super_move.issubset(black): count_super_black += 1

    for super_move in super_move_formations:
        if super_move.issubset(white): count_super_white += 1

    return count_super_white, count_super_black


def three_piece_config(black, white):
    
    count_white = 0
    count_black = 0

    for bl in black:
        if bl[0]=='d' or bl[1]=='4': continue
        cnt = 0
        for fig in state_space[bl]:
            if fig in black: cnt += 1
        if cnt == 2: count_black += 1
    
    for wh in white:
        if wh[0]=='d' or wh[1]=='4': continue
        cnt = 0
        for fig in state_space[wh]:
            if fig in white: cnt += 1
        if cnt == 2: count_white += 1

    return count_white, count_black
    


def two_piece_config(black, white):
    
    count_white = 0
    count_black = 0

    for bl in black:
        cnt = 0
        for fig in state_space[bl]:
            if fig in black: cnt += 1
        if cnt == 1: count_black += 1

    for wh in white:
        cnt = 0
        for fig in state_space[wh]:
            if fig in white: cnt += 1
        if cnt == 1: count_white += 1

    return count_white, count_black


def number_of_morrises(black, white):

    count_white = 0
    count_black = 0

    for mill in mill_combinations:
        flag_white = True
        flag_black = True
        for m in mill:
            if m not in black:
                flag_black = False
                break
        if flag_black:
            count_black += 1
            continue # ako je nesto u crnom mlinu ne moze biti i u bijelom

        for m in mill:
            if m not in white:
                flag_white = False
                break
        if flag_white:
            count_white += 1

    return count_white, count_black



def is_Mill(color, move):
    
    for mill in mill_combinations:
        help2 = set()
        
        for t in mill: help2.add(t)
        
        if move in help2 and help2.issubset(color):
            return True

    return False 


def is_Terminal(board, player, white, black):
    
    if not player:
        if len(white) < 3 and len(board) >= 5:  # minimalno na ploci moze biti 5 igraca
            return True
        if len(white) > 3:
            for k in white:
                for i in state_space[k]:
                    if i not in board: return False # cim se nade slobodna susjedna pozicija nije terminal
            return True

    else:
        if len(black) < 3 and len(board) >= 5: # minimalno na ploci moze biti 5 igraca
            return True

        if len(black) > 3:
            for k in black:
                for i in state_space[k]:
                    if i not in board: return False # cim se nade slobodna susjedna pozicija nije terminal
            return True


def branch_factor(board_x, black_x, white_x, max_player, stage, depth):
    
    # razmisli treba li ti uopce 'deepcopy', to je O(n)

    not_in_mill = set()
    board = copy.deepcopy(board_x)
    
    board_list = []
    first_col_list = []
    second_col_list = []

    in_mill_flag_list = []

    if max_player:
        
        first_col = copy.deepcopy(black_x)
        second_col = copy.deepcopy(white_x)

        initial_second = copy.deepcopy(white_x)
        help4 = set()

        if len(white_x) == 3: not_in_mill = white_x
        else:
            for m in mill_combinations:
                help1 = set()
                for t in m: help1.add(t)
                for i in white_x:
                    not_in_mill.add(i)
                    if i in help1 and help1.issubset(white_x):
                        help4.add(i)

            if help4 == white_x: not_in_mill = white_x   # ako su sve figure u mlinu onda smijes ubiti bilo koju 
            else: not_in_mill.difference_update(help4)
    
    else:
        
        first_col = copy.deepcopy(white_x)
        second_col = copy.deepcopy(black_x)
 
        initial_second = copy.deepcopy(black_x)
        help4 = set()

        if len(black_x) == 3: not_in_mill = black_x
        else:
            for m in mill_combinations:
                help1 = set()
                for t in m: help1.add(t)
                for i in black_x:
                    not_in_mill.add(i)
                    if i in help1 and help1.issubset(black_x):
                        help4.add(i)

            if help4 == black_x: not_in_mill = black_x   # ako su sve figure u mlinu onda smijes ubiti bilo koju 
            else: not_in_mill.difference_update(help4)
    
    if stage == 1:
        
        for move in state_space.keys():

            if move in board: continue

            help2 = set()
            help2 = first_col.union({move})

            first_col_list.append(help2)

            if not_in_mill and is_Mill(help2, move):

                for fig in not_in_mill:

                    in_mill_flag_list.append(True)

                continue

            in_mill_flag_list.append(False)

        return len(in_mill_flag_list)

    
    elif stage == 2:

        for figure in first_col:
           
            help_first_col = first_col.difference({figure})
            
            for adjacent in state_space[figure]:

                if depth == board_with_ai.GLOBAL_search_depth and board_with_ai.GLOBAL_last_move:
                    compare0 = board_with_ai.GLOBAL_last_move[0][1] # figura koju trenutno promatras ista kao novi potez u proslom koraku
                    compare1 = board_with_ai.GLOBAL_last_move[0][0] # ako je susjedni (novo moguce polje) isti kao proslo mjesto s kojeg smo krenuli
                    #print(figure, compare0)
                    #print(adjacent, compare1)
                    if figure == compare0 and adjacent == compare1:
                        #print('Brisanje stage 2')
                        #board_with_ai.GLOBAL_last_move = []
                        continue

                if adjacent in board: continue

                help2 = set()
                help2 = help_first_col.union({adjacent})
                
                first_col_list.append(help2)

                if not_in_mill and is_Mill(help2, adjacent):

                    for fig in not_in_mill:

                        in_mill_flag_list.append(True)

                    continue

                in_mill_flag_list.append(False)
        
        return len(in_mill_flag_list)


    else: # stage 3, similar to stage 2
        
     
        for figure in first_col:
           
            help_board = board.difference({figure})
            help_first_col = first_col.difference({figure})
            
            for move in state_space.keys(): # only difference between stage 2 and 3 is that in stage 3 we can move on any free field
                
                if depth == board_with_ai.GLOBAL_search_depth and board_with_ai.GLOBAL_last_move:
                    compare0 = board_with_ai.GLOBAL_last_move[0][0]
                    compare1 = board_with_ai.GLOBAL_last_move[0][1]
                    if move == compare0 and compare1 == figure:
                        #print('Brisanje stage 3')
                        #board_with_ai.GLOBAL_last_move = []
                        continue

                if move in board: continue

                
                help2 = set()
                help2 = help_first_col.union({move})
                
                first_col_list.append(help2)

                if not_in_mill and is_Mill(help2, move):

                    for fig in not_in_mill:

                        in_mill_flag_list.append(True)

                    continue

                in_mill_flag_list.append(False)
        
       
        return len(in_mill_flag_list)














