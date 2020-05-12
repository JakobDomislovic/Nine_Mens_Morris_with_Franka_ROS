#from alpha_beta_pruning import possible_moves_list
from state_space_descriptor import state_space, super_move_formations
import board_with_ai # global variable GLOBAL_search_depth


def number_of_pieces_heuristic(black, white, number_of_pieces, mill_move_flag, player, depth):
    '''
    comparing number of pieces
    '''
    evaluation = 0
    
    if number_of_pieces:
        # stage 1
        evaluation += (len(black) - len(white)) * 100
    
    else:
        # if stage 2 or 3 value mill more

        evaluation += (len(black) - len(white)) * 300
        
        if mill_move_flag:
                #print('Mill move flag TRUE')
            if player: 
                evaluation += -10000
                if depth == board_with_ai.GLOBAL_search_depth - 1: 
                    evaluation += -1000
            else: 
                evaluation += 10000
                if depth == board_with_ai.GLOBAL_search_depth - 1: 
                    evaluation += 1000
        
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
        if not player: evaluation += (black_blocking) * 50
        else: evaluation -= (white_blocking) * 50
        evaluation += (length_black - length_white) * 25 

    elif not player and length_black > 3:
        # u drugoj i trecoj fazi vise cijeni mlinove nego u prvoj
        evaluation += (black_blocking) * 30
        evaluation += (length_black - length_white) * 50
    
    elif player and length_white > 3:
        evaluation -= (white_blocking) * 30
        evaluation += (length_black - length_white) * 50
    
    else:
        evaluation += (length_black - length_white) * 1000
        if depth == board_with_ai.GLOBAL_search_depth - 1: 
            if player: evaluation += -1000
            else: evaluation += 1000 

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


def number_of_double_mill(black, white, number_of_pieces, mill_move_flag, player):
    '''
    difference between black double mill and white double mill
    '''

    evaluation = 0

    length_black = len(black)
    length_white = len(white)

    black_double_mills, white_double_mills = super_move_formation(black, white) 

    if number_of_pieces:
        evaluation += (black_double_mills - white_double_mills) * 50
        evaluation += (length_black - length_white) * 5
    
    else:
        evaluation += (black_double_mills - white_double_mills) * 50
        if black_double_mills or white_double_mills:
            evaluation += (length_black - length_white) * 500
        else:
            evaluation += (length_black - length_white) * 50
        if mill_move_flag: 
            if player: evaluation -= 1000
            else: evaluation += 1000
    return evaluation


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

    return count_super_black, count_super_white



