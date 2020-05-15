from state_space_descriptor import state_space, super_move_formations, mill_combinations
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
        evaluation += (length_black - length_white) * 10
        evaluation += (black_two_piece - white_two_piece) * 10
        evaluation += (black_three_piece - white_three_piece) * 7

    elif not player and length_black > 3:
        # u drugoj i trecoj fazi vise cijeni mlinove nego u prvoj
        if mill_move_flag: evaluation += 100
        evaluation += (length_black - length_white) * 15
        evaluation += (black_three_piece - white_three_piece) * 5
        evaluation += (black_super_mill - white_super_mill) * 10
    
    elif player and length_white > 3:
        # u drugoj i trecoj fazi vise cijeni mlinove nego u prvoj
        if mill_move_flag: evaluation += -100
        evaluation += (length_black - length_white) * 15
        evaluation += (black_two_piece - white_two_piece) * 5
        evaluation += (black_three_piece - white_three_piece) * 5
        evaluation += (black_super_mill - white_super_mill) * 10
    
    else:
        if player:
            if mill_move_flag: evaluation += -100
        else:
            evaluation += 100
        evaluation += (length_black - length_white) * 10
        evaluation += (black_two_piece - white_two_piece) * 50
        evaluation += (black_three_piece - white_three_piece) * 50   
    
    if depth == board_with_ai.GLOBAL_search_depth - 1:
        if mill_move_flag:
            if player:
                evaluation += -1000
            else:
                evaluation += 1000

    return evaluation


def advanced_heuristic(black, white, number_of_pieces, mill_move_flag, player, depth):
    evaluation = 0

    length_black = len(black)
    length_white = len(white)
    length_diff = length_black - length_white

    white_num_morris, black_num_morris = number_of_morrises(black, white)
    morrises_diff = black_num_morris - white_num_morris

    white_no_adj, black_no_adj = blocked_by_piece(black, white)
    no_adj_diff = black_no_adj - white_no_adj # pazi ovdje oduzimas protivnik - igrac jer ti je cilj da protivnik ima sto manje susjednih mjesta slobodno

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
        evaluation += no_adj_diff * 2
        evaluation += length_diff * 10
        evaluation += two_piece_diff * 15
        evaluation += three_piece_diff * 7

    elif not player and length_black > 3:
        # u drugoj i trecoj fazi vise cijeni mlinove nego u prvoj
        if mill_move_flag: evaluation += 30
        evaluation += morrises_diff * 40
        evaluation += no_adj_diff * 20
        evaluation += length_diff * 15
        evaluation += super_mill_diff * 30
    
    elif player and length_white > 3:
        # u drugoj i trecoj fazi vise cijeni mlinove nego u prvoj
        if mill_move_flag: evaluation += -30
        evaluation += morrises_diff * 40
        evaluation += no_adj_diff * 20
        evaluation += length_diff * 15
        evaluation += super_mill_diff * 30

    else:
        if player:
            if mill_move_flag: evaluation += -50
        else:
            evaluation += 50
        evaluation += two_piece_diff * 10
        evaluation += three_piece_diff * 2
        
        
    if depth == board_with_ai.GLOBAL_search_depth - 1:
        if mill_move_flag:
            if player:
                evaluation += -1000
            else:
                evaluation += 1000

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

















