#from alpha_beta_pruning import possible_moves_list
from state_space_descriptor import state_space

def number_of_pieces_heuristic(black, white, number_of_pieces):
    '''
    comparing number of pieces
    '''
    if number_of_pieces:
        # stage 1
        return (len(black) - len(white)) * 100
    else:
        # if stage 2 or 3 value mill more
        return (len(black) - len(white)) * 300


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


def try_2_block_pieces(black, white, number_of_pieces):
    '''
    heuristic is leading AI to block all enemy figures, while also trying to form mill
    '''

    evaluation = 0

    white_blocked, black_blocked = blocked_by_piece(black, white)
    length_black = len(black)
    length_white = len(white)

    if number_of_pieces:
        evaluation += (black_blocked) * 50
        evaluation += (length_black - length_white) * 25 

    else:
        # u drugoj i trecoj fazi vise cijeni mlinove nego u prvoj
        evaluation += (black_blocked) * 60
        evaluation += (length_black - length_white) * 50 

    return evaluation



def number_of_blocked_pieces(black, white, number_of_pieces):
    '''
    looks for number of blocked pieces and at number of pieces (if you can make mill and block somebody why not...)
    '''

    evaluation = 0

    white_blocked, black_blocked = blocked_by_piece(black, white)
    length_black = len(black)
    length_white = len(white)

    if number_of_pieces:
        evaluation += (black_blocked - white_blocked) * (-50)
        evaluation += (length_black - length_white) * 25 

    else:
        # u drugoj i trecoj fazi vise cijeni mlinove nego u prvoj
        evaluation += (black_blocked - white_blocked) * (-75)
        evaluation += (length_black - length_white) * 75

    return evaluation


def number_of_double_mill_formations(black, white, number_of_pieces):



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


def blocked(black, white):
    
    # pomocna fja koja vraca koliko figura odredena boja blokira

    number_of_blocked_black = 0
    number_of_blocked_white = 0

    for wh in white:
        t = True
        for spot in state_space[wh]:
            if spot not in black:
                t = False
                break
        if t: number_of_blocked_black += 1

    for bl in black:
        t = True
        for spot in state_space[bl]:
            if spot not in white:
                t = False
                break
        if t: number_of_blocked_white += 1

    return number_of_blocked_white, number_of_blocked_black









