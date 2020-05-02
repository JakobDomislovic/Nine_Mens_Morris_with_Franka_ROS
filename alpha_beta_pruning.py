from state_space_descriptor import state_space, position_on_board, mill_combinations

from heuristics import number_of_pieces_heuristic, first_stage_heuristics

import copy

'''
board - current state of board
depth - depth of search
max_player - True or False (in our case Max player is a Black player)
alpha, beta - initially +inf and -inf
white_pieces - set of currently placed white pieces
black_pieces - set of currently placed black pieces
full_board - all untaken places
number_of_pieces - when it reaches 0 we are entering in the second stage

alpha_beta algorithm returns: 
        move - move for player max
        evaluation_value
        figure to kill
        figure to move - if in 2nd or 3rd stage we first need to choose figure and then move that figure

'''

def alpha_beta(board, depth, max_player, alpha, beta, white_pieces, black_pieces, number_of_pieces):
    
    if is_Terminal(board, max_player, white_pieces, black_pieces):
        # utility
        if max_player: return None, 10000, None, None
        else: return None, -10000, None, None

    if depth == 0:
        return None, number_of_pieces_heuristic(black_pieces, white_pieces, max_player), None, None

    ##################### possible moves for every stage of the game ####################
    
    if number_of_pieces >= 0:
        '''
            FIRST STAGE
        '''
        if max_player:
            
            current_evaluation_max = alpha
            
            flag1 = False
            mill_flag_black = False
            
            for moves in possible_moves_list(board, black_pieces, white_pieces, max_player, 1):
                
                new_board, black, white, is_Mill_move = moves[0], moves[1], moves[2], moves[3]

                #print(new_board, black, white, is_Mill_move)

                _ , from_min, _, _ = alpha_beta(new_board, depth-1, False, current_evaluation_max, beta,
                                                white, black, number_of_pieces-1)
                
                #print(current_evaluation_max, from_min)
                
                move = new_board.difference(board)

                # -------------------------------------provjera MAX
                if current_evaluation_max >= from_min:
                    current_evaluation_max = current_evaluation_max
                    current_move_max = move
                else:
                    if is_Mill_move:
                        mill_move_black = white_pieces.symmetric_difference(white).pop()
                        mill_flag_black = True
                    current_evaluation_max = from_min
                    max_move = move
                    flag1 = True
                
                if flag1: 
                    current_move_max = max_move
                # -------------------------------------kraj provjere max

                # alpha-beta pruning
                if current_evaluation_max >= beta: 
                    #print('Beta pruning---------start')
                    #print(current_move_max, current_evaluation_max, alpha, beta)
                    #print('Beta pruning---------end')
                    return current_move_max, beta, None, None
                    
            if mill_flag_black: return current_move_max.pop(), current_evaluation_max, mill_move_black, None
            else: return current_move_max.pop(), current_evaluation_max, None, None

        else: # if not max_player

            current_evaluation_min = beta
            
            flag2 = False

            for moves in possible_moves_list(board, black_pieces, white_pieces, max_player, 1):
                
                new_board, white, black, is_Mill_move = moves[0], moves[1], moves[2], moves[3]
                
                _, from_max, _, _ = alpha_beta(new_board, depth-1, True, alpha, current_evaluation_min,
                                                white, black, number_of_pieces-1)

                #print(current_evaluation_min, from_max)
                
                move = new_board.difference(board)

                # -------------------------------------provjera MIN
                if current_evaluation_min <= from_max:
                    current_evaluation_min = current_evaluation_min
                    current_move_min = move
                else:
                    #print('NEW MIN')
                    current_evaluation_min = from_max
                    min_move = move
                    flag2 = True
                # -------------------------------------kraj provjere MIN
                if flag2: 
                    #print('in min flag')
                    current_move_min = min_move
                
                # alpha-beta pruning
                if current_evaluation_min <= alpha: 
                    #print('Alpha pruning---------start')
                    #print(current_move_min, current_evaluation_min, alpha, beta)
                    #print('Alpha pruning---------end')
                    return current_move_min, alpha, None, None
                
            #print(current_move_min, current_evaluation_min, alpha, beta)
            
            else: return current_move_min.pop(), current_evaluation_min, None, None

    else:
        '''
            SECOND & THIRD STAGE
        '''
        print('second stage')
        pass



def possible_moves_list(board_x, black_x, white_x, max_player, stage):
    
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
        for m in mill_combinations:
            help1 = set()
            for t in m: help1.add(t)
            for i in white_x:
                not_in_mill.add(i)
                if i in help1 and help1.issubset(white_x):
                    help4.add(i)

    else:
        first_col = copy.deepcopy(white_x)
        second_col = copy.deepcopy(black_x)
 
        initial_second = copy.deepcopy(black_x)
        help4 = set()
        for m in mill_combinations:
            help1 = set()
            for t in m: help1.add(t)
            for i in black_x:
                not_in_mill.add(i)
                if i in help1 and help1.issubset(black_x):
                    help4.add(i)
                    
    not_in_mill.difference_update(help4)
    
    for move in state_space.keys():
        
        if move in board: continue

        first_col_list.append(first_col.union({move}))
        
        if not_in_mill and is_Mill(first_col.union({move}), move):
            
            for fig in not_in_mill:
                
                in_mill_flag_list.append(True)
                
                help_set = set()
                help_set = board.union({move})
                help_set.difference_update({fig})
                board_list.append(help_set)

                second_col_list.append(second_col.difference({fig}))
                first_col_list.append(first_col_list[-1])
            
            first_col_list.pop() # jednog viska si dodao jer prije if is_Mill
            continue

        board_list.append(board.union({move}))
        second_col_list.append(initial_second)
        in_mill_flag_list.append(False)

    return zip(board_list, first_col_list, second_col_list, in_mill_flag_list)


def is_Mill(color, move):
        
    for mill in mill_combinations:
        help2 = set()
        
        for t in mill: help2.add(t)
        
        if move in help2 and help2.issubset(color):
            return True

    return False 


def is_Terminal(board, player, white, black):
    # checks whether we reached the terminal state
    if player:
        if len(white) < 3 and len(board) >= 5:  # minimalno na ploci moze biti 5 igraca
            return True
        else: 
            return False
        for k in white:
            for i in state_space[k]:
                if i not in board:
                    return False
        return True

    else:
        if len(black) < 3 and len(board) >= 5: # minimalno na ploci moze biti 5 igraca
            return True
        else:
            return False
        for k in black:
            for i in state_space[k]:
                if i not in board:
                    return False
        return True