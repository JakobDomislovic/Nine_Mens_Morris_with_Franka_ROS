from state_space_descriptor import state_space, position_on_board, mill_combinations

from heuristics import number_of_pieces_heuristic, first_stage_heuristics

import copy


def is_Mill(color, move):
    
    for mill in mill_combinations:
        help2 = set()
        
        for t in mill: help2.add(t)
        
        if move in help2 and help2.issubset(color):
            return True

    return False 


def is_Terminal(board, player, white, black):

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

    # kasnije da si skratis kod, zasad sve razdvoji za debugiranje

    #if number_of_pieces >= 0: 
    #    possible_config = possible_moves_list(board, black_pieces, white_pieces, max_player, 1)

    #elif len(black_pieces) > 3 and len(white_pieces) > 3:
    #    possible_config = possible_moves_list(board, black_pieces, white_pieces, max_player, 2)
    #
    #else:
    #    possible_config = possible_moves_list(board, black_pieces, white_pieces, max_player, 3)


    if number_of_pieces > 0:
        '''
            FIRST STAGE
        '''
        if max_player:
            
            current_evaluation_max = alpha
            
            mill_flag_black = False
            
            flag1 = False
            
            for moves in possible_moves_list(board, black_pieces, white_pieces, max_player, 1):
                
                
                new_board, black, white, is_Mill_move = moves[0], moves[1], moves[2], moves[3]

                _ , from_min, _, _ = alpha_beta(new_board, depth-1, False, current_evaluation_max, beta,
                                                white, black, number_of_pieces-1)
                
                move = new_board.difference(board)

                # -------------------------------------provjera MAX
                if current_evaluation_max >= from_min:
                    current_evaluation_max = current_evaluation_max
                    current_move_max = move
                    flag1 = False

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

                # beta pruning
                if current_evaluation_max >= beta: return current_move_max, beta, None, None
                    
            if mill_flag_black: return current_move_max.pop(), current_evaluation_max, mill_move_black, None
            else: return current_move_max.pop(), current_evaluation_max, None, None

        else: # if not max_player

            current_evaluation_min = beta
            
            flag2 = False
            
            for moves in possible_moves_list(board, black_pieces, white_pieces, max_player, 1):
                
                new_board, white, black, is_Mill_move = moves[0], moves[1], moves[2], moves[3]
                
                _, from_max, _, _ = alpha_beta(new_board, depth-1, True, alpha, current_evaluation_min,
                                                white, black, number_of_pieces-1)
  
                move = new_board.difference(board)

                # -------------------------------------provjera MIN
                if current_evaluation_min <= from_max:
                    current_evaluation_min = current_evaluation_min
                    current_move_min = move

                else:
                    current_evaluation_min = from_max
                    min_move = move
                    flag2 = True

                if flag2: 
                    current_move_min = min_move
                # -------------------------------------kraj provjere MIN
                
                # alpha pruning
                if current_evaluation_min <= alpha: return current_move_min, alpha, None, None
                
            else: return current_move_min.pop(), current_evaluation_min, None, None

    elif len(black_pieces) > 3 and len(white_pieces) > 3:
        
        '''
            SECOND STAGE
        '''

        if max_player:
            
            current_evaluation_max = alpha
            
            mill_flag_black = False
            
            flag1 = False
            
            for moves in possible_moves_list(board, black_pieces, white_pieces, max_player, 2):
                
                new_board, black, white, is_Mill_move, move = moves[0], moves[1], moves[2], moves[3], moves[4]

                _ , from_min, _, _ = alpha_beta(new_board, depth-1, False, current_evaluation_max, beta,
                                                white, black, number_of_pieces)
                
                #move = new_board.difference(board)

                # -------------------------------------provjera MAX
                if current_evaluation_max >= from_min:
                    current_evaluation_max = current_evaluation_max
                    current_move_max = move
                    new_place_4_fig = new_board.difference(board)
                    flag1 = False

                else:
                    if is_Mill_move:
                        mill_move_black = white_pieces.symmetric_difference(white).pop()
                        mill_flag_black = True
                    current_evaluation_max = from_min
                    new_place_4_fig = new_board.difference(board)
                    max_move = move
                    flag1 = True
                
                if flag1:
                    current_new_place = new_place_4_fig.pop()
                    current_move_max = max_move
                # -------------------------------------kraj provjere max

                # beta pruning
                if current_evaluation_max >= beta: 
                    return current_move_max, beta, None, None
            
            print(current_move_max, new_place_4_fig)
            if mill_flag_black: return current_move_max, current_evaluation_max, mill_move_black, current_new_place
            else: return current_move_max, current_evaluation_max, None, current_new_place

        else: # if not max_player

            current_evaluation_min = beta
            
            flag2 = False

            for moves in possible_moves_list(board, black_pieces, white_pieces, max_player, 2):
                
                new_board, black, white, is_Mill_move, move = moves[0], moves[1], moves[2], moves[3], moves[4]
                
                _, from_max, _, _ = alpha_beta(new_board, depth-1, True, alpha, current_evaluation_min,
                                                white, black, number_of_pieces)
  
                #move = new_board.difference(board)

                # -------------------------------------provjera MIN
                if current_evaluation_min <= from_max:
                    current_evaluation_min = current_evaluation_min
                    current_move_min = move
                    new_place_4_fig = new_board.difference(board)
                    flag2 = False

                else:
                    current_evaluation_min = from_max
                    min_move = move
                    new_place_4_fig = new_board.difference(board)
                    flag2 = True

                if flag2:
                    current_move_min = min_move
                # -------------------------------------kraj provjere MIN
                
                # alpha pruning
                if current_evaluation_min <= alpha: return current_move_min, alpha, None, None
                
            else: return current_move_min, current_evaluation_min, None, None
        

    else:
        '''
            THIRD STAGE
        '''

        if max_player:
            
            current_evaluation_max = alpha
            
            mill_flag_black = False
            
            flag1 = False
            
            for moves in possible_moves_list(board, black_pieces, white_pieces, max_player, 2):
                
                new_board, black, white, is_Mill_move, move = moves[0], moves[1], moves[2], moves[3], moves[4]

                _ , from_min, _, _ = alpha_beta(new_board, depth-1, False, current_evaluation_max, beta,
                                                white, black, number_of_pieces)
                
                #move = new_board.difference(board)

                # -------------------------------------provjera MAX
                if current_evaluation_max >= from_min:
                    current_evaluation_max = current_evaluation_max
                    current_move_max = move

                else:
                    if is_Mill_move:
                        mill_move_black = white_pieces.symmetric_difference(white).pop()
                        mill_flag_black = True
                    current_evaluation_max = from_min
                    new_place_4_fig = new_board.difference(board)
                    max_move = move
                    flag1 = True
                
                if flag1: 
                    current_move_max = max_move
                # -------------------------------------kraj provjere max

                # beta pruning
                if current_evaluation_max >= beta: 
                    return current_move_max, beta, None, None
            
            print(current_move_max, new_place_4_fig)
            if mill_flag_black: return current_move_max, current_evaluation_max, mill_move_black, new_place_4_fig.pop()
            else: return current_move_max, current_evaluation_max, None, new_place_4_fig.pop()

        else: # if not max_player

            current_evaluation_min = beta
            
            flag2 = False

            for moves in possible_moves_list(board, black_pieces, white_pieces, max_player, 2):
                
                new_board, black, white, is_Mill_move, move = moves[0], moves[1], moves[2], moves[3], moves[4]
                
                _, from_max, _, _ = alpha_beta(new_board, depth-1, True, alpha, current_evaluation_min,
                                                white, black, number_of_pieces)
  
                #move = new_board.difference(board)

                # -------------------------------------provjera MIN
                if current_evaluation_min <= from_max:
                    current_evaluation_min = current_evaluation_min
                    current_move_min = move

                else:
                    current_evaluation_min = from_max
                    min_move = move
                    new_place_4_fig = new_board.difference(board)
                    flag2 = True

                if flag2: 
                    current_move_min = min_move
                # -------------------------------------kraj provjere MIN
                
                # alpha pruning
                if current_evaluation_min <= alpha: return current_move_min, alpha, None, None
                
            else: return current_move_min, current_evaluation_min, None, None



def possible_moves_list(board_x, black_x, white_x, max_player, stage):
    
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
    
    if stage == 1:
        
        for move in state_space.keys():

            if move in board: continue

            help2 = set()
            help2 = first_col.union({move})

            first_col_list.append(help2)

            if not_in_mill and is_Mill(help2, move):

                for fig in not_in_mill:

                    in_mill_flag_list.append(True)

                    help_set = set()
                    help_set = board.union({move})
                    help_set.difference_update({fig})
                    board_list.append(help_set)

                    second_col_list.append(second_col.difference({fig}))
                    first_col_list.append(first_col_list[-1])

                first_col_list.pop() # jednog viska si dodao jer prije if is_Mill radil dodavanje
                continue

            board_list.append(board.union({move}))
            second_col_list.append(initial_second)
            in_mill_flag_list.append(False)

        return zip(board_list, first_col_list, second_col_list, in_mill_flag_list)

    
    elif stage == 2:
        
        move_figure = []
        new_place = []

        for figure in first_col:
           
            help_board = board.difference({figure})
            help_first_col = first_col.difference({figure})
            
            for adjacent in state_space[figure]:
                
                if adjacent in board: continue

                move_figure.append(figure)
                new_place.append(adjacent)

                help2 = set()
                help2 = help_first_col.union({adjacent})
                
                first_col_list.append(help2)

                if not_in_mill and is_Mill(help2, adjacent):

                    for fig in not_in_mill:

                        in_mill_flag_list.append(True)

                        help_set = set()
                        help_set = help_board.union({adjacent})
                        help_set.difference_update({fig})
                        board_list.append(help_set)

                        second_col_list.append(second_col.difference({fig}))
                        first_col_list.append(first_col_list[-1])
                        move_figure.append(move_figure[-1])
                        new_place.append(new_place[-1])

                    first_col_list.pop()
                    move_figure.pop()
                    new_place.pop()
                    continue

                board_list.append(help_board.union({adjacent}))
                second_col_list.append(initial_second)
                in_mill_flag_list.append(False)

        return zip(board_list, first_col_list, second_col_list, in_mill_flag_list, move_figure, new_place)


    else: # stage 3, similar to stage 2
        
        move_figure = []
        new_place = []

        for figure in first_col:
           
            help_board = board.difference({figure})
            help_first_col = first_col.difference({figure})
            
            for move in state_space.keys(): # only difference between stage 2 and 3 is that in stage 3 we can move on any free field
                
                if move in board: continue

                move_figure.append(figure)
                new_place.append(move)

                help2 = set()
                help2 = help_first_col.union({move})
                
                first_col_list.append(help2)

                if not_in_mill and is_Mill(help2, move):

                    for fig in not_in_mill:

                        in_mill_flag_list.append(True)

                        help_set = set()
                        help_set = help_board.union({move})
                        help_set.difference_update({fig})
                        board_list.append(help_set)

                        second_col_list.append(second_col.difference({fig}))
                        first_col_list.append(first_col_list[-1])
                        move_figure.append(move_figure[-1])
                        new_place.append(new_place[-1])

                    first_col_list.pop()
                    move_figure.pop()
                    new_place.pop()
                    continue

                board_list.append(help_board.union({move}))
                second_col_list.append(initial_second)
                in_mill_flag_list.append(False)

        return zip(board_list, first_col_list, second_col_list, in_mill_flag_list, move_figure, new_place)


# testiranja
#for i in possible_moves_list({'a1', 'a4', 'd2','g7', 'g4', 'd1'}, {'g7', 'g4', 'd1'}, {'a1', 'a4', 'd2'}, True, 3):
#    print('\nboard: {}'.format(i[0]))
#    print('black: {}'.format(i[1]))
#    print('white: {}'.format(i[2]))
#    print('in mill: {}'.format(i[3]))
#    print('move figure: {}'.format(i[4]))
#    print('new place: {}\n'.format(i[5]))





