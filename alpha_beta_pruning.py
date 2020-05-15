from state_space_descriptor import state_space, position_on_board, mill_combinations

from heuristics import number_of_pieces_heuristic # testing
from heuristics import first_stage_heuristics
from heuristics import number_of_blocked_pieces
from heuristics import try_2_block_pieces # testing
from heuristics import try_different_mills #testing
from heuristics import advanced_heuristic #testing
from heuristics import branch_factor_heuristic 

import copy
import board_with_ai


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

def alpha_beta(board, depth, max_player, alpha, beta, white_pieces, black_pieces, number_of_pieces, mill_move_flag):
    
    if number_of_pieces < 1: # ciljno stanje moze biti samo u drugoj ili trecoj fazi
        
        if is_Terminal(board, max_player, white_pieces, black_pieces):
            #print('terminal state, player on move: {}'.format(max_player))
            # utility
            if board_with_ai.GLOBAL_heur_choice == 1:
                if max_player: return None, -100000+number_of_pieces_heuristic(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None
                else: return None, 100000+number_of_pieces_heuristic(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None
        
            elif board_with_ai.GLOBAL_heur_choice == 2:
                if max_player: return None, -100000+try_2_block_pieces(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None
                else: return None, 100000+try_2_block_pieces(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None
        
            elif board_with_ai.GLOBAL_heur_choice == 3:
                if max_player: return None, -100000+try_different_mills(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None
                else: return None, 100000+try_different_mills(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None
            
            elif board_with_ai.GLOBAL_heur_choice == 4:
                if max_player: return None, -100000+advanced_heuristic(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None
                else: return None, 100000+advanced_heuristic(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None
            
            elif board_with_ai.GLOBAL_heur_choice == 5:
                if max_player: return None, -100000+branch_factor_heuristic(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None
                else: return None, 100000+branch_factor_heuristic(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None
            

    if depth == 0:
        if board_with_ai.GLOBAL_heur_choice == 1: return None, number_of_pieces_heuristic(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None
        
        elif board_with_ai.GLOBAL_heur_choice == 2: return None, try_2_block_pieces(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None
        
        elif board_with_ai.GLOBAL_heur_choice == 3: return None, try_different_mills(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None
        
        elif board_with_ai.GLOBAL_heur_choice == 4: return None, advanced_heuristic(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None

        elif board_with_ai.GLOBAL_heur_choice == 5: return None, branch_factor_heuristic(black_pieces, white_pieces, number_of_pieces, mill_move_flag, max_player, depth), None, None
    
    ##################### possible moves for every stage of the game ####################

    if number_of_pieces > 0:
        
        #    -----------------------------------------FIRST STAGE-----------------------------------------
        
        if max_player:
            
            current_evaluation_max = alpha
            
            mill_flag_black = False
            flag1 = False
           
            for moves in possible_moves_list(board, black_pieces, white_pieces, max_player, 1, depth):
                
                new_board, black, white, is_Mill_move = moves[0], moves[1], moves[2], moves[3]
                
                #print('First stage black: {}, {}, {}, {}'.format(new_board, black, white, is_Mill_move))

                _ , from_min, _, _ = alpha_beta(new_board, depth-1, False, current_evaluation_max, beta,
                                                white, black, number_of_pieces-1, is_Mill_move)
                
                
                #print(new_board)
                #print(white)
                #print(black)
                #print(len(white), len(black))
                #print(current_evaluation_max, from_min)
                # kod max cvora moras pamtiti najbolju cijenu i potez ide uz tu cijenu

                # -------------------------------------provjera MAX
                if current_evaluation_max >= from_min:
                    current_evaluation_max = current_evaluation_max

                else:
                    move = black.difference(black_pieces)
                    #if depth == 5: print('new move: {}'.format(black, black_pieces))
                    current_evaluation_max = from_min
                    current_move_max = move.pop()
                    flag1 = True
                    
                    if is_Mill_move:
                        if depth == 5: print('mill move: {}'.format(black, white))
                        mill_move_black = white_pieces.symmetric_difference(white).pop()
                        mill_flag_black = True
                    else:
                        mill_flag_black = False
                # -------------------------------------kraj provjere max
                
                # beta pruning
                if current_evaluation_max >= beta: return None, beta, None, None
            

            if flag1:
                #print('Stavi AI na poziciju: {}'.format(current_move_max))
                #print('MINIMAX vrijednost: {}'.format(current_evaluation_max))
                if mill_flag_black: return current_move_max, current_evaluation_max, mill_move_black, None
                else: return current_move_max, current_evaluation_max, None, None
            else:
                return None, current_evaluation_max, None, None

        else: # if not max_player

            current_evaluation_min = beta
            
            for moves in possible_moves_list(board, black_pieces, white_pieces, max_player, 1, depth):
                
                new_board, white, black, is_Mill_move = moves[0], moves[1], moves[2], moves[3]
                
                #print('First stage White: {}, {}, {}, {}'.format(new_board, black, white, is_Mill_move))
                
                _, from_max, _, _ = alpha_beta(new_board, depth-1, True, alpha, current_evaluation_min,
                                                white, black, number_of_pieces-1, is_Mill_move)

                # print('IN MIN')
                # print(new_board)
                # print(white)
                # print(black)
                # print(len(white), len(black))
                # print(current_evaluation_min, from_max)
                # print('In min: {}, {}'.format(current_evaluation_min, from_max))

                current_evaluation_min = min(current_evaluation_min, from_max)
                
                # alpha pruning
                if current_evaluation_min <= alpha: return None, alpha, None, None
                
            return None, current_evaluation_min, None, None

    #-----------------------------------------SECOND STAGE-----------------------------------------

    elif len(black_pieces) > 3 and max_player:
        
        current_evaluation_max = alpha
            
        mill_flag_black = False
        flag2 = False
        
        for moves in possible_moves_list(board, black_pieces, white_pieces, max_player, 2, depth):
            
            new_board, black, white, is_Mill_move, move, new_place = moves[0], moves[1], moves[2], moves[3], moves[4], moves[5]
           
            _ , from_min, _, _ = alpha_beta(new_board, depth-1, False, current_evaluation_max, beta,
                                            white, black, number_of_pieces, is_Mill_move)
            
            #if depth == 3: print(from_min, current_evaluation_max, move, new_place, is_Mill_move, len(black), len(white))

            # -------------------------------------provjera MAX
            if current_evaluation_max >= from_min:
                current_evaluation_max = current_evaluation_max
            
            else:
                #if depth==5: print('novi max')
                current_evaluation_max = from_min
                current_move_max = move
                current_new_place = new_place
                flag2 = True
                #if depth==5: 
                #    print(current_evaluation_max, current_move_max, current_new_place, is_Mill_move)
                if is_Mill_move:
                    mill_move_black = white_pieces.symmetric_difference(white).pop()
                    #print(mill_move_black)
                    mill_flag_black = True
                else:
                    mill_flag_black = False
            # -------------------------------------kraj provjere max
            
            #print('old/new/value: {}, {}, {}, {}, {}'.format(move, new_place, from_min, len(black), len(white)))
            # beta pruning
            if current_evaluation_max >= beta: return None, beta, None, None

        if flag2:
            # print('Posljednji potez')
            # print(current_evaluation_max)
            # print(current_move_max)
            # print(current_new_place)
            if mill_flag_black: 
                return current_move_max, current_evaluation_max, mill_move_black, current_new_place
            else: return current_move_max, current_evaluation_max, None, current_new_place
        else:
            return None, current_evaluation_max, None, None


    elif len(white_pieces) > 3 and not max_player:
        
        current_evaluation_min = beta
            
        for moves in possible_moves_list(board, black_pieces, white_pieces, max_player, 2, depth):
            
            new_board, white, black, is_Mill_move, move = moves[0], moves[1], moves[2], moves[3], moves[4]
            
            #print('Second stage white: {}, {}, {}, {}, {}'.format(new_board, black, white, is_Mill_move, move))
            
            _, from_max, _, _ = alpha_beta(new_board, depth-1, True, alpha, current_evaluation_min,
                                            white, black, number_of_pieces, is_Mill_move)


            current_evaluation_min = min(current_evaluation_min, from_max)
            
            # alpha pruning
            if current_evaluation_min <= alpha: return None, alpha, None, None
            
        return None, current_evaluation_min, None, None


    #-----------------------------------------THIRD STAGE-----------------------------------------

    elif len(black_pieces) == 3 and max_player:
        current_evaluation_max = alpha
            
        mill_flag_black = False
        flag3 = False
                    
        for moves in possible_moves_list(board, black_pieces, white_pieces, max_player, 3, depth):
            
            new_board, black, white, is_Mill_move, move, new_place = moves[0], moves[1], moves[2], moves[3], moves[4], moves[5]
            
            _ , from_min, _, _ = alpha_beta(new_board, depth-1, False, current_evaluation_max, beta,
                                            white, black, number_of_pieces, is_Mill_move)
            
            # -------------------------------------provjera MAX
            if current_evaluation_max >= from_min:
                current_evaluation_max = current_evaluation_max
            
            else:
                current_evaluation_max = from_min
                current_move_max = move
                current_new_place = new_place
                flag3 = True
                #print('\n new max Third stage black: {}, {}, {}, {}, {}, {}, {}'.format(new_board, black, white, is_Mill_move, move, new_place, number_of_pieces))
                if is_Mill_move:
                    mill_move_black = white_pieces.symmetric_difference(white).pop()
                    mill_flag_black = True
                else:
                    mill_flag_black = False
            # -------------------------------------kraj provjere max

            # beta pruning
            if current_evaluation_max >= beta: return current_move_max, beta, None, None
        
        if flag3: 
            if mill_flag_black: return current_move_max, current_evaluation_max, mill_move_black, current_new_place
            else: return current_move_max, current_evaluation_max, None, current_new_place
        else:
            return None, current_evaluation_max, None, None
    

    elif len(white_pieces) == 3 and not max_player:
        
        current_evaluation_min = beta
        
        for moves in possible_moves_list(board, black_pieces, white_pieces, max_player, 3, depth):
            
            new_board, white, black, is_Mill_move, move = moves[0], moves[1], moves[2], moves[3], moves[4]

            _, from_max, _, _ = alpha_beta(new_board, depth-1, True, alpha, current_evaluation_min,
                                            white, black, number_of_pieces, is_Mill_move)
            
            current_evaluation_min = min(current_evaluation_min, from_max)
            
            # alpha pruning
            if current_evaluation_min <= alpha: return None, alpha, None, None
            
        return None, current_evaluation_min, None, None




def possible_moves_list(board_x, black_x, white_x, max_player, stage, depth):
    
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

        #print(len(board_list), len(first_col_list), len(second_col_list), len(in_mill_flag_list))
        
        return zip(board_list, first_col_list, second_col_list, in_mill_flag_list)

    
    elif stage == 2:
        
        move_figure = []
        new_place = []

        for figure in first_col:
           
            help_board = board.difference({figure})
            help_first_col = first_col.difference({figure})
            
            for adjacent in state_space[figure]:

                if depth == board_with_ai.GLOBAL_search_depth and board_with_ai.GLOBAL_last_move:
                    compare0 = board_with_ai.GLOBAL_last_move[0][1] # figura koju trenutno promatras ista kao novi potez u proslom koraku
                    compare1 = board_with_ai.GLOBAL_last_move[0][0] # ako je susjedni (novo moguce polje) isti kao proslo mjesto s kojeg smo krenuli
                    #print(figure, compare0)
                    #print(adjacent, compare1)
                    if figure == compare0 and adjacent == compare1:
                        #print('Brisanje stage 2')
                        board_with_ai.GLOBAL_last_move = []
                        continue

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
        
        #print(len(board_list), len(first_col_list), len(second_col_list), len(in_mill_flag_list), len(move_figure), len(new_place))
        
        return zip(board_list, first_col_list, second_col_list, in_mill_flag_list, move_figure, new_place)


    else: # stage 3, similar to stage 2
        
        move_figure = []
        new_place = []

        for figure in first_col:
           
            help_board = board.difference({figure})
            help_first_col = first_col.difference({figure})
            
            for move in state_space.keys(): # only difference between stage 2 and 3 is that in stage 3 we can move on any free field
                
                if depth == board_with_ai.GLOBAL_search_depth and board_with_ai.GLOBAL_last_move:
                    compare0 = board_with_ai.GLOBAL_last_move[0][0]
                    compare1 = board_with_ai.GLOBAL_last_move[0][1]
                    if move == compare0 and compare1 == figure:
                        #print('Brisanje stage 3')
                        board_with_ai.GLOBAL_last_move = []
                        continue

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
        
        #print(len(board_list), len(first_col_list), len(second_col_list), len(in_mill_flag_list), len(move_figure), len(new_place))
      
        return zip(board_list, first_col_list, second_col_list, in_mill_flag_list, move_figure, new_place)


# testiranja
# for i in possible_moves_list({'f2', 'g4', 'f6', 'b4', 'c3', 'b6', 'b2', 'd6', 'e4', 'd2', 'c5'}, {'f6', 'b4', 'd6', 'b6', 'b2', 'c3', 'e4', 'd2', 'c5'}, {'f2', 'g4'},True, 2):
#     print('\nboard: {}'.format(i[0]))
#     print('black: {}'.format(i[1]))
#     print('white: {}'.format(i[2]))
#     print('in mill: {}'.format(i[3]))
#     print('move figure: {}'.format(i[4]))
#     print('new place: {}\n'.format(i[5]))





