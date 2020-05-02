from state_space_descriptor import state_space, position_on_board, mill_combinations

from heuristics import number_of_pieces_heuristic, first_stage_heuristics


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
        return None, first_stage_heuristics(black_pieces, white_pieces, max_player), None, None

    ##################### possible moves for every stage of the game ####################
    
    if number_of_pieces >= 0:
        '''
            FIRST STAGE
        '''
        
        # set of empty fields
        possible_moves = set(state_space.keys())
        possible_moves.difference_update(board)
        
        if max_player:
            
            current_evaluation_max = alpha
            
            flag1 = False
            for move in possible_moves:
                print('IN MAX')
                board_help = set()
                board_help = board.union({move})
                
                black_pieces_help = set()
                black_pieces_help = black_pieces.union({move})
                
                # trebas provjeriti je li doslo do mlina
                
                print('MAX MOVE: {}'.format(move))
                _ , from_min, _, _ = alpha_beta(board_help, depth-1, False, current_evaluation_max, beta,
                                                white_pieces, black_pieces_help, number_of_pieces-1)
                
                print(current_evaluation_max, from_min)
                
                #current_evaluation = max(current_evaluation, from_min)
                
                # provjera MAX
                if current_evaluation_max >= from_min:
                    current_evaluation_max = current_evaluation_max
                    current_move_max = move
                    #current_move = move
                else:
                    print('NEW MAX')
                    current_evaluation_max = from_min
                    max_move = move
                    flag1 = True
                
                if flag1: 
                    print('in flag')
                    current_move_max = max_move
                # kraj provjere max

                # alpha-beta pruning
                if current_evaluation_max >= beta: 
                    print('Beta pruning---------start')
                    print(current_move_max, current_evaluation_max, alpha, beta)
                    print('Beta pruning---------end')
                    return current_move_max, beta, None, None
                    
            print(current_move_max, current_evaluation_max, alpha, beta)
            return current_move_max, current_evaluation_max, None, None

        else: # if not max_player
            current_evaluation_min = beta
            
            flag2 = False
            for move in possible_moves:
                print('IN MIN')
                
                board_help = set()
                board_help = board.union({move})

                white_pieces_help = set()
                white_pieces_help = white_pieces.union({move})

                print('MOVE: {}'.format(move))

                # trebas provjeriti je li doslo do mlina

                _, from_max, _, _ = alpha_beta(board_help, depth-1, True, alpha, current_evaluation_min,
                                                white_pieces_help, black_pieces, number_of_pieces-1)
                print(current_evaluation_min, from_max)
                
                #current_evaluation = min(current_evaluation, from_max)

                # provjera MIN
                if current_evaluation_min <= from_max:
                    current_evaluation_min = current_evaluation_min
                    current_move_min = move
                else:
                    print('NEW MIN')
                    current_evaluation_min = from_max
                    min_move = move
                    flag2 = True
                # kraj provjere MIN
                if flag2: 
                    print('in min flag')
                    current_move_min = min_move
                
                # alpha-beta pruning
                if current_evaluation_min <= alpha: 
                    print('Alpha pruning---------start')
                    print(current_move_min, current_evaluation_min, alpha, beta)
                    print('Alpha pruning---------end')
                    return current_move_min, alpha, None, None
                
            print(current_move_min, current_evaluation_min, alpha, beta)
            return current_move_min, current_evaluation_min, None, None

    else:
        '''
            SECOND & THIRD STAGE
        '''
        print('second stage')
        pass



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
    
