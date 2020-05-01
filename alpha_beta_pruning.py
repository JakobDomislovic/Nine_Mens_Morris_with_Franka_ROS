from state_space_descriptor import state_space, position_on_board, mill_combinations

from heuristics import number_of_pieces_heuristic


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
        return None, number_of_pieces_heuristic(black_pieces, white_pieces), None, None

    ##################### possible moves for every stage of the game ####################
    
    if number_of_pieces >= 0:
        '''
            FIRST STAGE
        '''
        print('first stage')
        
        # set of empty fields
        possible_moves = set(state_space.keys())
        possible_moves.difference_update(board) 
        
        if max_player:

            current_evaluation = alpha

            for move in possible_moves:
                
                board_help = set()
                board_help = board.union({move})
                
                black_pieces_help = set()
                black_pieces_help = black_pieces.union({move})
                
                # trebas provjeriti je li doslo do mlina
                # ako je doslo do mlina moras vratiti koju figuru ubiti 

                _ , from_min, _, _ = alpha_beta(board_help, depth-1, False, current_evaluation, beta,
                                                white_pieces, black_pieces_help, number_of_pieces-1)
                
                current_evaluation = max(current_evaluation, from_min)
                
                # alpha-beta pruning
                if current_evaluation >= beta: return move, beta, None, None
            
            return move, current_evaluation, None, None

        else: # if not max_player

            current_evaluation = beta

            for move in possible_moves:
                
                board_help = set()
                board_help = board.union({move})

                white_pieces_help = set()
                white_pieces_help = white_pieces.union({move})

                # trebas provjeriti je li doslo do mlina

                _, from_max, _, _ = alpha_beta(board_help, depth-1, True, alpha, current_evaluation,
                                                white_pieces_help, black_pieces, number_of_pieces-1)
                
                current_evaluation = min(current_evaluation, from_max)
               
                # alpha-beta pruning
                if current_evaluation <= alpha: return move, alpha, None, None
            
            return move, current_evaluation, None, None

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
    
