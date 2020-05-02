

def number_of_pieces_heuristic(black, white, max_player):
    print('in heuristics')
    #return (len(black) - len(white)) * 100
    if max_player:
        return len(black) * (-10)
    else:
        return len(white) * (10)



# Strategically placing the pieces is more important than closing a mill in the first stage
def first_stage_heuristics(black, white, max_player):
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