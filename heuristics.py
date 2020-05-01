

def number_of_pieces_heuristic(black, white):
    return (len(black) - len(white)) * 100
    #if max_player:
    #    return len(color) * (10)
    #else:
    #    return len(color) * (-10)



# napravi posebnu heuristiku samo za prvu fazu igre