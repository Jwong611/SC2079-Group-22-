from Commons.Types import Position

def euclidean(
    start: Position,
    end: Position
) ->  float:
    return ((start.y - end.y)**2 + (start.x - end.x)**2)**.5