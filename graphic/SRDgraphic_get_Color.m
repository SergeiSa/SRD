function Color = SRDgraphic_get_Color(Index)

NumberOfColors = 10;
LocalIndex = rem(Index, NumberOfColors);

switch LocalIndex
    case 0
        Color = [0.5 0 0];
    case 1
        Color = [1 0 0];
    case 2
        Color = [0 0 1];
    case 3
        Color = [0.05 0.6 0.1];
    case 4
        Color = [0.3 0.3 0.3];
    case 5
        Color = [0 0 0];
    case 6
        Color = [0.3 0.3 0];
    case 7
        Color = [0 0.3 0.3];
    case 8
        Color = [0.4 0 0.3];
    case 9
        Color = [0.75 0.1 0.4];
    otherwise
        Color = [0 0 0];
end

end