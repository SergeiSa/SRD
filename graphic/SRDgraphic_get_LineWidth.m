function LineWidth = SRDgraphic_get_LineWidth(Index)

NumberOfColors = 10;
LocalIndex = rem(Index, NumberOfColors);

switch LocalIndex
    case 0
        LineWidth = 2;
    case 1
        LineWidth = 1.25;
    case 2
        LineWidth = 3;
    case 3
        LineWidth = 2.5;
    case 4
        LineWidth = 2.5;
    case 5
        LineWidth = 1;
    case 6
        LineWidth = 3;
    case 7
        LineWidth = 2.5;
    case 8
        LineWidth = 1.5;
    case 9
        LineWidth = 2.5;
    otherwise
        LineWidth = 3;
end

end