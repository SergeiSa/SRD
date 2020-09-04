function LineStyle = SRDgraphic_get_LineStyle(Index)

NumberOfColors = 4;
LocalIndex = rem(Index, NumberOfColors);

switch LocalIndex
    case 0
        LineStyle = '-.';
    case 1
        LineStyle = '-';
    case 2
        LineStyle = '--';
    case 3
        LineStyle = ':';
    otherwise
        LineStyle = '-';
end

end