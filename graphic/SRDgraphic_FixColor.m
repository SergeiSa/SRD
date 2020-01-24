function Color = SRDgraphic_FixColor(InputColor)
Color = InputColor;
for i = 1:3
    if Color(i) > 1
        Color(i) = 1;
    end
    if Color(i) < 0
        Color(i) = 0;
    end
end