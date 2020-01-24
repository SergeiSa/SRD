function LineWidth = SRDgraphic_FixLineWidth(InputLineWidth)
LineWidth = InputLineWidth;
if LineWidth < 0.5
    LineWidth = 0.5;
end
end