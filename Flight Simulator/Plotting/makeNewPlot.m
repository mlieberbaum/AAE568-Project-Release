function makeNewPlot(figurePos, xLabelIn, yLabelIn, titleIn)

    figure;
    hold on
    grid on
    set(gcf, 'Color', 'w')
    set(gcf, 'Position', figurePos)
    
    if nargin > 1
        xlabel(xLabelIn, 'FontSize', 16);
        ylabel(yLabelIn, 'FontSize', 16);
        title(titleIn, 'FontSize', 16);
    end
    
    set(get(gca, 'XAxis'), 'FontSize', 12)
    set(get(gca, 'YAxis'), 'FontSize', 12)
    
end