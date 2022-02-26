function visualiseAFrame(matrix)
% matrix44 : homogeneous matrix

hold on

% origin
origin = [matrix(1, 4), matrix(2, 4), matrix(3, 4)];
% length of frame vectors
length = 0.1;
radius = 3.0;
xend = origin +length*matrix(1:3, 1)';
yend = origin + length*matrix(1:3, 2)';
zend = origin + length*matrix(1:3, 3)';
% x-axis
line('XData', [origin(1) xend(1)], 'YData', [origin(2) xend(2)],...
    'ZData', [origin(3) xend(3)], 'Color','r', 'LineWidth', radius);
% plot3([origin(1) xend(1)], [origin(2) xend(2)],...
%     [origin(3) xend(3)], 'Color','r', 'linewidth', radius);
% y-axis
line('XData', [origin(1) yend(1)], 'YData', [origin(2) yend(2)],...
    'ZData', [origin(3) yend(3)], 'Color','g', 'LineWidth', radius);
% z-axis
line('XData', [origin(1) zend(1)], 'YData', [origin(2) zend(2)],...
    'ZData', [origin(3) zend(3)], 'Color','b', 'LineWidth', radius);