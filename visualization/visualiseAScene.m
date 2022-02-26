figure("Name","Visualization of Task-space Tracking with Minimum Discontinuities",...
    "Units","normalized",...
    "OuterPosition",[0, 0, 1, 1],...
    "Visible","on");
show(ur5,currentRobotJConfig,'PreservePlot',false,'Frames','off');
hold on
for i = 1:length(env)
    show(env{i});
end
axis([-0.5 1 -0.75 0.75 -0.5 1.0]);
view(135, 10);
grid off

for i = 1:size(all_44waypoints, 3)
    visualiseAFrame(all_44waypoints(:, :, i));
end

i = 240;
show(ur5, vertices(i).gIK_(1, 2:7), 'PreservePlot',false,'Frames','off');
plot3(xf(1:i), yf(1:i), zf(1:i), '.', 'Color', [0.6, 0.6, 0.6], 'MarkerSize',20)

