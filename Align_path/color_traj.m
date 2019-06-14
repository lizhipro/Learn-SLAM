function color_traj(traj_Est, traj_Real, error)
assert(length(traj_Est)==length(traj_Real) && length(traj_Est)==length(error));
num = 100;

e_max=max(error);
jet_color=colormap(jet(num));
selected_color = zeros(length(error), 3);
for i=1:length(error)
	color_index=ceil((error(i)/e_max)*(num-1)+1);
	selected_color(i,:)=jet_color(color_index,:);
end

for i=1:length(traj_Est)
    plot3(traj_Est(1,i),traj_Est(2,i),traj_Est(3,i),'.','color',selected_color(i,:));
    hold on;
end
hold on;
plot3(traj_Real(1,:),traj_Real(2,:),traj_Real(3,:),'--k');
grid on;

bar_max = roundn(e_max, -2);
bar = {0,bar_max/10,bar_max/10*2,bar_max/10*3,bar_max/10*4,bar_max/10*5,bar_max/10*6,bar_max/10*7,bar_max/10*8,bar_max/10*9,bar_max};
colorbar('YTickLabel',bar,'location','eastoutside');
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
end