data=load('mydata');
figure(1);
clf
fts = [];
for i = 1 : size(data,1)
  fts(end+1) = norm(data(i,1:3));
end
fts = -data(:,3);
%plot(data(:,1), 'r'); hold on;
%plot(data(:,2), 'g'); hold on;
%plot(data(:,3), 'b'); hold on;
plot((1:277)/277*14.6, fts(1224:1500), 'r', 'LineWidth', 3); hold on;
plot((1:277)/277*14.6, 200 * data((1224:1500),9), 'b', 'LineWidth', 3); hold on;
curs = [];
for i = 1 : size(data,1)
  curs(end+1) = 10 * norm(data(i,[10,13,15]));
  %curs(end+1) = 10 * norm(data(i,10:16));
  if(i < 1200)
    curs(end) = 0;
  end
end
curs = curs(1224:1500);
plot((1:277)/277*14.6, curs, 'g', 'LineWidth', 3); hold on;
title('End-Effector Force/Position and Mean Joint Current for Small Lever Task', 'FontSize', 18);
set(gcf,'color','w');
xlabel('Time (secs)', 'FontSize', 15);
ylabel('Sensor reading ()', 'FontSize', 15);
legend('Vertical force reading (Nm)', 'End-effector Vertical Position (cm)', 'Average Current (dA)', 'Location', 'NorthWest');
%plot(data(:,10), 'k'); hold on;
%plot(data(:,11), 'k'); hold on;
%plot(data(:,12), 'k'); hold on;
%plot(data(:,13), 'k'); hold on;
%plot(data(:,14), 'k'); hold on;
%plot(data(:,15), 'k'); hold on;
%plot(data(:,16), 'k'); hold on;