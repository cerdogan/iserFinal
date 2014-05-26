load fts;
load fts2;
clf;
xx = []; for i = 1 : 600, xx(end+1) = norm(fts(i,:)); end;
xx2 = []; for i = 4500 : size(fts2,1) - 430, xx2(end+1) = norm(fts2(i,:)); end;
plot(-xx, 'r', 'LineWidth', 12); hold on;
plot(-xx2, 'g', 'LineWidth', 12);
axis([0 600 -400 50]); 
set(gca,'FontSize',35);
