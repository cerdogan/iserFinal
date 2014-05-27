clf;
load exp1;
load exp2;
load exp3;
load exp4;
load exp5;
load exp6;
load exp7;
load exp8;
load exp9;
k = 1000;
lw = 7;
ms = 20;
subplot(2,1,1);
plot(exp5(1:k:end,1), exp5(1:k:end,3), "-xm", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot(exp4(1:k:end,1), exp4(1:k:end,3), "-xc", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot(exp3(1:k:end,1), exp3(1:k:end,3), "-xg", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot(exp2(1:k:(size(exp2,1)-k),1), -exp2(1:k:(size(exp2,1)-k),3), "-xr", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot(exp8(1:k:end,1), exp8(1:k:end,3), "o--c", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot(exp9(1:k:end,1), exp9(1:k:end,3), "o--m", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot(exp7(1:k:end,1), exp7(1:k:end,3), "o--g", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot(exp6(1:k:end,1), exp6(1:k:end,3), "o--r", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot(exp1(1:k:end,1), exp1(1:k:end,3), "-^", 'LineWidth', lw, 'MarkerSize', ms); hold on;  

plot([0.25]-1.5, [-3.5]+0.8, "ro", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0.25]-1.5, [-3.5]+0.2, "go", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0.25]-1.5, [-3.5]-0.4, "co", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0.25]-1.5, [-3.5]-1.0, "mo", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0.25]+0.5, [-3.5]+0.8, "rx", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0.25]+0.5, [-3.5]+0.2, "gx", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0.25]+0.5, [-3.5]-0.4, "cx", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0.25]+0.5, [-3.5]-1.0, "mx", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0.25]-1.5, [-3.5]+1.4, "^", 'LineWidth', lw, 'MarkerSize', ms); hold on; 

plot([0;0.5]-1.5, [-3.5;-3.5]+0.8, "-r", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0;0.5]-1.5, [-3.5;-3.5]+0.2, "-g", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0;0.5]-1.5, [-3.5;-3.5]-0.4, "-c", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0;0.5]-1.5, [-3.5;-3.5]-1.0, "-m", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0;0.5]+0.5, [-3.5;-3.5]+0.8, "-r", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0;0.5]+0.5, [-3.5;-3.5]+0.2, "-g", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0;0.5]+0.5, [-3.5;-3.5]-0.4, "-c", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0;0.5]+0.5, [-3.5;-3.5]-1.0, "-m", 'LineWidth', lw, 'MarkerSize', ms); hold on; 
plot([0;0.5]-1.5, [-3.5;-3.5]+1.4, "-", 'LineWidth', lw, 'MarkerSize', ms); hold on; 

axis([-2 10 -5 4]);
print -r300 -dpng bla.png
