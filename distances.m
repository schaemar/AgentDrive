A = [219.13252 217.31789 214.6021 210.99304 ]
Atime = [1.002 2.004 3.006 4.008 ]
B = [221.18332 218.63083 215.90877 ]
Btime = [2.004 3.006 4.008 ]
C = [219.13252 ]
Ctime = [4.008 ]
figure;
hold on;
plot(Atime,A,'Color',[0.14262408 0.28360736 0.09164882])
plot(Btime,B,'Color',[0.4848166 0.6978105 0.67561793])
plot(Ctime,C,'Color',[0.631319 0.9122018 0.76541644])
title('Car distances to the junction')
ylabel('distance')
xlabel('Simulation time')
hold off;