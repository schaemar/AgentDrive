A = [1.0 2.0 3.0 4.0 ]
Atime = [1.002 2.004 3.006 4.008 ]
B = [1.6999961 2.8999994 2.8999972 ]
Btime = [2.004 3.006 4.008 ]
C = [1.0 ]
Ctime = [4.008 ]
figure;
hold on;
plot(Atime,A,'Color',[0.9202252 0.3931834 0.14793557])
plot(Btime,B,'Color',[0.21811223 0.2371828 0.14733213])
plot(Ctime,C,'Color',[0.17864233 0.88998526 0.29824692])
title('Car speeds')
ylabel('speed')
xlabel('Simulation time')
hold off;