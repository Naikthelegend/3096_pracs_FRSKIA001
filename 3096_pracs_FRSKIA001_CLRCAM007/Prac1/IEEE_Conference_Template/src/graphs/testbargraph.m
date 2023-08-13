X = categorical({'non-threaded ','threads 1','threads 2', 'threads 4','threads 8','threads 16', 'threads 32'});
X = reordercats(X,{'non-threaded ','threads 1','threads 2', 'threads 4','threads 8','threads 16', 'threads 32'});
Y = [9.408 20.49722 26.698 11.272 8.702 12.646 15.996];
bar(X, Y, 0.5, 'FaceColor', 'red')
hold on
title('Execution Time for Different Thread Configurations')
xlabel('Thread Configurations')
ylabel('Execution Time (milliseconds)')

print -depsc bargraf_on_C_Threaded.eps

hold off				
