X = categorical({'-O0','-O3','-Ofast','-Os'});
Y = [21.456 24.218 16.93 10.01];

bar(X, Y, 0.5, 'red')

title('C Unrolled Loops (non-threaded)')
xlabel('Optimisation Flags')
ylabel('Execution Time (seconds)')