X = categorical({'double (size 8)', 'float (size 4)', 'fp16'});
Y = [37.26	18.312	85.96];  % Using the last row of data

bar(X, Y, 0.5, 'red')

title('C Bit Widths (unthreaded)')
xlabel('Bit Widths')
ylabel('Execution Time (seconds)')