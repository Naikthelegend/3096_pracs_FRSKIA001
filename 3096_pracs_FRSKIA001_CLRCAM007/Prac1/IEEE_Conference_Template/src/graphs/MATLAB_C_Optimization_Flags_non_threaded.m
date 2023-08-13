X = categorical({'-O0','-O1','-O2', '-O3','-Ofast','-Os','-Og'});
Y = [24.15 22.016 19.756 24.946 14.674 12.348 19.008];

% Create a bar graph
figure;
bar(X, Y,0.5, "red");

% Add a centered title
title({'C Optimization Flags (non-threaded)', 'Execution Time for Different Flags'})

% Label the x and y axes
xlabel('Optimization Flags')
ylabel('Execution Time (milliseconds)')

print -C_Optimization_Flags_non_threaded.eps