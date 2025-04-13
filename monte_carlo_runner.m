% Runs the RTLS flight algorithm 1000 times and records final landing distances

clear; clc;
num_trials = 100;
results = zeros(num_trials, 1);
successful_landings = 0;

for i = 1:num_trials
    try
        % Reset global variable
        global FINAL_DISTANCE
        FINAL_DISTANCE = NaN;


        %run sims
        main(true);
       
        % Save final distance from current run
        if ~isnan(FINAL_DISTANCE)
            results(i) = FINAL_DISTANCE;

            if FINAL_DISTANCE <= 800
                successful_landings = successful_landings + 1;
            end
        else
            results(i) = NaN;
        end
    catch ME
        warning("Simulation %d failed: %s", i, getReport(ME));
        results(i) = NaN;
    end
end

% Clean up failed runs
valid_results = results(~isnan(results));

% Plot histogram
figure;
histogram(valid_results, 50);
xlabel('Distance from Target (ft)');
ylabel('Number of Simulations');
title('RTLS Monte Carlo - Landing Accuracy Across 100 Runs');
grid on;

% Print stats
fprintf("Monte Carlo Results (n = %d):\n", numel(valid_results));
fprintf("Mean Distance: %.2f ft\n", mean(valid_results));
fprintf("Max Distance: %.2f ft\n", max(valid_results));
fprintf("Min Distance: %.2f ft\n", min(valid_results));
fprintf("Std Deviation: %.2f ft\n", std(valid_results));
fprintf("Successful Landings (<800 ft): %d / %d (%.2f%%)\n", ...
    successful_landings, num_trials, 100 * successful_landings / num_trials);
