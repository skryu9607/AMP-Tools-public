import matplotlib.pyplot as plt
import re

def analyze_and_plot_log_extended(log_data):
    # Regular expressions for extracting data
    # valid_solution_pattern = r"Goal Biased RRT reached the goal."
    # computation_time_pattern = r"RRT Computation Time: ([0-9.]+) seconds"
    # path_length_pattern = r"Total Path Length: ([0-9.]+)"
    valid_solution_pattern = r"End A star"
    computation_time_pattern = r"PRM Computation Time: ([0-9.]+) seconds"
    path_length_pattern = r"Total Path Length: ([0-9.]+)"

    # Extract data using regular expressions
    valid_solutions = len(re.findall(valid_solution_pattern, log_data))
    computation_times = [float(time) for time in re.findall(computation_time_pattern, log_data)]
    path_lengths = [float(length) for length in re.findall(path_length_pattern, log_data)]

    # Calculate the total number and sum of computation times and path lengths
    total_computation_time = sum(computation_times)
    total_path_length = sum(path_lengths)
    Type = 'PRM'
    WS = 'BenchMarks 100'

    # Create summary dictionary
    summary = {
        "Type": Type,
        "WS": WS,
        "N" : 1000,
        "r" : 2.0,
        "valid_solutions_count": valid_solutions,
        "total_computation_time": total_computation_time,
        "average total computation time": total_computation_time / valid_solutions if valid_solutions > 0 else 0,
        "total_path_length": total_path_length,
        "average total path length": total_path_length / valid_solutions if valid_solutions > 0 else 0,
    }
    
    # Print summary
    print(summary)
    
    # Plot boxplots
    fig, ax = plt.subplots(1, 2, figsize=(14, 6))

    # Boxplot for Computation Times
    ax[0].boxplot(computation_times)
    ax[0].set_title("Computation Times Distribution")
    ax[0].set_ylabel("Time (seconds)")
    ax[0].set_xticks([1])
    ax[0].set_xticklabels(["Computation Times"])

    # Boxplot for Path Lengths
    ax[1].boxplot(path_lengths)
    ax[1].set_title("Path Lengths Distribution")
    ax[1].set_ylabel("Path Length")
    ax[1].set_xticks([1])
    ax[1].set_xticklabels(["Path Lengths"])

    plt.tight_layout()
    plt.show()

# Example usage with extended log data
log_data_extended_example = """ >[LOG] Benchmarking PRM 
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0495727 seconds
Total Path Length: 11.3865
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.048352 seconds
Total Path Length: 10.538
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0458986 seconds
Total Path Length: 11.6884
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0430533 seconds
Total Path Length: 8.87003
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0326963 seconds
Total Path Length: 10.5591
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0397566 seconds
Total Path Length: 10.5545
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0446834 seconds
Total Path Length: 11.5815
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.044685 seconds
Total Path Length: 10.207
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0345654 seconds
Total Path Length: 9.25304
 >[LOG] Trial 10 / 100 
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0384063 seconds
Total Path Length: 10.4777
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0328498 seconds
Total Path Length: 9.03909
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0374334 seconds
Total Path Length: 9.15504
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0479443 seconds
Total Path Length: 9.79686
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0309044 seconds
Total Path Length: 10.8015
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0442026 seconds
Total Path Length: 8.87688
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0405237 seconds
Total Path Length: 12.9775
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0431106 seconds
Total Path Length: 11.3351
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0443346 seconds
Total Path Length: 9.8893
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0518374 seconds
Total Path Length: 9.3398
 >[LOG] Trial 20 / 100 
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0452839 seconds
Total Path Length: 13.5364
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0260162 seconds
Total Path Length: 8.85881
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0271591 seconds
Total Path Length: 9.79726
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.036527 seconds
Total Path Length: 9.79774
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0345197 seconds
Total Path Length: 8.74582
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0307136 seconds
Total Path Length: 9.91605
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0417679 seconds
Total Path Length: 9.23365
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0343811 seconds
Total Path Length: 8.67468
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0365156 seconds
Total Path Length: 10.3311
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0400689 seconds
Total Path Length: 9.69665
 >[LOG] Trial 30 / 100 
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0382687 seconds
Total Path Length: 8.70311
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0252687 seconds
Total Path Length: 10.0408
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0265022 seconds
Total Path Length: 10.0766
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0402728 seconds
Total Path Length: 10.5355
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0335058 seconds
Total Path Length: 10.3936
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0447704 seconds
Total Path Length: 8.92932
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0440952 seconds
Total Path Length: 11.5639
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0317783 seconds
Total Path Length: 13.6964
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0435119 seconds
Total Path Length: 10.7756
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.028011 seconds
Total Path Length: 10.065
 >[LOG] Trial 40 / 100 
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0400184 seconds
Total Path Length: 9.15513
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0406531 seconds
Total Path Length: 9.41612
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.034375 seconds
Total Path Length: 9.61885
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0395928 seconds
Total Path Length: 10.2853
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0319274 seconds
Total Path Length: 9.69087
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0339907 seconds
Total Path Length: 10.373
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0535928 seconds
Total Path Length: 8.60336
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.029517 seconds
Total Path Length: 12.3281
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0338619 seconds
Total Path Length: 10.4424
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0461802 seconds
Total Path Length: 10.5502
 >[LOG] Trial 50 / 100 
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0319114 seconds
Total Path Length: 10.3526
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0342084 seconds
Total Path Length: 9.97859
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0296192 seconds
Total Path Length: 10.4764
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0422147 seconds
Total Path Length: 11.4029
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0461733 seconds
Total Path Length: 10.0475
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.025646 seconds
Total Path Length: 10.3747
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0306718 seconds
Total Path Length: 9.00063
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0373762 seconds
Total Path Length: 9.82559
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0428772 seconds
Total Path Length: 11.9306
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.037328 seconds
Total Path Length: 10.8072
 >[LOG] Trial 60 / 100 
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.032444 seconds
Total Path Length: 10.9715
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0321468 seconds
Total Path Length: 9.3496
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.045105 seconds
Total Path Length: 11.66
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0379019 seconds
Total Path Length: 10.3821
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0259795 seconds
Total Path Length: 9.14586
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0433397 seconds
Total Path Length: 8.82913
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0484578 seconds
Total Path Length: 11.9799
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0392487 seconds
Total Path Length: 10.1927
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.047679 seconds
Total Path Length: 8.6158
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0302201 seconds
Total Path Length: 9.36518
 >[LOG] Trial 70 / 100 
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0405876 seconds
Total Path Length: 10.9303
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0331068 seconds
Total Path Length: 9.2724
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0434901 seconds
Total Path Length: 8.91082
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0380303 seconds
Total Path Length: 8.73825
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0402673 seconds
Total Path Length: 13.3779
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0307482 seconds
Total Path Length: 9.763
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0318949 seconds
Total Path Length: 9.39928
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0423392 seconds
Total Path Length: 12.2733
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0302094 seconds
Total Path Length: 13.5522
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0386943 seconds
Total Path Length: 11.438
 >[LOG] Trial 80 / 100 
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.033875 seconds
Total Path Length: 9.20675
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0396054 seconds
Total Path Length: 9.16439
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0374594 seconds
Total Path Length: 8.81023
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0420744 seconds
Total Path Length: 9.11174
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0488173 seconds
Total Path Length: 10.9587
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0428956 seconds
Total Path Length: 11.2195
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0435622 seconds
Total Path Length: 10.4916
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.024587 seconds
Total Path Length: 9.21535
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0519185 seconds
Total Path Length: 11.3395
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0441457 seconds
Total Path Length: 10.6228
 >[LOG] Trial 90 / 100 
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.041728 seconds
Total Path Length: 9.398
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0279242 seconds
Total Path Length: 10.1985
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0351011 seconds
Total Path Length: 9.56885
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0313695 seconds
Total Path Length: 10.2096
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0396932 seconds
Total Path Length: 11.0386
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0305524 seconds
Total Path Length: 9.80107
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0418661 seconds
Total Path Length: 9.55527
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0324863 seconds
Total Path Length: 8.7523
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0266604 seconds
Total Path Length: 10.4624
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0460657 seconds
Total Path Length: 10.4928
 >[LOG] Trial 100 / 100 
Sampling is finished.
Roadmap construction is completed.
End A star
PRM Computation Time: 0.0367125 seconds
Total Path Length: 14.2396
 >[LOG] PRM benchmark finished! 
"""

# Analyze and plot the extended example log data
analyze_and_plot_log_extended(log_data_extended_example)
