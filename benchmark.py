import pandas as pd
import matplotlib.pyplot as plt

# CSV 파일 로드
df = pd.read_csv("benchmark_results.csv")

print(df.head())
grouped = df.groupby(['n', 'r']).agg(
    valid_solution_rate=('valid_solution', 'mean'),
    average_computation_time=('computation_time', 'mean'),
    average_path_length=('path_length', 'mean')
).reset_index()

# (n, r) 조합을 하나의 문자열로 합쳐서 새로운 열 생성
grouped['n_r_combination'] = grouped['n'].astype(str) + ", " + grouped['r'].astype(str)

print(grouped)

# Boxplot을 통해 각 성능 지표의 분포를 시각화
fig, axes = plt.subplots(1, 3, figsize=(18, 6))

# Computation time에 대한 boxplot
df['n_r_combination'] = df['n'].astype(str) + ", " + df['r'].astype(str)
df.boxplot(column='computation_time', by='n_r_combination', ax=axes[0])
axes[0].set_title("Computation Time by (n, r)")
axes[0].set_xlabel("(n, r) Combinations")
axes[0].set_ylabel("Time (seconds)")
axes[0].tick_params(axis='x', rotation=45)

# Path length에 대한 boxplot
df.boxplot(column='path_length', by='n_r_combination', ax=axes[1])
axes[1].set_title("Path Length by (n, r)")
axes[1].set_xlabel("(n, r) Combinations")
axes[1].set_ylabel("Length")
axes[1].tick_params(axis='x', rotation=45)

# 유효한 솔루션 비율에 대한 bar plot
grouped.plot.bar(x='n_r_combination', y='valid_solution_rate', ax=axes[2])
axes[2].set_title("Valid Solution Rate by (n, r)")
axes[2].set_xlabel("(n, r) Combinations")
axes[2].set_ylabel("Rate")

plt.tight_layout()
plt.suptitle("Benchmark Analysis", y=1.02)
plt.show()
