import pandas as pd
import matplotlib.pyplot as plt
import glob

def load_data():
    computation_times = {}
    tree_sizes = {}

    # CSV 파일을 읽고 에이전트 수에 따라 데이터를 분류 및 정렬
    for filename in glob.glob("decentralized_benchmark_results_agents_*.csv"):
        num_agents = int(filename.split('_')[-1].split('.')[0])  # 파일 이름에서 에이전트 수 추출
        data = pd.read_csv(filename)

        # 데이터 저장
        computation_times[num_agents] = data['ComputationTime']
        #tree_sizes[num_agents] = data['TreeSize']

    # 에이전트 수를 기준으로 정렬하여 반환
    computation_times = dict(sorted(computation_times.items()))
    #tree_sizes = dict(sorted(tree_sizes.items()))

    return computation_times#, tree_sizes

def compute_averages(data):
    return {k: v.mean() for k, v in data.items()}

def plot_line(x, y, title, xlabel, ylabel):
    plt.figure(figsize=(10, 6))
    plt.plot(x, y, marker='o')
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.show()

def main():
    # 데이터 불러오기
    # computation_times, tree_sizes = load_data()
    computation_times = load_data()

    # 평균 계산 시간과 평균 트리 크기 계산
    avg_computation_times = compute_averages(computation_times)
    # avg_tree_sizes = compute_averages(tree_sizes)

    # Plot 1: Average Computation Time vs Number of Agents
    plot_line(
        list(avg_computation_times.keys()),
        list(avg_computation_times.values()),
        'Average Computation Time vs Number of Agents',
        'Number of Agents',
        'Average Computation Time (s)'
    )

    # Plot 2: Average Tree Size vs Number of Agents
    # plot_line(
    #     list(avg_tree_sizes.keys()),
    #     list(avg_tree_sizes.values()),
    #     'Average Tree Size vs Number of Agents',
    #     'Number of Agents',
    #     'Average Tree Size'
    # )

if __name__ == "__main__":
    main()
