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

    return computation_times #, tree_sizes

def plot_boxplot(data, title, ylabel):
    plt.figure(figsize=(10, 6))
    plt.boxplot(data.values(), labels=[f'{k} agents' for k in data.keys()])
    plt.title(title)
    plt.xlabel('Number of Agents')
    plt.ylabel(ylabel)
    plt.grid(True)
    plt.show()

def main():
    # 데이터 불러오기
    #computation_times, tree_sizes = load_data()
    computation_times = load_data()

    # 컴퓨테이션 시간 박스 플롯
    plot_boxplot(computation_times, 'Computation Time for Different Number of Agents', 'Computation Time (s)')

    # 트리 크기 박스 플롯
    #plot_boxplot(tree_sizes, 'Tree Size for Different Number of Agents', 'Tree Size')

if __name__ == "__main__":
    main()
