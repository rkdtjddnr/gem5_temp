import os
import re
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from multiprocessing import Pool, cpu_count
from tqdm import tqdm

# 설정값
base_dir = "/home/jmhhh/Documents/CXL_network/gem5_dpdk_multiqueue/gem5-dpdk-setup/rundir/dpdk-set-2core"
num_queues = 2
script = "dpdk-set"
target_case = "1NIC-2Qs-256SIZE-31457280RATE-60Gbps-ddio-enabled-dpdk-set.sh"
log_file = "simout"
log_file_path = os.path.join(base_dir, target_case, log_file)

# 각 로그 라인에서 데이터를 추출하는 정규 표현식
log_pattern = re.compile(
    r"RXD\[\d+\] RX Total Time: (\d+), EtherLink Time: (\d+), Port2Fifo Time: (\d+), Fifo2DMAStart Time: (\d+), DMA Time: (\d+)"
)

# 로그 라인 파싱 함수
def parse_log_line(line):
    match = log_pattern.search(line)
    if match:
        # 각 시간을 tick에서 ms로 변환 (tick / 10^8)
        return {
            'total_time': int(match.group(1)) / 10**8,
            'etherlink_time': int(match.group(2)) / 10**8,
            'port2fifo_time': int(match.group(3)) / 10**8,
            'fifo2dma_start_time': int(match.group(4)) / 10**8,
            'dma_time': int(match.group(5)) / 10**8
        }
    return None

# 로그 파일을 읽으면서 바로 파싱
if __name__ == "__main__":
    with open(log_file_path, 'r') as file:
        log_lines = file.readlines()
    
    with Pool(cpu_count()) as pool:
        parsed_data = list(tqdm(pool.imap(parse_log_line, log_lines), total=len(log_lines), desc="Parsing logs"))

    # None 값 제거
    parsed_data = [data for data in parsed_data if data]

    # DataFrame으로 변환
    df = pd.DataFrame(parsed_data)

    # 각 타일(90%, 95%, 99%)에 해당하는 latency를 계산
    percentiles = [30, 90, 95, 99]
    tail_latency = np.percentile(df['total_time'], percentiles)

    # 타일 별 breakdown을 추출 (total_time은 제외)
    breakdown_components = ['etherlink_time', 'fifo2dma_start_time', 'dma_time']
    breakdowns = {p: df[df['total_time'] >= tail_latency[i]][breakdown_components].mean() for i, p in enumerate(percentiles)}

    # 100% 기준으로 정규화한 breakdown 계산
    normalized_breakdowns = {p: breakdowns[p] / breakdowns[p].sum() * 100 for p in percentiles}

    # Breakdown 데이터를 DataFrame으로 변환
    breakdown_df = pd.DataFrame(breakdowns).transpose()
    normalized_breakdown_df = pd.DataFrame(normalized_breakdowns).transpose()

    # Breakdown 데이터 시각화
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))

    breakdown_df.plot(kind='bar', stacked=True, ax=axes[0])
    axes[0].set_title('Tail Latency Breakdown (90th, 95th, 99th Percentile)')
    axes[0].set_xlabel('Percentile')
    axes[0].set_ylabel('Time (ms)')
    axes[0].legend(title='Breakdown Component')
    axes[0].set_xticks(range(len(breakdown_df.index)))
    axes[0].set_xticklabels(breakdown_df.index)

    normalized_breakdown_df.plot(kind='bar', stacked=True, ax=axes[1])
    axes[1].set_title('Normalized Breakdown (100% Total Latency)')
    axes[1].set_xlabel('Percentile')
    axes[1].set_ylabel('Percentage (%)')
    axes[1].legend(title='Breakdown Component')
    axes[1].set_xticks(range(len(normalized_breakdown_df.index)))
    axes[1].set_xticklabels(normalized_breakdown_df.index)

    plt.tight_layout()
    plt.savefig(f"tail_latency_breakdown_with_normalized_{target_case}.png")
    plt.show()
