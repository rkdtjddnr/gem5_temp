import pandas as pd
import numpy as np
import argparse

def parse_log_file(filename, batch_size=32, ring_size=1024):
    # 파일에서 로그 데이터 읽기
    df = pd.read_csv(filename, sep=",")
    
    print(df)
    # 컬럼 이름의 공백 제거
    df.columns = df.columns.str.strip()
    
    # Head 업데이트 포인트 찾기
    update_points = []
    for _, row in df.iterrows():
        if row["Head"] % batch_size == 0:
            update_points.append((row["Tick"], row["Head"]))
    
    # 시간 간격 계산
    time_intervals = []
    for i in range(1, len(update_points)):
        prev_tick, prev_head = update_points[i - 1]
        curr_tick, curr_head = update_points[i]
        
        # 링 버퍼 고려하여 비교
        if prev_head > curr_head:
            prev_head -= ring_size
        
        if curr_head - prev_head == batch_size:
            time_intervals.append((curr_tick - prev_tick)/1000)
    
    # 통계 정보 계산
    stats = {
        "Mean (ns)": np.mean(time_intervals) if time_intervals else None,
        "Median (ns)": np.median(time_intervals) if time_intervals else None,
        "Min (ns)": np.min(time_intervals) if time_intervals else None,
        "Max (ns)": np.max(time_intervals) if time_intervals else None,
        "Std (ns)": np.std(time_intervals) if time_intervals else None,
    }
    
    return time_intervals, stats

def main():
    parser = argparse.ArgumentParser(description="Parse NIC head update logs and compute time intervals.")
    parser.add_argument("filename", type=str, help="Path to the log file")
    args = parser.parse_args()
    
    time_intervals, stats = parse_log_file(args.filename)
    
    # 결과 출력
    print("Head Update Time Intervals (ns):", time_intervals)
    print("Statistics (ns):", stats)

if __name__ == "__main__":
    main()
