import pandas as pd
import numpy as np
import argparse

def process_tick_intervals(filename):
    # CSV 파일 읽기 (한 줄에 하나의 tick 값이 있다고 가정)
    df = pd.read_csv(filename, header=None, names=["Tick"])
    # open as int   
    df["Tick"] = df["Tick"].astype(int)

    # Tick 간격 계산
    df["Interval"] = df["Tick"].diff()
    # Convert tick to ns (divide by 1000)
    df["Interval"] = df["Interval"] / 1000

    # 첫 번째 값은 NaN이므로 제거
    df = df.dropna()
    
    trash_threshold = 10000
    #delete the row with interval larger than trash_threshold
    df = df[df["Interval"] < trash_threshold]
    print(df)
    
    # Get the row with interval larger than threshold   
    # threshold = 1200
    # for i in range(1, len(df["Interval"])):
    #     if df["Interval"][i] > threshold:
    #         print(f"Interval larger than {threshold} ns: df['Interval'][{i}] = {df['Interval'][i]}")

    # 통계 정보 계산 
    stats = {
        "Mean (ns)": np.mean(df["Interval"]),
        "Median (ns)": np.median(df["Interval"]),
        "Min (ns)": np.min(df["Interval"]),
        "Max (ns)": np.max(df["Interval"]),
        "Std (ns)": np.std(df["Interval"]),
    }

    return df["Interval"], stats

def main():
    parser = argparse.ArgumentParser(description="Parse tick CSV file and compute interval statistics.")
    parser.add_argument("filename", type=str, help="Path to the CSV file containing tick values.")
    args = parser.parse_args()

    intervals, stats = process_tick_intervals(args.filename)

    # 결과 출력
    print("\nStatistics (in ns):")
    for key, value in stats.items():
        print(f"{key}: {value:.2f} ns")
    print("Intervals (ns):")
    print(intervals.to_string(index=False))

if __name__ == "__main__":
    main()