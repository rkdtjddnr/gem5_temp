import pandas as pd
import numpy as np
import argparse
import re
import matplotlib.pyplot as plt
    
def plot_combined_ioxbar_utilization_stacked(summary_df):
    categories = ['fail', 'desc', 'mbuf', 'mmio']
    colors = ['#d62728', '#1f77b4', '#ff7f0e', '#2ca02c']

    # 막대차트 데이터 준비
    bar_data = []
    for _, row in summary_df.iterrows():
        name = row['name']
        utilization = row['utilization(%)']
        fail_pct = row['fail(%)'] * utilization / 100
        desc_pct = row['desc(%)'] * row['success(%)'] * utilization / 10000
        mbuf_pct = row['mbuf(%)'] * row['success(%)'] * utilization / 10000
        mmio_pct = row['mmio(%)'] * row['success(%)'] * utilization / 10000
        bar_data.append([name, fail_pct, desc_pct, mbuf_pct, mmio_pct])

    bar_df = pd.DataFrame(bar_data, columns=['name'] + categories)
    bar_df.set_index('name', inplace=True)

    fig_width = max(6, len(bar_df) * 1.2)  # 항목 수에 따라 적절히 조정
    fig, ax = plt.subplots(figsize=(fig_width, 5))  # 가로 길이 줄임
    bar_width = 0.4
    x = range(len(bar_df))
    bottoms = [0] * len(bar_df)

    for i, (cat, color) in enumerate(zip(categories, colors)):
        values = bar_df[cat].values
        bars = ax.bar(x, values, bar_width, bottom=bottoms, label=cat, color=color)
        for j, bar in enumerate(bars):
            height = bar.get_height()
            if height > 1.0:
                ax.text(
                    bar.get_x() + bar.get_width() / 2,
                    bar.get_y() + height / 2,
                    f'{height:.1f}%',
                    ha='center', va='center',
                    color='black', fontsize=9
                )
            bottoms[j] += height

    ax.set_title("IOXBAR Layer Utilization Breakdown (Stacked)", fontsize=13)
    ax.set_ylabel("Utilization (%)")
    ax.set_ylim(0, 100)
    ax.set_xticks(x)
    ax.set_xticklabels(bar_df.index, rotation=15)
    ax.legend(title="Category")
    ax.grid(axis='y', linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.savefig(f"IOXBAR_Utilization_Stacked_Breakdown.png")

def plot_success_only_utilization_stacked(summary_df):
    categories = ['desc', 'mbuf', 'mmio']
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c']

    # 막대차트 데이터 준비
    bar_data = []
    for _, row in summary_df.iterrows():
        name = row['name']
        desc_pct = row['desc(%)']
        mbuf_pct = row['mbuf(%)']
        mmio_pct = row['mmio(%)']
        bar_data.append([name, desc_pct, mbuf_pct, mmio_pct])

    bar_df = pd.DataFrame(bar_data, columns=['name'] + categories)
    bar_df.set_index('name', inplace=True)

    fig_width = max(6, len(bar_df) * 1.2)
    fig, ax = plt.subplots(figsize=(fig_width, 5))
    bar_width = 0.4
    x = range(len(bar_df))
    bottoms = [0] * len(bar_df)

    for cat, color in zip(categories, colors):
        values = bar_df[cat].values
        bars = ax.bar(x, values, bar_width, bottom=bottoms, label=cat, color=color)
        for j, bar in enumerate(bars):
            height = bar.get_height()
            if height > 1.0:
                ax.text(
                    bar.get_x() + bar.get_width() / 2,
                    bar.get_y() + height / 2,
                    f'{height:.1f}%',
                    ha='center', va='center',
                    color='black', fontsize=9
                )
            bottoms[j] += height

    ax.set_title("Traffic Breakdown (Stacked)", fontsize=13)
    ax.set_ylabel("Breakdown (%)")
    ax.set_ylim(0, 100)
    ax.set_xticks(x)
    ax.set_xticklabels(bar_df.index, rotation=15)
    ax.legend(title="Category")
    ax.grid(axis='y', linestyle='--', alpha=0.7)
    plt.tight_layout()
    plt.savefig(f"IOXBAR_Success_Only_Traffic_Stacked_Breakdown.png")

def plot_ioxbar_utilization_breakdown(summary_df):
    for _, row in summary_df.iterrows():
        labels = ['desc', 'mbuf', 'mmio']
        values = [
            row['desc(%)'] * row['success(%)'] / 100,
            row['mbuf(%)'] * row['success(%)'] / 100,
            row['mmio(%)'] * row['success(%)'] / 100
        ]
        fail = row['fail(%)']
        success_total = sum(values)

        plt.figure(figsize=(8, 5))
        plt.bar(['fail'], [fail], label='fail')
        plt.bar(labels, values, label='success breakdown')
        plt.title(f"IOXBAR Utilization Breakdown: {row['name']}")
        plt.ylabel('Utilization (%)')
        plt.ylim(0, 100)
        plt.legend()
        plt.grid(axis='y')
        plt.tight_layout()
        plt.savefig(f"IOXBAR_Utilization_Breakdown_{row['name']}.png")

def parse_ioxbar_log(file_path):
    pattern = re.compile(
        r'\[LOG\], (\d+), (IOXBAR_(REQ|RESP)\[\d+\]_OCCUPANCY), ([\d\.]+), ([\d\.]+), ([\d\.]+), ([\d\.]+), ([\d\.]+)'
    )

    data = []
    with open(file_path, 'r') as f:
        for line in f:
            match = pattern.search(line)
            if match:
                tick = int(match.group(1))
                name = match.group(2)
                values = list(map(float, match.groups()[3:]))
                data.append((tick, name, *values))

    df = pd.DataFrame(
        data,
        columns=[
            "tick", "name", "fail_occupancy", "success_occupancy",
            "descriptor_occupancy", "mbuf_occupancy", "mmio_occupancy"
        ]
    )

    # Group by each occupancy type
    results = []
    for name, group in df.groupby("name"):
        group = group.sort_values("tick").reset_index(drop=True)
        start_tick = group["tick"].iloc[0]
        end_tick = group["tick"].iloc[-1]
        delta_time = end_tick - start_tick

        delta_fail = group["fail_occupancy"].iloc[-1] - group["fail_occupancy"].iloc[0]
        delta_success = group["success_occupancy"].iloc[-1] - group["success_occupancy"].iloc[0]
        delta_desc = group["descriptor_occupancy"].iloc[-1] - group["descriptor_occupancy"].iloc[0]
        delta_mbuf = group["mbuf_occupancy"].iloc[-1] - group["mbuf_occupancy"].iloc[0]
        delta_mmio = group["mmio_occupancy"].iloc[-1] - group["mmio_occupancy"].iloc[0]

        total = delta_fail + delta_success

        result = {
            "name": name,
            "start_tick": start_tick,
            "end_tick": end_tick,
            "duration": delta_time,
            "total_occupancy": total,
            "utilization(%)": (total / delta_time) * 100 if delta_time > 0 else 0,
            "fail(%)": (delta_fail / total) * 100 if total > 0 else 0,
            "success(%)": (delta_success / total) * 100 if total > 0 else 0,
            "desc(%)": (delta_desc / delta_success) * 100 if delta_success > 0 else 0,
            "mbuf(%)": (delta_mbuf / delta_success) * 100 if delta_success > 0 else 0,
            "mmio(%)": (delta_mmio / delta_success) * 100 if delta_success > 0 else 0,
        }
        results.append(result)
    
    summary_df = pd.DataFrame(results)
    print(summary_df.to_string(index=False))
    return summary_df

def main():
    parser = argparse.ArgumentParser(description="Parse SW tail update logs and compute time intervals.")
    parser.add_argument("filename", type=str, help="Path to the log file")
    args = parser.parse_args()
    
    summary_df = parse_ioxbar_log(args.filename)
    plot_combined_ioxbar_utilization_stacked(summary_df)
    plot_success_only_utilization_stacked(summary_df)
    
    

if __name__ == "__main__":
    main()