#!/usr/bin/env python3

import os
import csv
import sys
import matplotlib.pyplot as plt

def main():
    csv_path = os.path.expanduser('~/ship_ws/maps/maps_metrics.csv')

    if not os.path.exists(csv_path):
        print(f"File not found: {csv_path}")
        sys.exit(1)

    try:
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        if not rows:
            print(f"File is empty: {csv_path}")
            sys.exit(0)

        headers = list(rows[0].keys())
        table_rows = [[row.get(h, '') for h in headers] for row in rows]

        fig, ax = plt.subplots(figsize=(14, max(4, 1 + len(rows) * 0.4)))
        fig.patch.set_facecolor('white')

        table = ax.table(cellText=table_rows, colLabels=headers,
                        cellLoc='center', loc='center',
                        bbox=[0, 0, 1, 1])

        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1, 2)

        for i in range(len(headers)):
            table[(0, i)].set_facecolor('#CCCCCC')
            table[(0, i)].set_text_props(weight='bold', color='black')

        for i in range(1, len(rows) + 1):
            for j in range(len(headers)):
                table[(i, j)].set_facecolor('white')
                table[(i, j)].set_text_props(color='black')
                table[(i, j)].set_edgecolor('black')

        ax.axis('off')
        fig.suptitle(f'Map Metrics ({len(rows)} entries)',
                     fontsize=14, fontweight='bold', color='black')

        plt.tight_layout()
        plt.show()

    except Exception as e:
        print(f"Error reading CSV: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
