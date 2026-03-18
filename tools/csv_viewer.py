"""
RFx CSV Data Viewer
-------------------
Interactive visualization of CSV data recorded from the RFx dashboard.

Usage:
  python csv_viewer.py                    # Opens a file dialog
  python csv_viewer.py path/to/data.csv   # Opens a specific file

Requirements:
  pip install matplotlib pandas
"""

import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import SpanSelector, CheckButtons, Button
import matplotlib.gridspec as gridspec
import numpy as np


def load_csv(filepath):
    """Load and parse an RFx CSV file."""
    df = pd.read_csv(filepath)
    # Convert timestamp to seconds
    if 'Timestamp_ms' in df.columns:
        df['Time_s'] = df['Timestamp_ms'] / 1000.0
    return df


def create_viewer(df, filename=""):
    """Create the interactive multi-panel plot."""

    fig = plt.figure(figsize=(16, 11))
    fig.suptitle(f'RFx Data Viewer — {os.path.basename(filename)}', fontsize=13, fontweight='bold')
    fig.patch.set_facecolor('#1e1e1e')

    gs = gridspec.GridSpec(5, 1, hspace=0.35, left=0.07, right=0.88, top=0.93, bottom=0.06)

    t = df['Time_s'].values

    # --- Color scheme ---
    colors = {
        'commanded': '#5ab5ff',
        'actual': '#ff9f43',
        'torque': '#ff6b6b',
        'tension': '#feca57',
        'line_length': '#54a0ff',
        'target': '#2ecc71',
        'pitch': '#a29bfe',
        'yaw': '#fd79a8',
        'pitch_vel': '#6c5ce7',
        'yaw_vel': '#e84393',
        'slack': '#ff4444',
        'grid': '#333333',
        'text': '#cccccc',
        'bg': '#1e1e1e',
        'panel_bg': '#242424',
    }

    style_kwargs = dict(linewidth=0.8)

    def style_ax(ax, title):
        ax.set_facecolor(colors['panel_bg'])
        ax.tick_params(colors=colors['text'], labelsize=8)
        ax.xaxis.label.set_color(colors['text'])
        ax.yaxis.label.set_color(colors['text'])
        ax.title.set_color(colors['text'])
        ax.title.set_fontsize(10)
        ax.set_title(title, loc='left', pad=4)
        ax.grid(True, color=colors['grid'], linewidth=0.5, alpha=0.7)
        for spine in ax.spines.values():
            spine.set_color(colors['grid'])

    axes = []

    # ── Panel 1: Velocity ──
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, df['Commanded_Vel_rps'], color=colors['commanded'], label='Commanded', **style_kwargs)
    ax1.plot(t, df['Actual_Vel_rps'], color=colors['actual'], label='Actual', **style_kwargs)
    ax1.set_ylabel('rev/s', fontsize=8, color=colors['text'])
    style_ax(ax1, 'Motor Velocity')
    ax1.legend(fontsize=7, loc='upper right', framealpha=0.5, facecolor=colors['panel_bg'],
               labelcolor=colors['text'])
    axes.append(ax1)

    # ── Panel 2: Torque & Tension ──
    ax2 = fig.add_subplot(gs[1], sharex=ax1)
    ax2.plot(t, df['Torque_Nm'], color=colors['torque'], label='Torque (Nm)', **style_kwargs)
    ax2_twin = ax2.twinx()
    ax2_twin.plot(t, df['Tension_N'], color=colors['tension'], label='Tension (N)', **style_kwargs)
    ax2.set_ylabel('Nm', fontsize=8, color=colors['torque'])
    ax2_twin.set_ylabel('N', fontsize=8, color=colors['tension'])
    ax2_twin.tick_params(colors=colors['tension'], labelsize=8)
    ax2_twin.spines['right'].set_color(colors['tension'])
    style_ax(ax2, 'Torque & Tension')
    # Combined legend
    lines1, labels1 = ax2.get_legend_handles_labels()
    lines2, labels2 = ax2_twin.get_legend_handles_labels()
    ax2.legend(lines1 + lines2, labels1 + labels2, fontsize=7, loc='upper right',
               framealpha=0.5, facecolor=colors['panel_bg'], labelcolor=colors['text'])
    axes.append(ax2)

    # ── Panel 3: Line Length & Target ──
    ax3 = fig.add_subplot(gs[2], sharex=ax1)
    ax3.plot(t, df['Line_Length_m'], color=colors['line_length'], label='Line Length', **style_kwargs)
    if 'Target_Length_m' in df.columns:
        ax3.plot(t, df['Target_Length_m'], color=colors['target'], label='Target',
                 linestyle='--', **style_kwargs)
    # Highlight slack regions
    if 'Torque_Nm' in df.columns:
        # Approximate slack detection visually (torque near zero)
        slack_mask = df['Torque_Nm'].abs() < 0.015
        if slack_mask.any():
            ax3.fill_between(t, ax3.get_ylim()[0] if ax3.get_ylim()[0] != ax3.get_ylim()[1] else 0,
                             df['Line_Length_m'].max() * 1.05,
                             where=slack_mask, color=colors['slack'], alpha=0.08, label='Low tension')
    ax3.set_ylabel('m', fontsize=8, color=colors['text'])
    style_ax(ax3, 'Line Length')
    ax3.legend(fontsize=7, loc='upper right', framealpha=0.5, facecolor=colors['panel_bg'],
               labelcolor=colors['text'])
    axes.append(ax3)

    # ── Panel 4: IMU Orientation ──
    ax4 = fig.add_subplot(gs[3], sharex=ax1)
    ax4.plot(t, df['Pitch_deg'], color=colors['pitch'], label='Pitch', **style_kwargs)
    ax4.plot(t, df['Yaw_deg'], color=colors['yaw'], label='Yaw', **style_kwargs)
    ax4.set_ylabel('deg', fontsize=8, color=colors['text'])
    style_ax(ax4, 'IMU Orientation')
    ax4.legend(fontsize=7, loc='upper right', framealpha=0.5, facecolor=colors['panel_bg'],
               labelcolor=colors['text'])
    axes.append(ax4)

    # ── Panel 5: Angular Velocity ──
    ax5 = fig.add_subplot(gs[4], sharex=ax1)
    ax5.plot(t, df['Pitch_Vel_dps'], color=colors['pitch_vel'], label='Pitch vel', **style_kwargs)
    ax5.plot(t, df['Yaw_Vel_dps'], color=colors['yaw_vel'], label='Yaw vel', **style_kwargs)
    ax5.set_ylabel('°/s', fontsize=8, color=colors['text'])
    ax5.set_xlabel('Time (s)', fontsize=9, color=colors['text'])
    style_ax(ax5, 'Angular Velocity')
    ax5.legend(fontsize=7, loc='upper right', framealpha=0.5, facecolor=colors['panel_bg'],
               labelcolor=colors['text'])
    axes.append(ax5)

    # ── Shared crosshair cursor ──
    vlines = []
    for ax in axes:
        vl = ax.axvline(x=0, color='#ffffff', linewidth=0.5, alpha=0.4, visible=False)
        vlines.append(vl)

    def on_mouse_move(event):
        if event.inaxes in axes:
            for vl in vlines:
                vl.set_xdata([event.xdata, event.xdata])
                vl.set_visible(True)
            fig.canvas.draw_idle()
        else:
            for vl in vlines:
                vl.set_visible(False)
            fig.canvas.draw_idle()

    fig.canvas.mpl_connect('motion_notify_event', on_mouse_move)

    # ── Zoom-to-span selector on bottom panel ──
    def on_select(xmin, xmax):
        for ax in axes:
            ax.set_xlim(xmin, xmax)
        fig.canvas.draw_idle()

    span = SpanSelector(ax5, on_select, 'horizontal', useblit=True,
                        props=dict(alpha=0.2, facecolor='#5ab5ff'),
                        interactive=True, drag_from_anywhere=True)

    # ── Reset zoom button ──
    ax_btn = fig.add_axes([0.89, 0.01, 0.1, 0.03])
    ax_btn.set_facecolor(colors['panel_bg'])
    btn_reset = Button(ax_btn, 'Reset Zoom', color=colors['panel_bg'], hovercolor='#444444')
    btn_reset.label.set_color(colors['text'])
    btn_reset.label.set_fontsize(8)

    def reset_zoom(event):
        for ax in axes:
            ax.set_xlim(t[0], t[-1])
        fig.canvas.draw_idle()

    btn_reset.on_clicked(reset_zoom)

    # ── Summary stats ──
    duration = t[-1] - t[0]
    samples = len(df)
    rate = samples / duration if duration > 0 else 0
    stats_text = (f"Duration: {duration:.1f}s  |  Samples: {samples}  |  Rate: {rate:.0f} Hz  |  "
                  f"Drag bottom panel to zoom  |  Scroll to pan")
    fig.text(0.07, 0.01, stats_text, fontsize=7, color='#888888')

    plt.show()


def main():
    filepath = None

    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        # Try tkinter file dialog
        try:
            import tkinter as tk
            from tkinter import filedialog
            root = tk.Tk()
            root.withdraw()
            filepath = filedialog.askopenfilename(
                title="Select RFx CSV file",
                filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
                initialdir=os.path.expanduser("~/Downloads")
            )
            root.destroy()
        except ImportError:
            print("Usage: python csv_viewer.py <path_to_csv>")
            sys.exit(1)

    if not filepath:
        print("No file selected.")
        sys.exit(0)

    if not os.path.exists(filepath):
        print(f"File not found: {filepath}")
        sys.exit(1)

    print(f"Loading: {filepath}")
    df = load_csv(filepath)
    print(f"  {len(df)} samples, columns: {list(df.columns)}")
    create_viewer(df, filepath)


if __name__ == '__main__':
    main()
