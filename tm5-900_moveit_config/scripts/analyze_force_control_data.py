#!/usr/bin/env python3
"""
TM Robot Force Control Data Analysis and Visualization

分析和可視化記錄的力控制數據
自動處理缺少的欄位
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import argparse
from matplotlib.patches import FancyArrow


def load_data(csv_file):
    """載入 CSV 數據"""
    df = pd.read_csv(csv_file)
    
    # 計算相對時間（從開始的秒數）
    df['time_relative'] = (df['ros_time_sec'] - df['ros_time_sec'].iloc[0]) + \
                          (df['ros_time_nsec'] - df['ros_time_nsec'].iloc[0]) / 1e9
    
    # 如果缺少 control_mode，從力量推斷
    if 'control_mode' not in df.columns:
        print("Warning: 'control_mode' column not found, inferring from force data...")
        # 假設當法向力 > 1.5N 時進入 FORCE_CONTROL
        df['control_mode'] = 'APPROACH'
        df.loc[df['normal_force_magnitude'] > 1.5, 'control_mode'] = 'FORCE_CONTROL'
    
    # 如果缺少 normal_force_error 和 force_integral，設為 NaN
    if 'normal_force_error' not in df.columns:
        df['normal_force_error'] = np.nan
    if 'force_integral' not in df.columns:
        df['force_integral'] = np.nan
    
    return df


def plot_force_data(df, save_dir=None):
    """繪製力數據"""
    fig, axes = plt.subplots(3, 1, figsize=(12, 12))
    fig.suptitle('Force Control Data', fontsize=16)
    
    time = df['time_relative'].values  # 轉換為 numpy array
    
    # 1. 法向力
    axes[0].plot(time, df['normal_force_magnitude'].values, 'b-', linewidth=2, label='Normal Force')
    if 'normal_force_error' in df.columns and not df['normal_force_error'].isna().all():
        target_force = df['normal_force_magnitude'] - df['normal_force_error']
        non_zero_target = target_force[target_force != 0]
        if len(non_zero_target) > 0:
            axes[0].axhline(y=non_zero_target.iloc[0], color='r', linestyle='--', label='Target Force')
    axes[0].set_ylabel('Force (N)')
    axes[0].set_title('Normal Force Magnitude')
    axes[0].legend()
    axes[0].grid(True)
    
    # 2. Base frame 的 XY 力
    axes[1].plot(time, df['force_base_x'].values, 'r-', label='Fx (base)', linewidth=1.5)
    axes[1].plot(time, df['force_base_y'].values, 'g-', label='Fy (base)', linewidth=1.5)
    axes[1].set_ylabel('Force (N)')
    axes[1].set_title('Force Components in Base Frame')
    axes[1].legend()
    axes[1].grid(True)
    
    # 3. Sensor frame 的力
    axes[2].plot(time, df['force_sensor_x'].values, 'r-', label='Fx (sensor)', alpha=0.7, linewidth=1.5)
    axes[2].plot(time, df['force_sensor_y'].values, 'g-', label='Fy (sensor)', alpha=0.7, linewidth=1.5)
    axes[2].plot(time, df['force_sensor_z'].values, 'b-', label='Fz (sensor)', alpha=0.7, linewidth=1.5)
    axes[2].set_ylabel('Force (N)')
    axes[2].set_xlabel('Time (s)')
    axes[2].set_title('Force Components in Sensor Frame')
    axes[2].legend()
    axes[2].grid(True)
    
    plt.tight_layout()
    
    if save_dir:
        plt.savefig(save_dir / 'force_data.png', dpi=300, bbox_inches='tight')
        print(f"Saved: {save_dir / 'force_data.png'}")
    plt.show()


def plot_trajectory(df, save_dir=None):
    """繪製 TCP 軌跡，包含期望軌跡和修正軌跡"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('TCP Trajectory with Desired and Corrected Paths', fontsize=16)
    
    time = df['time_relative'].values  # 轉換為 numpy array
    
    # XY 軌跡 - 主要視圖
    ax = axes[0, 0]
    
    # 實際 TCP 軌跡（粗實線）
    ax.plot(df['tcp_x'].values, df['tcp_y'].values, 'b-', linewidth=2.5, label='Actual TCP', zorder=3)

    # 檢查是否有軌跡資料
    has_desired_traj = 'desired_traj_x' in df.columns and not df['desired_traj_x'].isna().all()
    has_corrected_traj = 'corrected_traj_x' in df.columns and not df['corrected_traj_x'].isna().all()
    
    # 期望軌跡（虛線）
    if has_desired_traj:
        # 過濾掉 0,0 的點（初始化值）
        valid_desired = (df['desired_traj_x'] != 0) | (df['desired_traj_y'] != 0)
        if valid_desired.any():
            ax.plot(df.loc[valid_desired, 'desired_traj_x'].values, 
                   df.loc[valid_desired, 'desired_traj_y'].values, 
                   'g--', linewidth=5.5, label='Desired Trajectory', 
                   alpha=0.7, zorder=2)
            ax.scatter(df.loc[valid_desired, 'desired_traj_x'].values, 
                        df.loc[valid_desired, 'desired_traj_y'].values, 
                        c='orange', s=10, label='Desired trajectory point')
    # 修正軌跡（細鏈線）
    if has_corrected_traj:
        # 過濾掉 0,0 的點
        valid_corrected = (df['corrected_traj_x'] != 0) | (df['corrected_traj_y'] != 0)
        if valid_corrected.any():
            ax.plot(df.loc[valid_corrected, 'corrected_traj_x'].values, 
                   df.loc[valid_corrected, 'corrected_traj_y'].values, 
                   'r-.', linewidth=1.0, label='Corrected Trajectory', 
                   alpha=0.6, zorder=1)
            ax.scatter(df.loc[valid_corrected, 'corrected_traj_x'].values, 
                        df.loc[valid_corrected, 'corrected_traj_y'].values, 
                        c='pink', s=10, label='Corrected trajectory point')

    # 起點和終點
    ax.scatter(df['tcp_x'].iloc[0], df['tcp_y'].iloc[0], 
              c='green', s=150, marker='o', label='Start', zorder=5, edgecolors='black')
    ax.scatter(df['tcp_x'].iloc[-1], df['tcp_y'].iloc[-1], 
              c='red', s=150, marker='s', label='End', zorder=5, edgecolors='black')
    
    ax.set_xlabel('X (mm)', fontsize=12)
    ax.set_ylabel('Y (mm)', fontsize=12)
    ax.set_title('XY Trajectory Comparison', fontsize=13)
    ax.legend(loc='best', fontsize=10)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # X 位置隨時間
    axes[0, 1].plot(time, df['tcp_x'].values, 'b-', linewidth=2, label='Actual')
    if has_desired_traj:
        valid_desired = (df['desired_traj_x'] != 0) | (df['desired_traj_y'] != 0)
        if valid_desired.any():
            axes[0, 1].plot(time[valid_desired], df.loc[valid_desired, 'desired_traj_x'].values, 
                          'g--', linewidth=1.5, label='Desired', alpha=0.7)
            axes[0, 1].scatter(time[valid_desired], df.loc[valid_desired, 'desired_traj_x'].values, 
                          c='orange', s=10, label='Desired trajectory point')
    if has_corrected_traj:
        valid_corrected = (df['corrected_traj_x'] != 0) | (df['corrected_traj_y'] != 0)
        if valid_corrected.any():
            axes[0, 1].plot(time[valid_corrected], df.loc[valid_corrected, 'corrected_traj_x'].values, 
                          'r-.', linewidth=1.8, label='Corrected', alpha=0.5)
            axes[0, 1].scatter(time[valid_corrected], df.loc[valid_corrected, 'corrected_traj_x'].values, 
                          c='pink', s=10, label='Corrected trajectory point')
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('X (mm)')
    axes[0, 1].set_title('X Position vs Time')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Y 位置隨時間
    axes[1, 0].plot(time, df['tcp_y'].values, 'b-', linewidth=2, label='Actual')
    if has_desired_traj:
        valid_desired = (df['desired_traj_x'] != 0) | (df['desired_traj_y'] != 0)
        if valid_desired.any():
            axes[1, 0].plot(time[valid_desired], df.loc[valid_desired, 'desired_traj_y'].values, 
                          'g--', linewidth=1.5, label='Desired', alpha=0.7)
            axes[1, 0].scatter(time[valid_desired], df.loc[valid_desired, 'desired_traj_y'].values, 
                          c='orange', s=10, label='Desired trajectory point')
    if has_corrected_traj:
        valid_corrected = (df['corrected_traj_x'] != 0) | (df['corrected_traj_y'] != 0)
        if valid_corrected.any():
            axes[1, 0].plot(time[valid_corrected], df.loc[valid_corrected, 'corrected_traj_y'].values, 
                          'r-.', linewidth=1.0, label='Corrected', alpha=0.6)
            axes[1, 0].scatter(time[valid_corrected], df.loc[valid_corrected, 'corrected_traj_y'].values, 
                          c='pink', s=10, label='Corrected trajectory point')
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Y (mm)')
    axes[1, 0].set_title('Y Position vs Time')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    
    # Z 位置隨時間
    axes[1, 1].plot(time, df['tcp_z'].values, 'b-', linewidth=2)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Z (mm)')
    axes[1, 1].set_title('Z Position vs Time')
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    if save_dir:
        plt.savefig(save_dir / 'trajectory.png', dpi=300, bbox_inches='tight')
        print(f"Saved: {save_dir / 'trajectory.png'}")
    plt.show()


def plot_force_direction(df, save_dir=None):
    """繪製力的方向（法向和切向）"""
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle('Force Direction Vectors', fontsize=16)
    
    time = df['time_relative'].values  # 轉換為 numpy array
    
    # 法向方向
    axes[0].plot(time, df['normal_direction_x'].values, 'r-', label='Normal X', linewidth=1.5)
    axes[0].plot(time, df['normal_direction_y'].values, 'g-', label='Normal Y', linewidth=1.5)
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Direction (unit vector)')
    axes[0].set_title('Normal Direction Unit Vector')
    axes[0].legend()
    axes[0].grid(True)
    axes[0].set_ylim([-1.5, 1.5])
    
    # 切向方向
    axes[1].plot(time, df['tangential_direction_x'].values, 'r-', label='Tangential X', linewidth=1.5)
    axes[1].plot(time, df['tangential_direction_y'].values, 'g-', label='Tangential Y', linewidth=1.5)
    axes[1].set_xlabel('Time (s)')
    axes[1].set_ylabel('Direction (unit vector)')
    axes[1].set_title('Tangential Direction Unit Vector')
    axes[1].legend()
    axes[1].grid(True)
    axes[1].set_ylim([-1.5, 1.5])
    
    plt.tight_layout()
    
    if save_dir:
        plt.savefig(save_dir / 'force_direction.png', dpi=300, bbox_inches='tight')
        print(f"Saved: {save_dir / 'force_direction.png'}")
    plt.show()

def plot_force_direction_on_trajectory(df, save_dir=None, skip_points=3, arrow_scale=5.0):
    """
    在 TCP 軌跡上直接繪製力的方向箭頭
    
    Parameters:
    -----------
    df : DataFrame
        包含軌跡和力數據的 DataFrame
    save_dir : Path, optional
        保存圖片的目錄
    skip_points : int
        每隔多少個點繪製一次箭頭（避免太密集）
    arrow_scale : float
        箭頭長度縮放因子（mm）
    """
    fig, ax = plt.subplots(figsize=(14, 10))
    fig.suptitle('Force Direction Vectors on TCP Trajectory', fontsize=16)
    
    # 繪製 TCP 軌跡
    ax.plot(df['tcp_x'].values, df['tcp_y'].values, 'b-', linewidth=2.0, 
            label='TCP Trajectory', alpha=0.6, zorder=1)
    
    # 標記起點和終點
    ax.scatter(df['tcp_x'].iloc[0], df['tcp_y'].iloc[0], 
              c='green', s=200, marker='o', label='Start', zorder=5, edgecolors='black', linewidth=2)
    ax.scatter(df['tcp_x'].iloc[-1], df['tcp_y'].iloc[-1], 
              c='red', s=200, marker='s', label='End', zorder=5, edgecolors='black', linewidth=2)
    
    # 檢查是否有法向和切向方向數據
    has_normal = all(col in df.columns for col in ['normal_direction_x', 'normal_direction_y'])
    has_tangential = all(col in df.columns for col in ['tangential_direction_x', 'tangential_direction_y'])
    has_corrected_traj = 'corrected_traj_x' in df.columns and not df['corrected_traj_x'].isna().all()
    
    if not has_normal and not has_tangential:
        print("Warning: No direction data found in DataFrame")
        ax.text(0.5, 0.5, 'No force direction data available', 
                ha='center', va='center', transform=ax.transAxes, fontsize=14)
    else:
        # 每隔 skip_points 個點繪製箭頭
        indices = range(0, len(df), skip_points)
        
        for idx in indices:
            tcp_x = df['tcp_x'].iloc[idx]
            tcp_y = df['tcp_y'].iloc[idx]
            
            # 繪製法向力箭頭（紅色）
            if has_normal:
                normal_x = df['normal_direction_x'].iloc[idx]
                normal_y = df['normal_direction_y'].iloc[idx]
                normal_magnitude = df['normal_force_magnitude'].iloc[idx]
                
                # 只在力量足夠大時繪製箭頭
                if normal_magnitude > 0.5:  # 大於 0.5N 才繪製
                    # 箭頭長度根據力量大小調整
                    arrow_length = arrow_scale * (normal_magnitude / 5.0)  # 假設 5N 為參考值
                    
                    ax.arrow(tcp_x, tcp_y, 
                            normal_x * arrow_length, 
                            normal_y * arrow_length,
                            head_width=0.1, head_length=0.10, 
                            fc='red', ec='darkred', 
                            linewidth=0.5, alpha=0.8, zorder=3,
                            length_includes_head=True)
            
            # 繪製切向箭頭（綠色）
            if has_tangential:
                tangential_x = df['tangential_direction_x'].iloc[idx]
                tangential_y = df['tangential_direction_y'].iloc[idx]
                
                # 計算切向大小
                tangential_magnitude = np.sqrt(
                    (df['force_base_x'].iloc[idx] * tangential_x)**2 + 
                    (df['force_base_y'].iloc[idx] * tangential_y)**2
                )
                
                arrow_length = arrow_scale * (tangential_magnitude / 5.0)
                
                ax.arrow(tcp_x, tcp_y, 
                        tangential_x * arrow_length, 
                        tangential_y * arrow_length,
                        head_width=0.1, head_length=0.10, 
                        fc='green', ec='darkgreen', 
                        linewidth=0.5, alpha=0.8, zorder=3,
                        length_includes_head=True)
                
    # 標記接觸點 (第一次進入 FORCE_CONTROL 的點)
    force_control_rows = df[df['control_mode'] == 'FORCE_CONTROL']
    if not force_control_rows.empty:
        first_idx = force_control_rows.index[0]
        ax.scatter(df['tcp_x'].loc[first_idx], df['tcp_y'].loc[first_idx], 
              c='yellow', s=250, marker='P', label='Contact Point', zorder=6, edgecolors='black', linewidth=2)
        # 繪製法向力箭頭（紅色）
        if has_normal:
            normal_x = df['normal_direction_x'].iloc[idx]
            normal_y = df['normal_direction_y'].iloc[idx]
            normal_magnitude = df['normal_force_magnitude'].iloc[idx]
            
            # 只在力量足夠大時繪製箭頭
            if normal_magnitude > 0.5:  # 大於 0.5N 才繪製
                # 箭頭長度根據力量大小調整
                arrow_length = arrow_scale * (normal_magnitude / 5.0)  # 假設 5N 為參考值
                
                ax.arrow(df['tcp_x'].loc[first_idx], df['tcp_y'].loc[first_idx], 
                        normal_x * arrow_length, 
                        normal_y * arrow_length,
                        head_width=0.1, head_length=0.10, 
                        fc='red', ec='darkred', 
                        linewidth=0.5, alpha=0.8, zorder=3,
                        length_includes_head=True)
        
        # 繪製切向箭頭（綠色）
        if has_tangential:
            tangential_x = df['tangential_direction_x'].iloc[idx]
            tangential_y = df['tangential_direction_y'].iloc[idx]
            
            # 計算切向大小
            tangential_magnitude = np.sqrt(
                (df['force_base_x'].iloc[idx] * tangential_x)**2 + 
                (df['force_base_y'].iloc[idx] * tangential_y)**2
            )
            
            arrow_length = arrow_scale * (tangential_magnitude / 5.0)

            ax.arrow(df['tcp_x'].loc[first_idx], df['tcp_y'].loc[first_idx], 
                    tangential_x * arrow_length, 
                    tangential_y * arrow_length,
                    head_width=0.1, head_length=0.10, 
                    fc='green', ec='darkgreen', 
                    linewidth=0.5, alpha=0.8, zorder=3,
                    length_includes_head=True)

    # 修正軌跡（細鏈線）
    if has_corrected_traj:
        # 過濾掉 0,0 的點
        valid_corrected = (df['corrected_traj_x'] != 0) | (df['corrected_traj_y'] != 0)
        if valid_corrected.any():
            ax.plot(df.loc[valid_corrected, 'corrected_traj_x'].values, 
                df.loc[valid_corrected, 'corrected_traj_y'].values, 
                'r-.', linewidth=1.0, label='Corrected Trajectory', 
                alpha=0.6, zorder=1)
            ax.scatter(df.loc[valid_corrected, 'corrected_traj_x'].values, 
                        df.loc[valid_corrected, 'corrected_traj_y'].values, 
                        c='pink', s=10, label='Corrected trajectory point')
    
    # 添加圖例說明
    
    normal_arrow = FancyArrow(0, 0, 0, 0, color='red', label='Normal Force')
    tangential_arrow = FancyArrow(0, 0, 0, 0, color='green', label='Tangential Direction')
    ax.legend(handles=[ax.lines[0], ax.collections[0], ax.collections[1], 
                      normal_arrow, tangential_arrow], 
             loc='best', fontsize=11)
    
    ax.set_xlabel('X (mm)', fontsize=13)
    ax.set_ylabel('Y (mm)', fontsize=13)
    ax.set_title('Force Direction Vectors at TCP Points', fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.tight_layout()
    
    if save_dir:
        plt.savefig(save_dir / 'force_direction_on_trajectory.png', dpi=300, bbox_inches='tight')
        print(f"Saved: {save_dir / 'force_direction_on_trajectory.png'}")
    plt.show()

def plot_corrected_trajectory(df, save_dir=None):
    """繪製修正後的軌跡"""
    force_control_rows = df[df['control_mode'] == 'FORCE_CONTROL']
    if not force_control_rows.empty:   
        fig, axes = plt.subplots(1, 2, figsize=(14, 10))
        fig.suptitle('Corrected Trajectory', fontsize=16)
        ax1, ax2 = axes

        first_idx = force_control_rows.index[0]
        ax1.scatter(df['tcp_x'].loc[first_idx], df['tcp_y'].loc[first_idx], 
              c='yellow', s=250, marker='P', label='Contact Point', zorder=6, edgecolors='black', linewidth=2)
        # 實際 TCP 軌跡（粗實線）
        ax1.plot(df['tcp_x'].loc[first_idx:].values, df['tcp_y'].loc[first_idx:].values, 'b-', linewidth=2.5, label='Actual TCP', zorder=3)

        # 檢查是否有軌跡資料
        has_desired_traj = 'desired_traj_x' in df.columns and not df['desired_traj_x'].isna().all()
        has_corrected_traj = 'corrected_traj_x' in df.columns and not df['corrected_traj_x'].isna().all()
        offset_idx = 0
        # 期望軌跡（虛線）
        if has_desired_traj:
            # 過濾掉 0,0 的點（初始化值）
            valid_desired = (df['desired_traj_x'] != 0) | (df['desired_traj_y'] != 0)
            if valid_desired.any():
                ax1.plot(df.loc[first_idx-offset_idx:, 'desired_traj_x'].values, 
                    df.loc[first_idx-offset_idx:, 'desired_traj_y'].values, 
                    'g--', linewidth=5.5, label='Desired Trajectory', 
                    alpha=0.7, zorder=2)
                ax1.scatter(df.loc[first_idx-offset_idx:, 'desired_traj_x'].values, 
                            df.loc[first_idx-offset_idx:, 'desired_traj_y'].values, 
                            c='orange', s=10, label='Desired trajectory point')
        # 修正軌跡（細鏈線）
        if has_corrected_traj:
            # 過濾掉 0,0 的點
            valid_corrected = (df['corrected_traj_x'] != 0) | (df['corrected_traj_y'] != 0)
            if valid_corrected.any():
                ax1.plot(df.loc[first_idx:, 'corrected_traj_x'].values, 
                    df.loc[first_idx:, 'corrected_traj_y'].values, 
                    'r-.', linewidth=1.0, label='Corrected Trajectory', 
                    alpha=0.6, zorder=1)
                ax1.scatter(df.loc[first_idx:, 'corrected_traj_x'].values, 
                            df.loc[first_idx:, 'corrected_traj_y'].values, 
                            c='pink', s=10, label='Corrected trajectory point')
        
        # ========== 右圖：視覺化 delta_L1, delta_L2, theta ==========
        # 選擇要視覺化的索引範圍（前幾個點）
        viz_indices = list(range(first_idx, min(first_idx + 1, len(df))))
        
        for idx in viz_indices:
            if idx >= len(df):
                break
                
            # 點 A (當前 TCP 位置)
            A = np.array([df['tcp_x'].loc[idx], df['tcp_y'].loc[idx]])
            # 點 B (期望軌跡點)
            B = np.array([df['desired_traj_x'].loc[idx], df['desired_traj_y'].loc[idx]])
            # 點 C (修正後的軌跡點)
            C = np.array([df['corrected_traj_x'].loc[idx], df['corrected_traj_y'].loc[idx]])

            arrow_scale=0.050
            # 繪製法向力箭頭（紅色）
            normal_x = df['normal_direction_x'].iloc[idx]
            normal_y = df['normal_direction_y'].iloc[idx]
            normal_magnitude = df['normal_force_magnitude'].iloc[idx]
                
                # 只在力量足夠大時繪製箭頭
            if normal_magnitude > 0.5:  # 大於 0.5N 才繪製
                # 箭頭長度根據力量大小調整
                arrow_length = arrow_scale * (normal_magnitude / 1.0)  # 假設 5N 為參考值
                
                ax2.arrow(df['tcp_x'].loc[idx], df['tcp_y'].loc[idx], 
                        normal_x * arrow_length, 
                        normal_y * arrow_length,
                        head_width=0.005, head_length=0.005, 
                        fc='red', ec='darkred', 
                        linewidth=0.005, alpha=0.8, zorder=3,
                        length_includes_head=True)
            
            # 繪製切向箭頭（綠色）
            tangential_x = df['tangential_direction_x'].iloc[idx]
            tangential_y = df['tangential_direction_y'].iloc[idx]
            ax2.text(A[0]-0.05,A[1]-0.05, f'm1:{((B[1]-A[1])/(B[0]-A[0])):.4f}')
            ax2.text(A[0]-0.05,A[1], f'm2:{tangential_y/tangential_x:.4f}')
            ax2.text(A[0]-0.05,A[1]-0.025, f'y:{tangential_y:.4f}, x:{tangential_x:.4f}')
            
                
            # 計算切向大小
            tangential_magnitude = np.sqrt(
                (df['force_base_x'].iloc[idx] * tangential_x)**2 + 
                (df['force_base_y'].iloc[idx] * tangential_y)**2
            )
            
            arrow_length = arrow_scale * (tangential_magnitude / 1.0)

            ax2.arrow(df['tcp_x'].loc[idx], df['tcp_y'].loc[idx], 
                    tangential_x * arrow_length, 
                    tangential_y * arrow_length,
                    head_width=0.005, head_length=0.005, 
                    fc='green', ec='darkgreen', 
                    linewidth=0.005, alpha=0.8, zorder=3,
                    length_includes_head=True)
            
            # 跳過無效點
            if np.any(np.isnan(A)) or np.any(np.isnan(B)) or np.any(np.isnan(C)):
                continue
            if (B[0] == 0 and B[1] == 0) or (C[0] == 0 and C[1] == 0):
                continue
            
            # 繪製點
            ax2.scatter(*A, c='blue', s=200, marker='o', zorder=6, edgecolors='black', linewidth=1.5)
            ax2.text(A[0],A[1], f'({A[0]:.4f}, {A[1]:.4f})', fontsize=9, color='blue', weight='bold',)
            ax2.scatter(*B, c='green', s=200, marker='*', zorder=6, edgecolors='black', linewidth=1.5)
            ax2.text(B[0],B[1], f'({B[0]:.4f}, {B[1]:.4f})', fontsize=9, color='blue', weight='bold',)
            ax2.scatter(*C, c='red', s=200, marker='X', zorder=6, edgecolors='black', linewidth=1.5)
            ax2.text(C[0],C[1], f'({C[0]:.4f}, {C[1]:.4f})', fontsize=9, color='blue', weight='bold',)
            
            # 繪製 delta_L1 (A -> B，藍色實線)
            ax2.plot([A[0], B[0]], [A[1], B[1]], 'b-', linewidth=2, zorder=4)
            delta_L1 = np.linalg.norm(B - A)
            mid_AB = (A + B) / 2
            ax2.text(mid_AB[0], mid_AB[1], f'ΔL1={delta_L1:.4f}mm', 
                    fontsize=9, color='blue', weight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))
            
            # 繪製 delta_L2 (A -> C，紅色實線)
            ax2.plot([A[0], C[0]], [A[1], C[1]], 'r-', linewidth=2, zorder=4)
            delta_L2 = np.linalg.norm(C - A)
            mid_AC = (A + C) / 2
            ax2.text(mid_AC[0], mid_AC[1], f'ΔL2={delta_L2:.4f}mm', 
                    fontsize=9, color='red', weight='bold',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))
            
            # 繪製角度 θ (使用弧線)
            # 計算兩個向量的角度
            vec_AB = B - A
            vec_AC = C - A
            
            # 計算角度（弧度）
            angle_AB = np.arctan2(vec_AB[1], vec_AB[0])
            angle_AC = np.arctan2(vec_AC[1], vec_AC[0])
            theta = np.abs(angle_AB - angle_AC)
            
            # 繪製角度弧線
            if delta_L1 > 0.1:  # 避免太小的向量
                arc_radius = min(delta_L1, delta_L2) * 0.3
                angle_start = np.degrees(min(angle_AB, angle_AC))
                angle_end = np.degrees(max(angle_AB, angle_AC))
                
                arc = plt.matplotlib.patches.Arc(A, 2*arc_radius, 2*arc_radius,
                                                angle=0, theta1=angle_start, theta2=angle_end,
                                                color='purple', linewidth=2, zorder=5)
                ax2.add_patch(arc)
                
                # 標註角度
                mid_angle = (angle_AB + angle_AC) / 2
                text_pos = A + arc_radius * 1.5 * np.array([np.cos(mid_angle), np.sin(mid_angle)])
                ax2.text(text_pos[0], text_pos[1], f'θ={np.degrees(theta):.1f}°', 
                        fontsize=9, color='purple', weight='bold',
                        bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))
            
            # 標註點
            # offset = 2
            # ax2.text(A[0]+offset, A[1]+offset, 'A (TCP)', fontsize=8, color='blue')
            # ax2.text(B[0]+offset, B[1]+offset, 'B (Desired)', fontsize=8, color='green')
            # ax2.text(C[0]+offset, C[1]+offset, 'C (Corrected)', fontsize=8, color='red')
        
        # 添加圖例（只在第一次迭代添加）
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='blue', markersize=10, label='TCP Point (A)'),
            Line2D([0], [0], marker='*', color='w', markerfacecolor='green', markersize=12, label='Desired Point (B)'),
            Line2D([0], [0], marker='X', color='w', markerfacecolor='red', markersize=10, label='Corrected Point (C)'),
            Line2D([0], [0], color='blue', linewidth=2, label='ΔL1 (A→B)'),
            Line2D([0], [0], color='red', linewidth=2, label='ΔL2 (A→C)'),
            Line2D([0], [0], color='purple', linewidth=2, label='θ (angle)')
        ]
        ax2.legend(handles=legend_elements, loc='best', fontsize=9)
                
    ax1.set_xlabel('X (mm)', fontsize=12)
    ax1.set_ylabel('Y (mm)', fontsize=12)
    ax1.set_title('XY Trajectory Comparison', fontsize=13)
    ax1.legend(loc='best', fontsize=10)
    ax1.grid(True, alpha=0.3)
    ax1.axis('equal')

    ax2.set_xlabel('X (mm)', fontsize=12)
    ax2.set_ylabel('Y (mm)', fontsize=12)
    ax2.set_title('Delta_L1, Delta_L2, Theta Visualization', fontsize=13)
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')

    plt.tight_layout()
    if save_dir:
        plt.savefig(save_dir / 'corrected_trajectory.png', dpi=300, bbox_inches='tight')
        print(f"Saved: {save_dir / 'corrected_trajectory.png'}")
    plt.show()

def print_statistics(df):
    """印出統計資訊"""
    print("\n" + "="*60)
    print("DATA STATISTICS")
    print("="*60)
    
    # 時間資訊
    total_time = df['time_relative'].iloc[-1]
    print(f"\nTotal Duration: {total_time:.2f} seconds")
    print(f"Total Samples: {len(df)}")
    print(f"Sampling Rate: {len(df)/total_time:.2f} Hz")
    
    # 力統計（整體）
    print("\n--- Force Statistics (Overall) ---")
    print(f"Normal Force:")
    print(f"  Mean: {df['normal_force_magnitude'].mean():.2f} N")
    print(f"  Std:  {df['normal_force_magnitude'].std():.2f} N")
    print(f"  Max:  {df['normal_force_magnitude'].max():.2f} N")
    print(f"  Min:  {df['normal_force_magnitude'].min():.2f} N")
    
    print(f"\nBase Frame Forces:")
    print(f"  Fx - Mean: {df['force_base_x'].mean():.2f} N, Std: {df['force_base_x'].std():.2f} N")
    print(f"  Fy - Mean: {df['force_base_y'].mean():.2f} N, Std: {df['force_base_y'].std():.2f} N")
    print(f"  Fz - Mean: {df['force_base_z'].mean():.2f} N, Std: {df['force_base_z'].std():.2f} N")
    
    # 軌跡統計
    print("\n--- TCP Movement ---")
    dx = df['tcp_x'].iloc[-1] - df['tcp_x'].iloc[0]
    dy = df['tcp_y'].iloc[-1] - df['tcp_y'].iloc[0]
    dz = df['tcp_z'].iloc[-1] - df['tcp_z'].iloc[0]
    total_distance = np.sqrt(dx**2 + dy**2 + dz**2)
    print(f"  Start Position: ({df['tcp_x'].iloc[0]:.2f}, {df['tcp_y'].iloc[0]:.2f}, {df['tcp_z'].iloc[0]:.2f}) mm")
    print(f"  End Position:   ({df['tcp_x'].iloc[-1]:.2f}, {df['tcp_y'].iloc[-1]:.2f}, {df['tcp_z'].iloc[-1]:.2f}) mm")
    print(f"  ΔX: {dx:.2f} mm")
    print(f"  ΔY: {dy:.2f} mm")
    print(f"  ΔZ: {dz:.2f} mm")
    print(f"  Total Distance: {total_distance:.2f} mm")
    print("\n" + "="*60)


def main():
    parser = argparse.ArgumentParser(
        description='Analyze TM robot force control data',
        epilog='Examples:\n'
               '  python3 %(prog)s --latest\n'
               '  python3 %(prog)s file.csv\n'
               '  python3 %(prog)s file.csv --save\n',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument('csv_file', type=str, nargs='?', 
                       help='Path to CSV log file (optional if using --latest)')
    parser.add_argument('--save', action='store_true', 
                       help='Save plots to file')
    parser.add_argument('--latest', action='store_true', 
                       help='Use the latest CSV file in ~/tm_force_control_logs/')
    parser.add_argument('--list', action='store_true',
                       help='List all available CSV files')
    
    args = parser.parse_args()
    
    log_dir = Path.home() / "tm_force_control_logs"
    
    # 列出所有檔案
    if args.list:
        csv_files = sorted(log_dir.glob("force_control_log_*.csv"), 
                          key=lambda p: p.stat().st_mtime, reverse=True)
        print(f"\nAvailable CSV files in {log_dir}:")
        print("="*60)
        for i, csv_file in enumerate(csv_files, 1):
            file_size = csv_file.stat().st_size / 1024  # KB
            print(f"{i}. {csv_file.name} ({file_size:.1f} KB)")
        print("="*60)
        return
    
    # 決定要使用哪個檔案
    if args.latest or args.csv_file is None:
        csv_files = sorted(log_dir.glob("force_control_log_*.csv"), 
                          key=lambda p: p.stat().st_mtime, reverse=True)
        
        if not csv_files:
            print(f"Error: No CSV files found in {log_dir}")
            print("Tip: Run with --list to see available files")
            return
        
        csv_path = csv_files[0]
        print(f"Using latest file: {csv_path.name}")
    else:
        csv_path = Path(args.csv_file)
        if not csv_path.exists():
            print(f"Error: File not found: {csv_path}")
            return
    
    print(f"Loading data from: {csv_path}")
    df = load_data(csv_path)
    
    # 印出統計
    # print_statistics(df)
    
    # 設置保存目錄
    save_dir = None
    if args.save:
        save_dir = csv_path.parent / f"{csv_path.stem}_plots"
        save_dir.mkdir(exist_ok=True)
        print(f"\nSaving plots to: {save_dir}")
    
    # 繪圖
    print("\nGenerating plots...")
    plot_force_data(df, save_dir)
    plot_trajectory(df, save_dir)
    plot_force_direction_on_trajectory(df, save_dir, skip_points=500, arrow_scale=0.50)
    plot_corrected_trajectory(df, save_dir)
    # plot_force_direction(df, save_dir)
    
    print("\nAnalysis complete!")
    if save_dir:
        print(f"All plots saved to: {save_dir}")


if __name__ == "__main__":
    main()


# #!/usr/bin/env python3
# """
# TM Robot Force Control Data Analysis and Visualization

# 分析和可視化記錄的力控制數據
# 自動處理缺少的欄位
# """

# import pandas as pd
# import matplotlib.pyplot as plt
# import numpy as np
# from pathlib import Path
# import argparse


# def load_data(csv_file):
#     """載入 CSV 數據"""
#     df = pd.read_csv(csv_file)
    
#     # 計算相對時間（從開始的秒數）
#     df['time_relative'] = (df['ros_time_sec'] - df['ros_time_sec'].iloc[0]) + \
#                           (df['ros_time_nsec'] - df['ros_time_nsec'].iloc[0]) / 1e9
    
#     # 如果缺少 control_mode，從力量推斷
#     if 'control_mode' not in df.columns:
#         print("Warning: 'control_mode' column not found, inferring from force data...")
#         # 假設當法向力 > 1.5N 時進入 FORCE_CONTROL
#         df['control_mode'] = 'APPROACH'
#         df.loc[df['normal_force_magnitude'] > 1.5, 'control_mode'] = 'FORCE_CONTROL'
    
#     # 如果缺少 normal_force_error 和 force_integral，設為 NaN
#     if 'normal_force_error' not in df.columns:
#         df['normal_force_error'] = np.nan
#     if 'force_integral' not in df.columns:
#         df['force_integral'] = np.nan
    
#     return df


# def plot_force_data(df, save_dir=None):
#     """繪製力數據"""
#     fig, axes = plt.subplots(3, 1, figsize=(12, 12))
#     fig.suptitle('Force Control Data', fontsize=16)
    
#     time = df['time_relative']
    
#     # 1. 法向力
#     axes[0].plot(time, df['normal_force_magnitude'], 'b-', linewidth=2, label='Normal Force')
#     if 'normal_force_error' in df.columns and not df['normal_force_error'].isna().all():
#         target_force = df['normal_force_magnitude'] - df['normal_force_error']
#         non_zero_target = target_force[target_force != 0]
#         if len(non_zero_target) > 0:
#             axes[0].axhline(y=non_zero_target.iloc[0], color='r', linestyle='--', label='Target Force')
#     axes[0].set_ylabel('Force (N)')
#     axes[0].set_title('Normal Force Magnitude')
#     axes[0].legend()
#     axes[0].grid(True)
    
#     # 2. Base frame 的 XY 力
#     axes[1].plot(time, df['force_base_x'], 'r-', label='Fx (base)', linewidth=1.5)
#     axes[1].plot(time, df['force_base_y'], 'g-', label='Fy (base)', linewidth=1.5)
#     axes[1].set_ylabel('Force (N)')
#     axes[1].set_title('Force Components in Base Frame')
#     axes[1].legend()
#     axes[1].grid(True)
    
#     # 3. Sensor frame 的力
#     axes[2].plot(time, df['force_sensor_x'], 'r-', label='Fx (sensor)', alpha=0.7, linewidth=1.5)
#     axes[2].plot(time, df['force_sensor_y'], 'g-', label='Fy (sensor)', alpha=0.7, linewidth=1.5)
#     axes[2].plot(time, df['force_sensor_z'], 'b-', label='Fz (sensor)', alpha=0.7, linewidth=1.5)
#     axes[2].set_ylabel('Force (N)')
#     axes[2].set_title('Force Components in Sensor Frame')
#     axes[2].legend()
#     axes[2].grid(True)
    
#     # 4. 控制模式
#     # if 'control_mode' in df.columns:
#     #     mode_mapping = {'APPROACH': 0, 'FORCE_CONTROL': 1, 'UNKNOWN': -1}
#     #     mode_numeric = df['control_mode'].map(mode_mapping)
#     #     axes[3].plot(time, mode_numeric, 'k-', linewidth=2)
#     #     axes[3].set_ylabel('Control Mode')
#     #     axes[3].set_xlabel('Time (s)')
#     #     axes[3].set_title('Control Mode')
#     #     axes[3].set_yticks([0, 1])
#     #     axes[3].set_yticklabels(['APPROACH', 'FORCE_CONTROL'])
#     #     axes[3].grid(True)
#     # else:
#     #     axes[3].text(0.5, 0.5, 'Control mode data not available', 
#     #                 ha='center', va='center', transform=axes[3].transAxes)
#     #     axes[3].set_xlabel('Time (s)')
    
#     plt.tight_layout()
    
#     if save_dir:
#         plt.savefig(save_dir / 'force_data.png', dpi=300, bbox_inches='tight')
#         print(f"Saved: {save_dir / 'force_data.png'}")
#     plt.show()


# def plot_trajectory(df, save_dir=None):
#     """繪製 TCP 軌跡"""
#     fig, axes = plt.subplots(2, 2, figsize=(14, 10))
#     fig.suptitle('TCP Trajectory', fontsize=16)
    
#     time = df['time_relative']
    
#     # XY 軌跡
#     axes[0, 0].plot(df['tcp_x'], df['tcp_y'], 'b-', linewidth=2)
#     axes[0, 0].scatter(df['tcp_x'].iloc[0], df['tcp_y'].iloc[0], c='g', s=100, label='Start', zorder=5)
#     axes[0, 0].scatter(df['tcp_x'].iloc[-1], df['tcp_y'].iloc[-1], c='r', s=100, label='End', zorder=5)
#     axes[0, 0].set_xlabel('X (mm)')
#     axes[0, 0].set_ylabel('Y (mm)')
#     axes[0, 0].set_title('XY Trajectory')
#     axes[0, 0].legend()
#     axes[0, 0].grid(True)
#     axes[0, 0].axis('equal')
    
#     # X 位置隨時間
#     axes[0, 1].plot(time, df['tcp_x'], 'r-', linewidth=2)
#     axes[0, 1].set_xlabel('Time (s)')
#     axes[0, 1].set_ylabel('X (mm)')
#     axes[0, 1].set_title('X Position vs Time')
#     axes[0, 1].grid(True)
    
#     # Y 位置隨時間
#     axes[1, 0].plot(time, df['tcp_y'], 'g-', linewidth=2)
#     axes[1, 0].set_xlabel('Time (s)')
#     axes[1, 0].set_ylabel('Y (mm)')
#     axes[1, 0].set_title('Y Position vs Time')
#     axes[1, 0].grid(True)
    
#     # Z 位置隨時間
#     axes[1, 1].plot(time, df['tcp_z'], 'b-', linewidth=2)
#     axes[1, 1].set_xlabel('Time (s)')
#     axes[1, 1].set_ylabel('Z (mm)')
#     axes[1, 1].set_title('Z Position vs Time')
#     axes[1, 1].grid(True)
    
#     plt.tight_layout()
    
#     if save_dir:
#         plt.savefig(save_dir / 'trajectory.png', dpi=300, bbox_inches='tight')
#         print(f"Saved: {save_dir / 'trajectory.png'}")
#     plt.show()


# def plot_velocity(df, save_dir=None):
#     """繪製速度數據"""
#     fig, axes = plt.subplots(2, 1, figsize=(12, 8))
#     fig.suptitle('Velocity Data', fontsize=16)
    
#     time = df['time_relative']
    
#     # 速度指令
#     axes[0].plot(time, df['velocity_cmd_x'], 'r-', label='Vx cmd', linewidth=1.5)
#     axes[0].plot(time, df['velocity_cmd_y'], 'g-', label='Vy cmd', linewidth=1.5)
#     axes[0].set_ylabel('Velocity Command (mm/s)')
#     axes[0].set_title('Velocity Commands')
#     axes[0].legend()
#     axes[0].grid(True)
    
#     # 實際 TCP 速度
#     axes[1].plot(time, df['tcp_vel_x'], 'r-', label='Vx actual', linewidth=1.5)
#     axes[1].plot(time, df['tcp_vel_y'], 'g-', label='Vy actual', linewidth=1.5)
#     axes[1].set_xlabel('Time (s)')
#     axes[1].set_ylabel('TCP Velocity (mm/s)')
#     axes[1].set_title('Actual TCP Velocity')
#     axes[1].legend()
#     axes[1].grid(True)
    
#     plt.tight_layout()
    
#     if save_dir:
#         plt.savefig(save_dir / 'velocity.png', dpi=300, bbox_inches='tight')
#         print(f"Saved: {save_dir / 'velocity.png'}")
#     plt.show()


# def plot_force_direction(df, save_dir=None):
#     """繪製力的方向（法向和切向）"""
#     fig, axes = plt.subplots(1, 2, figsize=(14, 6))
#     fig.suptitle('Force Direction Vectors', fontsize=16)
    
#     time = df['time_relative']
    
#     # 法向方向
#     axes[0].plot(time, df['normal_direction_x'], 'r-', label='Normal X', linewidth=1.5)
#     axes[0].plot(time, df['normal_direction_y'], 'g-', label='Normal Y', linewidth=1.5)
#     axes[0].set_xlabel('Time (s)')
#     axes[0].set_ylabel('Direction (unit vector)')
#     axes[0].set_title('Normal Direction Unit Vector')
#     axes[0].legend()
#     axes[0].grid(True)
#     axes[0].set_ylim([-1.5, 1.5])
    
#     # 切向方向
#     axes[1].plot(time, df['tangential_direction_x'], 'r-', label='Tangential X', linewidth=1.5)
#     axes[1].plot(time, df['tangential_direction_y'], 'g-', label='Tangential Y', linewidth=1.5)
#     axes[1].set_xlabel('Time (s)')
#     axes[1].set_ylabel('Direction (unit vector)')
#     axes[1].set_title('Tangential Direction Unit Vector')
#     axes[1].legend()
#     axes[1].grid(True)
#     axes[1].set_ylim([-1.5, 1.5])
    
#     plt.tight_layout()
    
#     if save_dir:
#         plt.savefig(save_dir / 'force_direction.png', dpi=300, bbox_inches='tight')
#         print(f"Saved: {save_dir / 'force_direction.png'}")
#     plt.show()


# def print_statistics(df):
#     """印出統計資訊"""
#     print("\n" + "="*60)
#     print("DATA STATISTICS")
#     print("="*60)
    
#     # 時間資訊
#     total_time = df['time_relative'].iloc[-1]
#     print(f"\nTotal Duration: {total_time:.2f} seconds")
#     print(f"Total Samples: {len(df)}")
#     print(f"Sampling Rate: {len(df)/total_time:.2f} Hz")
    
#     # 力統計（整體）
#     print("\n--- Force Statistics (Overall) ---")
#     print(f"Normal Force:")
#     print(f"  Mean: {df['normal_force_magnitude'].mean():.2f} N")
#     print(f"  Std:  {df['normal_force_magnitude'].std():.2f} N")
#     print(f"  Max:  {df['normal_force_magnitude'].max():.2f} N")
#     print(f"  Min:  {df['normal_force_magnitude'].min():.2f} N")
    
#     print(f"\nBase Frame Forces:")
#     print(f"  Fx - Mean: {df['force_base_x'].mean():.2f} N, Std: {df['force_base_x'].std():.2f} N")
#     print(f"  Fy - Mean: {df['force_base_y'].mean():.2f} N, Std: {df['force_base_y'].std():.2f} N")
#     print(f"  Fz - Mean: {df['force_base_z'].mean():.2f} N, Std: {df['force_base_z'].std():.2f} N")
    
#     # 力統計（僅 FORCE_CONTROL 模式）
#     # if 'control_mode' in df.columns:
#     #     force_control_data = df[df['control_mode'] == 'FORCE_CONTROL']
#     #     if len(force_control_data) > 0:
#     #         print("\n--- Force Statistics (FORCE_CONTROL mode only) ---")
#     #         print(f"Normal Force:")
#     #         print(f"  Mean: {force_control_data['normal_force_magnitude'].mean():.2f} N")
#     #         print(f"  Std:  {force_control_data['normal_force_magnitude'].std():.2f} N")
#     #         print(f"  Max:  {force_control_data['normal_force_magnitude'].max():.2f} N")
#     #         print(f"  Min:  {force_control_data['normal_force_magnitude'].min():.2f} N")
#     #         print(f"  Samples: {len(force_control_data)}")
    
#     # # 控制模式統計
#     # if 'control_mode' in df.columns:
#     #     print("\n--- Control Mode Duration ---")
#     #     mode_changes = df[df['control_mode'] != df['control_mode'].shift()].index
#     #     for i in range(len(mode_changes)):
#     #         start_idx = mode_changes[i]
#     #         end_idx = mode_changes[i+1] if i+1 < len(mode_changes) else len(df)-1
            
#     #         mode = df.loc[start_idx, 'control_mode']
#     #         duration = df.loc[end_idx, 'time_relative'] - df.loc[start_idx, 'time_relative']
#     #         samples = end_idx - start_idx + 1
#     #         print(f"  {mode}: {duration:.2f} seconds ({samples} samples)")
    
#     # 軌跡統計
#     print("\n--- TCP Movement ---")
#     dx = df['tcp_x'].iloc[-1] - df['tcp_x'].iloc[0]
#     dy = df['tcp_y'].iloc[-1] - df['tcp_y'].iloc[0]
#     dz = df['tcp_z'].iloc[-1] - df['tcp_z'].iloc[0]
#     total_distance = np.sqrt(dx**2 + dy**2 + dz**2)
#     print(f"  Start Position: ({df['tcp_x'].iloc[0]:.2f}, {df['tcp_y'].iloc[0]:.2f}, {df['tcp_z'].iloc[0]:.2f}) mm")
#     print(f"  End Position:   ({df['tcp_x'].iloc[-1]:.2f}, {df['tcp_y'].iloc[-1]:.2f}, {df['tcp_z'].iloc[-1]:.2f}) mm")
#     print(f"  ΔX: {dx:.2f} mm")
#     print(f"  ΔY: {dy:.2f} mm")
#     print(f"  ΔZ: {dz:.2f} mm")
#     print(f"  Total Distance: {total_distance:.2f} mm")
    
#     # 速度統計
#     # print("\n--- Velocity Statistics ---")
#     # print(f"Command Velocity:")
#     # print(f"  Vx - Mean: {df['velocity_cmd_x'].mean():.2f} mm/s, Max: {df['velocity_cmd_x'].abs().max():.2f} mm/s")
#     # print(f"  Vy - Mean: {df['velocity_cmd_y'].mean():.2f} mm/s, Max: {df['velocity_cmd_y'].abs().max():.2f} mm/s")
#     # print(f"Actual TCP Velocity:")
#     # print(f"  Vx - Mean: {df['tcp_vel_x'].mean():.2f} mm/s, Max: {df['tcp_vel_x'].abs().max():.2f} mm/s")
#     # print(f"  Vy - Mean: {df['tcp_vel_y'].mean():.2f} mm/s, Max: {df['tcp_vel_y'].abs().max():.2f} mm/s")
    
#     print("\n" + "="*60)


# def main():
#     parser = argparse.ArgumentParser(
#         description='Analyze TM robot force control data',
#         epilog='Examples:\n'
#                '  python3 %(prog)s --latest\n'
#                '  python3 %(prog)s file.csv\n'
#                '  python3 %(prog)s file.csv --save\n',
#         formatter_class=argparse.RawDescriptionHelpFormatter
#     )
#     parser.add_argument('csv_file', type=str, nargs='?', 
#                        help='Path to CSV log file (optional if using --latest)')
#     parser.add_argument('--save', action='store_true', 
#                        help='Save plots to file')
#     parser.add_argument('--latest', action='store_true', 
#                        help='Use the latest CSV file in ~/tm_force_control_logs/')
#     parser.add_argument('--list', action='store_true',
#                        help='List all available CSV files')
    
#     args = parser.parse_args()
    
#     log_dir = Path.home() / "tm_force_control_logs"
    
#     # 列出所有檔案
#     if args.list:
#         csv_files = sorted(log_dir.glob("force_control_log_*.csv"), 
#                           key=lambda p: p.stat().st_mtime, reverse=True)
#         print(f"\nAvailable CSV files in {log_dir}:")
#         print("="*60)
#         for i, csv_file in enumerate(csv_files, 1):
#             file_size = csv_file.stat().st_size / 1024  # KB
#             print(f"{i}. {csv_file.name} ({file_size:.1f} KB)")
#         print("="*60)
#         return
    
#     # 決定要使用哪個檔案
#     if args.latest or args.csv_file is None:
#         csv_files = sorted(log_dir.glob("force_control_log_*.csv"), 
#                           key=lambda p: p.stat().st_mtime, reverse=True)
        
#         if not csv_files:
#             print(f"Error: No CSV files found in {log_dir}")
#             print("Tip: Run with --list to see available files")
#             return
        
#         csv_path = csv_files[0]
#         print(f"Using latest file: {csv_path.name}")
#     else:
#         csv_path = Path(args.csv_file)
#         if not csv_path.exists():
#             print(f"Error: File not found: {csv_path}")
#             return
    
#     print(f"Loading data from: {csv_path}")
#     df = load_data(csv_path)
    
#     # 印出統計
#     print_statistics(df)
    
#     # 設置保存目錄
#     save_dir = None
#     if args.save:
#         save_dir = csv_path.parent / f"{csv_path.stem}_plots"
#         save_dir.mkdir(exist_ok=True)
#         print(f"\nSaving plots to: {save_dir}")
    
#     # 繪圖
#     print("\nGenerating plots...")
#     plot_force_data(df, save_dir)
#     plot_trajectory(df, save_dir)
#     # plot_velocity(df, save_dir)
#     plot_force_direction(df, save_dir)
    
#     print("\nAnalysis complete!")
#     if save_dir:
#         print(f"All plots saved to: {save_dir}")


# if __name__ == "__main__":
#     main()