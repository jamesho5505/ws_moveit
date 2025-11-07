import pandas as pd
import numpy as np

# 讀取 CSV 檔案
# df = pd.read_csv('force_pair_log.csv')
df = pd.read_csv('loadcellfsr9.csv')  
print(f"總數據筆數: {len(df)}")

# 過濾有效數據（排除 nan 和 null 值）
valid_df = df.dropna(subset=['fsr_total_g', 'loadcell_g'])
print(f"有效數據筆數: {len(valid_df)}")

# 提取實際值和預測值
actual = valid_df['loadcell_g'].values
predicted = valid_df['fsr_total_g'].values

# 計算誤差
errors = predicted - actual
absolute_errors = np.abs(errors)

# 計算 MAE (Mean Absolute Error)
mae = np.mean(absolute_errors)

# 計算 RMSE (Root Mean Square Error)
mse = np.mean(errors ** 2)
rmse = np.sqrt(mse)

# 計算 MAPE (Mean Absolute Percentage Error)
# 排除實際值為 0 的數據點，避免除以零
non_zero_mask = actual != 0
if np.sum(non_zero_mask) > 0:
    mape = np.mean(np.abs((predicted[non_zero_mask] - actual[non_zero_mask]) / actual[non_zero_mask])) * 100
    mape_count = np.sum(non_zero_mask)
else:
    mape = np.nan
    mape_count = 0

# 輸出結果
print("\n" + "="*50)
print("誤差指標")
print("="*50)
print(f"MAE (平均絕對誤差):        {mae:.4f} g")
print(f"RMSE (均方根誤差):         {rmse:.4f} g")
print(f"MAPE (平均絕對百分比誤差): {mape:.2f} %")

print("\n" + "="*50)
print("額外統計")
print("="*50)
print(f"用於 MAPE 計算的數據筆數:  {mape_count}")
print(f"最大絕對誤差:              {np.max(absolute_errors):.4f} g")
print(f"最小絕對誤差:              {np.min(absolute_errors):.4f} g")
print(f"誤差標準差:                {np.std(errors):.4f} g")
print(f"平均誤差 (偏差):           {np.mean(errors):.4f} g")

# 可選：繪製誤差分布圖
try:
    import matplotlib.pyplot as plt
    
    plt.figure(figsize=(12, 4))
    
    # 誤差分布直方圖
    plt.subplot(1, 3, 1)
    plt.hist(errors, bins=50, edgecolor='black', alpha=0.7)
    plt.xlabel('error (g)')
    plt.ylabel('frequency')
    plt.title('distribution of errors')
    plt.grid(True, alpha=0.3)
    
    # 預測值 vs 實際值
    plt.subplot(1, 3, 2)
    plt.scatter(actual, predicted, alpha=0.3, s=1)
    plt.plot([actual.min(), actual.max()], [actual.min(), actual.max()], 'r--', lw=2)
    plt.xlabel('Load Cell (actual) (g)')
    plt.ylabel('FSR (predicted) (g)')
    plt.title('predicted vs actual')
    plt.grid(True, alpha=0.3)
    
    # 絕對誤差隨時間變化
    plt.subplot(1, 3, 3)
    plt.plot(valid_df['t_s'].values, absolute_errors, alpha=0.5, linewidth=0.5)
    plt.xlabel('time (s)')
    plt.ylabel('absolute error (g)')
    plt.title('absolute error over time')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('error_analysis.png', dpi=300, bbox_inches='tight')
    print("\n圖表已儲存為 'error_analysis.png'")
    plt.show()
    
except ImportError:
    print("\n註: 若要繪製圖表，請安裝 matplotlib: pip install matplotlib")