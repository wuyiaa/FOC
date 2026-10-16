# 电压采集实施记录

## 已确认范围

- ADC2 规则组连续扫描 + 循环 DMA 采集三相端电压。
- ADC2 注入组由 TIM1 CH4 下降沿触发，采集母线电压。
- 采样阶段只提供观测值，限幅及 SVPWM 继续使用固定 24 V。
- FOC_CURRENT_OUTPUT_UTILIZATION 改为初值 0.95 的 Debug 可调变量。
- 不在本次新增 dq 矢量限幅或更改现有 PI 抗饱和算法。

## 实施步骤

- [x] 核对 v1.2 原理图、ADC2 引脚及规则组顺序。
- [x] 新增 User/bsp/voltage_sample.c/.h，保留原始计数、伏特值和更新状态。
- [x] 启动前校准 ADC2、启动规则 DMA 和外部触发注入转换。
- [x] ADC1 注入完成回调检查 ADC2 JEOS，并读取最新完成的 DMA 扫描。
- [x] 将裕量系数改为变量，变更时刷新两轴 PID 数组的输出及积分限幅。
- [x] 将新文件加入 Keil 工程。

## 通道和换算

| 信号 | 引脚 | ADC2 通道 | 采样路径 |
|---|---|---|---|
| VA | PC0 | IN6 | 规则 Rank 2 |
| VB | PC1 | IN7 | 规则 Rank 3 |
| VC | PC2 | IN8 | 规则 Rank 4 |
| VBUS | PC5 | IN11 | 注入 Rank 1 |

规则 Rank 1 的 ADSPE、Rank 5 的 NTC2 保持原配置。DMA 缓冲区保存两次完整的五通道扫描，读取非写入中的一半，并检查读取期间是否跨越边界。三相是顺序采样，不是同时采样，不与 PWM 固定位置同步。

换算：电压 = 原始计数 × 3.3 / 4096 × 16。四路均无软件滤波。

## Keil 观察和后续人工核对

- voltage_sample.va_v / vb_v / vc_v：物理 MA/MB/MC 端子对地电压，单位 V，不随软件换相交换。
- voltage_sample.vbus_v：母线电压，单位 V。
- voltage_sample.va_raw / vb_raw / vc_raw / vbus_raw：原始 ADC 计数。
- phase_valid / bus_valid：至少有过有效发布值，ADC 报错时清零。
- bus_fresh：本次电流中断是否读到了新母线采样。
- phase_update_count / bus_update_count：发布次数；不是所有 ADC 转换次数。
- bus_not_ready_count：回调中未检测到母线转换完成的次数。
- adc_error_code：正常为 0。

24 V 理想母线电压对应约 1862 个计数，ADC 引脚电压约 1.5 V。实际电压应与万用表及实测参考电压对照。端电压包含 PWM，瞬时值不等于一个 PWM 周期的平均值。

FOC_CURRENT_OUTPUT_UTILIZATION 接受 0 到 1 的有限数值；非法值不更新已生效的限幅。修改后在下一次有效电流 PID 更新时刷新两轴参数 [3]、[4]，仍以固定 24 V 计算。直接修改这些数组项会保留到下次裕量变量变化。默认限幅约 13.164 V。

当前 foc_ctrl.c 中的固定 ud/uq 赋值会覆盖 PI 输出，因此仅改变 PID 限幅不一定改变实际开环输出。本次保留该用户调试代码。

硬件采样值正确性由用户实测确认后，再接入实时母线限幅。
