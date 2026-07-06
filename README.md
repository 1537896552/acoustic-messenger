# 声语信使 - 电子系统课程设计

基于 **4FSK 音频调制** 的数字短信息通信系统，通过声波实现两台 STM32 终端之间的数据传输。

## 项目信息

| 项目 | 详情 |
|------|------|
| 课程 | 电子系统课程设计 |
| 主控 | STM32F103C8T6 |
| 调制方式 | 4FSK（1500/1600/1700/1800 Hz） |
| 通信距离 | ≥ 0.5 米 |
| 码元时长 | 100 ms |

## 硬件架构

- **显示**：0.96" I2C OLED（PB6/PB7）
- **输入**：4x4 矩阵键盘（PA0-PA7）
- **发射**：PA8 PWM → N-MOS 低边驱动 → 扬声器
- **接收**：驻极体话筒 → MCP6002 运放放大 → PB0 ADC → Goertzel 检测
- **电源**：P-MOS + N-MOS 一键软开关，关机待机 < 1 mA

## 目录结构

```
├── materials/
│   ├── c/
│   │   ├── fsk4_rx_oled_single_file.c    # 接收端源代码（OLED版）
│   │   └── fsk4_tx_single_file(1).c      # 发送端源代码
│   ├── fsk4_tx.hex                       # 4FSK发送端固件
│   ├── fsk4_rx.hex                       # 4FSK接收端固件（无屏）
│   ├── fsk4_rx_oled.hex                  # 4FSK接收端固件（带OLED）
│   ├── oled_i2c_bringup_test.hex         # OLED显示测试固件
│   ├── stm32-4fsk-audio.zip              # 4FSK音频通信代码工程
│   ├── dianshe.zip                       # 电设项目压缩包
│   └── *.docx                            # 设计方案、工作日志
├── team67-media/
│   └── 通67组电设.mp4 / 封面.jpg         # 项目展示视频
└── README.md
```

## 快速开始

详见 `materials/` 中的设计方案文档及 HEX 固件文件。
