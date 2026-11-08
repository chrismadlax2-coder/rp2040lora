# Waveshare RP2040-LoRa (LF 433MHz) 設備與角色清單

本專案使用兩片 Waveshare RP2040-LoRa (LF, 410~525 MHz) 作為 CRSF 遙控發射與接收系統。

## 硬體唯一編號 (RP2040 Unique Flash ID / USB Serial)

| 設備代號 | 角色 | 韌體檔案 | RP2040 晶片唯一編號 (Serial ID) | 預設 COM 埠 | 說明 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **板子 A** | **接收機 (RX)** | `build/examples/crsf-rc/crsf_rx.uf2` | `E663B035975EC02C` | COM17 | 連接遙控模型飛控板 (GPIO0 -> FC RX) |
| **板子 B** | **發射機 (TX)** | `build/examples/crsf-rc/crsf_tx.uf2` | `E663B0359715412B` | COM8 | 連接遙控器發射模組 (Radio TX -> GPIO1) |

---

## 腳位定義 (Header P1)

### 板子 A - 接收機 (RX: `E663B035975EC02C`)
- **GPIO 0 (Pin 1)**: UART0 TX (416,666 bps) -> 接飛控 CRSF RX
- **GND (Pin 3)**: 接飛控 GND
- **VSYS (Pin 31)**: 接飛控 5V 供電 (3.6V ~ 5.5V)

### 板子 B - 發射機 (TX: `E663B0359715412B`)
- **GPIO 1 (Pin 2)**: UART0 RX (416,666 bps) -> 接遙控器 CRSF TX / S.PORT
- **GND (Pin 3)**: 接遙控器模組槽 GND
- **VSYS (Pin 31) / VBUS (Pin 32)**: 接遙控器 5V 電源或電池

---

## LoRa 射頻參數 (方案 A: SF5 100Hz + FHSS 自動跳頻)
- **跳頻機制**: FHSS (Frequency Hopping Spread Spectrum) 8 頻點偽隨機交錯跳躍
- **跳頻表 (8 頻點)**:
  - Ch 0: `431.5 MHz` (同步搜尋基準頻點)
  - Ch 1: `433.5 MHz`
  - Ch 2: `432.0 MHz`
  - Ch 3: `434.5 MHz`
  - Ch 4: `432.5 MHz`
  - Ch 5: `435.0 MHz`
  - Ch 6: `431.0 MHz`
  - Ch 7: `433.0 MHz`
- **相鄰跳頻間距**: $\ge 1.5\text{ MHz}$ (大幅提升抗窄頻干擾能力)
- **同步機制**: 時鐘飛輪定時器（Flywheel）+ 80ms 快速捕獲
- **防串擾**: 專屬網路 UID (`0xA0`)
- **帶寬 (BW)**: `500 kHz`
- **擴頻因子 (SF)**: `SF5` (Time-on-Air 5.46ms)
- **編碼率 (CR)**: `4/5`
- **發射功率**: `22 dBm`
- **目標更新率**: `100 Hz` (每 10ms 跳一頻並傳送一包)
- **解耦架構**: 發射端手柄 UART（250Hz）與 100Hz 射頻發射完全解耦；接收端採用即時槽換頻（Immediate Slot Advance）與微秒級 `Radio.Standby()` 控制
- **雙重失控保護**:
  - 發射端（板 B）：手柄 CRSF 斷訊超時（>100ms）自動廣播安全回退幀（油門 988µs，姿態 1500µs 中立，ARM 988µs 鎖定）
  - 接收端（板 A）：射頻斷訊超時（>100ms）主動以 50Hz 持續注入安全 CRSF 舵量至飛控
- **驗證狀態**: 2026-11-08 實機全鏈路驗證通過（Betaflight 100Hz 無延遲；ArduPilot 雙重失控保護與斷訊模擬全數通過）
