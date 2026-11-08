# lora simple RC_SYSTEM

基於 Raspberry Pi RP2040 與 Semtech SX1262 LoRa 射頻模組（Waveshare RP2040-LoRa 410~525MHz 版）的開源高性能 CRSF 遙控發射機與接收機系統。

本專案實現了標準 TBS Crossfire (CRSF) 協議解析、8 頻點偽隨機自動跳頻擴頻（FHSS）、100Hz 硬體定時器解耦架構，在 Betaflight 飛控實測達到滿速 100 pkts/s 零體感延遲。

---

## 系統架構圖

```
[ 遙控手柄 (FlySky FS-i6X) ]
           │ OpenI6X 韌體 (400,000 bps, 反向 UART)
           ▼
[ 板 B: RP2040-LoRa 發射機 (TX) ] ── (晶片 ID: E663B0359715412B)
           │ 100Hz 嚴格定時器 (10ms 間隔)
           │ LoRa 433MHz FHSS 8 頻點跳躍 (SF5, BW 500kHz, 22dBm)
           ▼
[ 板 A: RP2040-LoRa 接收機 (RX) ] ── (晶片 ID: E663B035975EC02C)
           │ 即時換頻 (Immediate Slot Advance) + 時鐘飛輪
           │ 標準 CRSF 輸出 (416,666 bps)
           ▼
[ 飛控板 (Omnibus F4SD / Betaflight) ]
```

---

## 硬體規格與腳位定義

| 設備代號 | 角色 | 預設 COM | RP2040 晶片 Unique ID | 連接目標 | 腳位定義 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **板 A** | **接收機 (RX)** | COM17 | `E663B035975EC02C` | Omnibus F4SD 飛控 (UART6) | **GP0** (TX) $\rightarrow$ FC RX<br>**GND** $\rightarrow$ FC GND<br>**VSYS** $\rightarrow$ FC 5V |
| **板 B** | **發射機 (TX)** | COM8 | `E663B0359715412B` | FS-i6X 遙控器 (OpenI6X) | **GP1** (RX Inverted) $\leftarrow$ 手柄 PD5 (USART2_TX)<br>**GND** $\rightarrow$ 手柄 GND<br>**VSYS** $\leftarrow$ 手柄電源 |

---

## 射頻與跳頻參數 (FHSS)

- **通訊頻段**: 433 MHz 免授權 ISM 頻段
- **跳頻機制**: FHSS (Frequency Hopping Spread Spectrum) 8 頻點偽隨機交錯跳躍
- **跳頻表 (8 頻點)**:
  - **Ch 0**: `431.5 MHz` (同步基準頻點)
  - **Ch 1**: `433.5 MHz`
  - **Ch 2**: `432.0 MHz`
  - **Ch 3**: `434.5 MHz`
  - **Ch 4**: `432.5 MHz`
  - **Ch 5**: `435.0 MHz`
  - **Ch 6**: `431.0 MHz`
  - **Ch 7**: `433.0 MHz`
  - *相鄰跳頻間距 $\ge 1.5\text{ MHz}$，具備優異抗窄頻干擾能力。*
- **防串擾網路識別**: Network UID `0xA0`
- **調制參數**:
  - 帶寬 (Bandwidth): `500 kHz`
  - 擴頻因子 (Spreading Factor): `SF5` (空中時間約 5.39ms)
  - 編碼率 (Coding Rate): `4/5`
  - 前導碼 (Preamble): `12 symbols`
  - 發射功率: `22 dBm`
- **目標更新率**: 嚴格 **100.0 Hz**（每 10,000 µs 跳一頻並傳送一包）

---

## 關鍵核心技術

1. **射頻定時器與手柄解耦架構**：
   - OpenTX / OpenI6X 手柄以 250Hz 高速注入搖桿數據，發射機在背景即時快取最新舵量。
   - 射頻發射改由獨立 10,000 µs 硬體定時器驅動，保證發射端跳頻相位精準穩定。
2. **接收端即時槽位切換 (Immediate Slot Advance)**：
   - 接收機在校驗並輸出 CRSF 封包後，於 100µs 內即時切換至下一個頻點，擁有高達 4.5ms 的安全時間窗口等待下一包前導碼抵達。
3. **零延遲 Standby 模式**：
   - 消除 Semtech 官方 `Radio.Sleep()` 的 2ms 延遲阻塞，全面採用微秒級 `Radio.Standby()` 完成頻率重設。
4. **時鐘飛輪 (Clock Flywheel)**：
   - 當單一頻點遭遇突發性空中雜訊時，飛輪定時器（12.5ms 超時）自動強制跳頻，維持與發射端的同步步調。

---

## 編譯與燒錄

### 編譯環境需求
- CMake $\ge 3.20$
- Ninja
- ARM GNU Toolchain (`arm-none-eabi-gcc`)
- Pico SDK (`PICO_SDK_PATH`)

### 編譯指令
```bash
cd RP2040-LoRa-TestCode
cmake -B build -G Ninja
ninja -C build
```
編譯完成後於 `build/examples/crsf-rc/` 產生：
- `crsf_tx.uf2` (發射機韌體)
- `crsf_rx.uf2` (接收機韌體)

### 燒錄方式
支援 RP2040 1200-baud Touch Reboot 自動重啟進入 UF2 開機磁碟機：
```powershell
# 例如透過 COM 埠進入開機磁碟機並複製韌體
$p = New-Object System.IO.Ports.SerialPort "COM17", 1200
$p.DtrEnable = $true; $p.Open(); Start-Sleep -Milliseconds 200; $p.Close()
Copy-Item build/examples/crsf-rc/crsf_rx.uf2 I:\
```

---

## 實測驗證記錄

- **實測日期**: 2026-11-08
- **發射機輸出**: `Rate: 100 pkts/s | FHSS: HOPPING (Ch X) | Mode: HANDSET UART | GP1: ~7100 B/s (Valid: 250)`
- **接收機輸出**: `Rate: 100 pkts/s | FHSS: LOCKED 100Hz | RSSI: -4~-5 dBm | SNR: 2 dB`
- **地面站驗證**: Betaflight Configurator 接收機分頁舵面跟隨流暢，無體感延遲，Failsafe 警告完全解除。
