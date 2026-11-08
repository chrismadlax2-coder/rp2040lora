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
[ 飛控板 (Omnibus F4SD / Betaflight / ArduPilot) ]
```

---

## 硬體規格與腳位定義

| 設備代號 | 角色 | 預設 COM | RP2040 晶片 Unique ID | 連接目標 | 腳位定義 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **板 A** | **接收機 (RX)** | COM17 | `E663B035975EC02C` | Omnibus F4SD 飛控 (UART6) | **GP0** (TX) $\rightarrow$ FC RX<br>**GP2** $\rightarrow$ BIND 引腳（接地對頻）<br>**GND** $\rightarrow$ FC GND<br>**VSYS** $\rightarrow$ FC 5V |
| **板 B** | **發射機 1 (TX 1)** | COM8 | `E663B0359715412B` | FS-i6X 遙控器 (OpenI6X) | **GP1** (RX Inverted) $\leftarrow$ 手柄 PD5<br>**GP2** $\rightarrow$ BIND 引腳（接地廣播）<br>**GND** $\rightarrow$ 手柄 GND<br>**VSYS** $\leftarrow$ 手柄電源 |
| **板 C** | **發射機 2 (TX 2)** | COM19 | UID: `0x2C843F97` | 備用/多控發射模組 | **GP1** (RX Inverted) $\leftarrow$ 手柄訊號<br>**GP2** $\rightarrow$ BIND 引腳（接地廣播）<br>**GND** / **VSYS** $\leftarrow$ 供電 |

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
5. **全鏈路雙重失控保護體系 (Dual-Layer Failsafe Architecture)**：
   - **發射端保護 (TX Level)**：當手柄斷電或 CRSF 信號中斷超過 100ms，板 B 自動切入安全回退機制，持續以 100Hz 空中廣播安全 CRSF 舵量（油門 MIN 988µs / 姿態 MID 1500µs / 解鎖 DISARM 988µs）。
   - **接收端保護 (RX Level)**：當空中射頻訊號完全中斷超過 100ms，板 A 主動以 50Hz 頻率向飛控注入標準 CRSF 安全舵量。
   - **消除致命飛安隱患**：無論是手柄端掉電還是空中射頻中斷，皆能完美解決 ArduPilot 在接收端斷訊時「凍結最後舵量（Hold Last Position）」的致命問題，立即觸發 `FS_THR_VALUE` 進入 RTL 返航或自動迫降。
6. **硬體配對與 Flash NVM 持久化 (GP2 Hardware Binding & Storage)**：
   - **32-bit 硬體唯一 UID**：發射端自動提取 RP2040 Flash Unique ID 作為專屬發射識別碼。
   - **GP2 一鍵配對模式**：將 GP2 引腳接地（GND），發射端自動在基準頻點（431.5MHz）廣播配對信標；接收端在 GP2 接地或無配對紀錄時自動監聽。
   - **內部 Flash 永久記憶**：配對成功後，接收端自動將 TX UID 寫入 RP2040 內部 SPI Flash 末端扇區，掉電不遺失，開機秒連並嚴格過濾非配對發射機訊號。

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
- **地面站驗證記錄**:
  - **Betaflight Configurator**: 接收機分頁舵面跟隨流暢，無體感延遲，100Hz 滿速刷新。
  - **ArduPilot 4.8.0-dev (Mission Planner 遙控器校準分頁)**:
    - **正常連線**：手柄搖桿即時跟隨無延遲。
    - **板 B 手柄訊號中斷驗證**：手柄 CRSF 訊號中斷後，板 B 自動廣播安全回退舵量，地面站確認油門降至 `987µs`、姿態自動居中 `1500µs`、ARM 鎖定 `987µs`，飛控觸發 `Locate Copter alarm` 警報。
  - **硬體配對與 Flash NVM 持久化驗證 (板 C 發射機 $\rightarrow$ 板 A 接收機)**:
    - **配對觸發**：板 C（TX 2，UID `0x2C843F97`）將 GP2 接地廣播信標（LED 10Hz 快閃）；板 A（RX）檢測到信標後瞬間完成配對並寫入 RP2040 內部 Flash，LED 由快閃轉為等待連線慢閃。
    - **連線鎖定**：板 C 斷開 GP2 恢復 100Hz FHSS 發射，板 A 瞬間鎖定（LED 恆亮），即時除錯確認：
      `[RX] Rate: 100 pkts/s | FHSS: LOCKED 100Hz (Ch 5 435.0MHz) | Bound UID: 0x2C843F97 | RSSI: -7dBm | SNR: 2dB`
    - **斷電記憶**：接收端斷電重啟後無需重新配對，直接從 Flash 載入專屬 UID 秒連。
  - **空域防串擾與非法發射機隔離驗證 (板 B 發射機 vs 板 A 接收機)**:
    - **測試條件**：板 A 內部配對綁定板 C（`0x2C843F97`），板 C 關機；此時開啟未配對的板 B（TX 1，UID `0x2B411597`）於同頻段 100Hz 滿功率發射。
    - **接收端表現**：板 A 檢測到射頻訊號（RSSI -8dBm, SNR 2dB），但因 UID 不符合直接於 RF 層丟棄，即時日誌確認：
      `[RX] Rate: 0 pkts/s | FHSS: SEARCHING (Ch0) | Bound UID: 0x2C843F97 | RSSI: -8dBm | SNR: 2dB`
    - **飛安保護結果**：板 A 維持 2Hz 慢閃搜尋狀態，拒絕鎖定板 B；地面站 Mission Planner 顯示持續維持失控保護安全舵量（油門 987µs、姿態回中 1500µs），徹底防止被非配對發射機誤劫持。
