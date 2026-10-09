# DNESP32S3 外设引脚定义（LCD_SPI / 摄像头 / SD 卡）

- **开发板**：正点原子 DNESP32S3（ATK_DNESP32S3 V1.2）
- **主控**：ESP32-S3-WROOM-1（原理图 U1）
- **资料来源**：
  - 原理图 `ATK_DNESP32S3_V1.2.pdf`
    - `CORE.SchDoc`（Page1，ESP32 模组 / IO 功能选择）
    - `DEVICE1.SchDoc`（Page2，TF 卡 / 摄像头 / SPI LCD / RGBLCD / 传感器）
    - `DEVICE2.SchDoc`（Page3，电源 / USB / XL9555 / 音频）
  - `DNESP32-S3 IO引脚分配表.xlsx`
- 说明：全板使用的 SPI2 总线（`SPI_MOSI=IO11 / SPI_SCK=IO12 / SPI_MISO=IO13`）被 **SPI LCD 与 TF 卡共用**；`OV_SCL/OV_SDA` 与触摸 `CT_SCL/CT_SDA` 共用。部分控制脚（复位、电源、背光、摄像头 PWDN/RESET）由 I2C 扩展芯片 **XL9555**（U8）控制。

---

## 1. SPI LCD（LCD_SPI）引脚定义

### 1.1 与 ESP32-S3 直接相连的信号

| 信号 | ESP32 GPIO | 引脚编号 | 说明 |
|---|---|---|---|
| `LCD_SPI_MOSI`（SDA） | **IO11** | 19 | SPI2 MOSI，与 TF 卡共用 |
| `LCD_SPI_SCK`（SCL）  | **IO12** | 20 | SPI2 SCK，与 TF 卡共用 |
| `LCD_SPI_MISO`        | **IO13** | 21 | SPI2 MISO，SPI LCD 通常不用，与 TF 卡共用 |
| `LCD_CS` / `SLCD_CS`  | **IO21** | 23 | SPI LCD 片选（同时是 RGBLCD 的 R6） |
| `LCD_DC`              | **IO40** | 33 | 数据/命令选择（同时是触摸 CT_INT） |
| `LCD_BL`（背光）      | XL9555 **P1.0** | U8 pin13 | 由 I2C 扩展 IO 控制 |
| `LCD_RST` / `SLCD_RST`| XL9555 **P1.2** | U8 pin15 | 由 I2C 扩展 IO 控制 |
| `LCD_PWR` / `SLCD_PWR`| XL9555 **P1.3** | U8 pin16 | 由 I2C 扩展 IO 控制 |

> 注意：
> - IO21 在 RGBLCD 模式下是 `LCD_R6`，在 SPILCD 模式下是片选 `LCD_CS`；IO40 同时是触摸中断 `CT_INT` 与 SPILCD 的 `LCD_DC`。
> - Reset、Power、Backlight 三根线没有直接接到 ESP32 GPIO，需要通过 XL9555 拉高/拉低（详见第 4 节）。

### 1.2 SPI LCD / WIRELESS 复用插座 U2（8 pin）

板上 8 pin 插座 `U2` 标注为 **SPI LCD(WIRELESS)**，可插 SPI LCD 模块或无线模块（如 nRF24L01）。其网络连接：

| U2 引脚 | 插座丝印名 | 板上网络 | 对应 ESP32 |
|---|---|---|---|
| 1 | GND | GND | — |
| 2 | VCC | VCC3.3 | 3V3 |
| 3 | CE  | `SLCD_PWR` | XL9555 P1.3 |
| 4 | CSN | `SLCD_CS`  | IO21 |
| 5 | SCK | `SPI_SCK`  | IO12 |
| 6 | MOSI| `SPI_MOSI` | IO11 |
| 7 | MISO| `IO_SEL`   | 经 P5 选择（见下） |
| 8 | IRQ | `SLCD_RST` | XL9555 P1.2 |

### 1.3 IO 功能选择排针 P5（2×3，SPI LCD SELECT）

| P5 引脚 | 网络 | 说明 |
|---|---|---|
| 1 | `IIC_INT` | IO0（XL9555 INT） |
| 2 | `SPI_MISO` | IO13 |
| 3 | `BOOT` | IO0（BOOT 按键） |
| 4 | `IO_SEL` | 连接到 U2.7 |
| 5 | `1WIRE_DQ` | IO0（DHT11/DS18B20） |
| 6 | `LCD_DC` | IO40 |

---

## 2. 摄像头（OV2640，8-bit DVP）引脚定义

摄像头接口为 18 pin（原理图 P2，标注 OV2640）。数据线 `OV_D0~D7` 与 PCLK/HREF/VSYNC 与 RGBLCD 的 R/B/G 线复用。

### 2.1 与 ESP32-S3 直接相连的信号

| 信号 | ESP32 GPIO | 引脚编号 | 说明 |
|---|---|---|---|
| `OV_D0`    | **IO4**  | 4  | 数据位 0（同时是 RGBLCD 的 LCD_DE） |
| `OV_D1`    | **IO5**  | 5  | 数据位 1（LCD_CLK） |
| `OV_D2`    | **IO6**  | 6  | 数据位 2（LCD_B7） |
| `OV_D3`    | **IO7**  | 7  | 数据位 3（LCD_B6） |
| `OV_D4`    | **IO15** | 8  | 数据位 4（LCD_B5） |
| `OV_D5`    | **IO16** | 9  | 数据位 5（LCD_B4） |
| `OV_D6`    | **IO17** | 10 | 数据位 6（LCD_B3） |
| `OV_D7`    | **IO18** | 11 | 数据位 7（LCD_G7） |
| `OV_PCLK`  | **IO45** | 26 | 像素时钟（LCD_R3） |
| `OV_HREF`  | **IO48** | 25 | 行同步（LCD_R4） |
| `OV_VSYNC` | **IO47** | 24 | 场同步（LCD_R5） |
| `OV_SCL`（SCCB SCL） | **IO38** | 31 | 与触摸 `CT_SCL` 共用 |
| `OV_SDA`（SCCB SDA） | **IO39** | 32 | 与触摸 `CT_SDA` 共用 |
| `OV_RESET` | XL9555 **P0.5** | U8 pin9 | 由 I2C 扩展 IO 控制 |
| `OV_PWDN`  | XL9555 **P0.4** | U8 pin8 | 由 I2C 扩展 IO 控制 |

> `OV_SCL`/`OV_SDA` 上各有 4.7K 上拉（R2/R3）到 VCC3.3；摄像头接口附近有 104 去耦电容（C4）。

### 2.2 摄像头连接器 P2 引脚 ↔ 信号（按原理图）

| P2 引脚 | 信号 | P2 引脚 | 信号 |
|---|---|---|---|
| 1  | `OV_VSYNC` | 10 | `OV_D4` |
| 2  | `OV_SCL`   | 11 | `OV_D5` |
| 3  | `OV_HREF`  | 12 | `OV_D6` |
| 4  | `OV_SDA`   | 13 | `OV_D7` |
| 5  | `OV_RESET` | 14 | `OV_PCLK` |
| 6  | `OV_D0`    | 15 | 电源/地（原理图未以网络标签显示） |
| 7  | `OV_D1`    | 16 | `OV_PWDN` |
| 8  | `OV_D2`    | 17 | 电源/地（原理图未以网络标签显示） |
| 9  | `OV_D3`    | 18 | 电源/地（原理图未以网络标签显示） |

---

## 3. SD 卡（TF 卡，SPI 模式）引脚定义

TF 卡座为 J2，工作在 SPI 模式，同样挂在 SPI2 总线上。

| 信号 | ESP32 GPIO | 引脚编号 | TF 卡座引脚 / 名称 | 说明 |
|---|---|---|---|---|
| `TF_CS`    | **IO2**  | 38 | J2 pin2 / DATA3 | 片选，47K 上拉（R15） |
| `SPI_MOSI` | **IO11** | 19 | J2 pin3 / CMD   | 数据输入，47K 上拉（R16） |
| `SPI_SCK`  | **IO12** | 20 | J2 pin5 / CLK   | 时钟 |
| `SPI_MISO` | **IO13** | 21 | J2 pin7 / DATA0 | 数据输出，47K 上拉（R17） |
| 电源 | 3V3 | — | J2 pin4 / VDD | — |
| 地   | GND | — | J2 pin6、pin10~13 / VSS、GND | — |

> J2 其余引脚：pin1(DATA2)、pin8(DATA1)、pin9(CD#) 在原理图中按 SPI 模式作为悬空/固定处理，SPI 通信只需 CS/MOSI/SCK/MISO 四根线。

---

## 4. I2C 扩展芯片 XL9555（U8）相关控制脚

- I2C 地址：由 A0/A1/A2 全接地决定，即 **0x20**（7 位）
- 总线：`IIC_SCL = IO42`、`IIC_SDA = IO41`，中断 `IIC_INT = IO0`
- 与 LCD / 摄像头 / 触摸相关的输出脚：

| XL9555 引脚 | P 口 | 连接网络 | 功能 |
|---|---|---|---|
| pin13 | P1.0 | `LCD_BL`   | LCD 背光 |
| pin14 | P1.1 | `CT_RST`   | 触摸复位 |
| pin15 | P1.2 | `SLCD_RST` | SPI LCD 复位 |
| pin16 | P1.3 | `SLCD_PWR` | SPI LCD 电源 |
| pin9  | P0.5 | `OV_RESET` | 摄像头复位 |
| pin8  | P0.4 | `OV_PWDN`  | 摄像头掉电 |
| pin4  | P0.0 | `AP_INT`   | 光线/接近传感器中断（输入） |
| pin5  | P0.1 | `QMA_INT`  | 加速度计中断（输入） |

---

## 5. 总线/复用注意事项

1. **SPI2 共用**：`IO11/IO12/IO13` 同时接 SPI LCD 与 TF 卡，同一时刻只能选择一个从设备（各自独立的 CS：LCD=IO21，TF=IO2）。
2. **RGBLCD / 摄像头复用**：`OV_D0~D7`、`OV_PCLK/HREF/VSYNC` 与 RGBLCD 的 R/G/B、DE、CLK 复用，不能同时驱动 RGB 屏和摄像头。
3. **SCCB 与触摸共用**：`IO38/IO39` 既是摄像头 SCCB，也是电容触摸 I2C（CT_SCL/SDA），使用时注意设备地址冲突。
4. **扩展 IO**：LCD 的 RST/PWR/BL 以及摄像头 RST/PWDN 均需先初始化 XL9555 才能操作。

---

## 6. 参考宏定义（Arduino / ESP-IDF 风格）

```cpp
/* ---------- SPI 总线（SPI LCD 与 TF 卡共用） ---------- */
#define PIN_SPI_MOSI      11
#define PIN_SPI_SCK       12
#define PIN_SPI_MISO      13

/* ---------- SPI LCD ---------- */
#define PIN_LCD_CS        21   // SLCD_CS
#define PIN_LCD_DC        40   // LCD_DC
#define PIN_LCD_SCK       PIN_SPI_SCK
#define PIN_LCD_MOSI      PIN_SPI_MOSI
#define PIN_LCD_MISO      PIN_SPI_MISO
// LCD_BL / LCD_RST / LCD_PWR 走 XL9555（0x20）：BL=P1.0, RST=P1.2, PWR=P1.3

/* ---------- 摄像头 OV2640 (DVP) ---------- */
#define CAM_PIN_D0        4
#define CAM_PIN_D1        5
#define CAM_PIN_D2        6
#define CAM_PIN_D3        7
#define CAM_PIN_D4        15
#define CAM_PIN_D5        16
#define CAM_PIN_D6        17
#define CAM_PIN_D7        18
#define CAM_PIN_XCLK      -1   // 板上由 LEDC/XCLK 提供，见实际模块
#define CAM_PIN_PCLK      45
#define CAM_PIN_VSYNC     47
#define CAM_PIN_HREF      48
#define CAM_PIN_SIOD      39   // OV_SDA (SCCB)
#define CAM_PIN_SIOC      38   // OV_SCL (SCCB)
// OV_RESET = XL9555 P0.5, OV_PWDN = XL9555 P0.4

/* ---------- TF 卡 (SPI) ---------- */
#define SD_CS             2    // TF_CS
#define SD_SCK            PIN_SPI_SCK
#define SD_MOSI           PIN_SPI_MOSI
#define SD_MISO           PIN_SPI_MISO

/* ---------- I2C / XL9555 ---------- */
#define IIC_SCL           42
#define IIC_SDA           41
#define XL9555_ADDR       0x20
#define XL9555_P0         0    // 端口0
#define XL9555_P1         1    // 端口1
// LCD_BL  = P1 bit0
// LCD_RST = P1 bit2
// LCD_PWR = P1 bit3
// OV_RESET= P0 bit5
// OV_PWDN = P0 bit4
```
