# ESP32 -> STM32F1 SWD programmer

The ESP32 uses GPIO18 as SWCLK and GPIO19 as SWDIO and exposes a serial programming protocol at 115200 8N1.

## Firmware

Build/upload with Arduino CLI:

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32 esp32-swd.ino
arduino-cli upload -p COM3 --fqbn esp32:esp32:esp32 esp32-swd.ino
```

## Host flasher

Install the dependency:

```powershell
python -m pip install -r requirements.txt
```

Flash a binary:

```powershell
python stm32_swd.py COM3 firmware.bin --address 0x08000000
```

Flash an Intel HEX image:

```powershell
python stm32_swd.py COM3 firmware.hex
```

Safe connection/target query:

```powershell
python stm32_swd.py COM3 --info
```

The host tool releases DTR/RTS after opening the ESP32 serial port because common ESP32 auto-reset circuits use these control lines.

## Protocol

- `PING` -> `PONG STM32-SWD`
- `INFO` -> SW-DP IDCODE and detected STM32F1 flash size
- `ERASE <address> <size>` -> erase all 1 KiB pages touched by the range
- `WRITE <address> <length> <crc32>` followed by raw bytes -> program halfwords and verify them
- `RESET` -> Cortex-M system reset

The programmer currently targets STM32F1 medium-density parts (up to 128 KiB internal flash) using the STM32F1 flash controller registers and SWD MEM-AP.

## STM32 temperature demo

`stm32-temperature.ino` is an STM32F1 Arduino sketch that reads the internal temperature-sensor ADC channel and sends a periodic line over USART1 (PA9) at 115200 baud:

```text
TEMP_RAW=1680 TEMP_C=25.4
```

The temperature is only an approximate estimate because STM32F1 internal sensor calibration and actual VDDA vary by chip. The formula assumes 3.3 V VDDA and uses the typical 1.43 V @ 25 C / 4.3 mV/C characteristics.

### Wiring

```text
STM32F1       ESP32
PA9 (USART1 TX) -> GPIO16 (RX2)
GND             -> GND
```

GPIO17 is configured as ESP32 TX2 but is not required for this one-way temperature demo. Keep SWD wiring unchanged: ESP32 GPIO18 -> SWCLK and GPIO19 -> SWDIO.

### Flash the STM32 sketch

After installing an STM32 Arduino core that supports your exact F1 board, compile the sketch with that board's FQBN and use the existing ESP32 SWD flasher:

```powershell
arduino-cli compile --fqbn <STM32-F1-FQBN> stm32-temperature.ino
python stm32_swd.py COM3 stm32-temperature.ino.bin --address 0x08000000
```

The current machine only has the `esp32:esp32` Arduino core installed, and its network connection could not download the STM32 core, so the STM32 image could not be built automatically in this run. The source is ready for an STM32F1 Arduino core.

### Read temperature through ESP32

Once the STM32 firmware is running and PA9 is connected to ESP32 GPIO16:

```powershell
python stm32_swd.py COM3 --temp
```

The ESP32 continuously receives the STM32 UART output and returns the latest temperature through the USB serial protocol using `TEMP`.

The original experimental diagnostic sketch is preserved as `esp32-swd.diagnostic.txt`.
