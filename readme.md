# Belajar RTOS IO USB CDC (STM32F103CBTx)

Proyek ini adalah contoh aplikasi firmware berbasis STM32CubeIDE untuk mikrokontroler STM32F103CBTx yang menggabungkan FreeRTOS, komunikasi USB CDC (Virtual COM Port), pembacaan input digital, dan pengendalian output melalui shift register 74HC595.

## Fitur Utama
- **FreeRTOS**: Manajemen multitasking menggunakan dua task utama (InputScanTask & OutputControlTask) dan message queue.
- **USB CDC**: Komunikasi serial virtual antara STM32 dan PC melalui USB.
- **Input Digital**: Membaca status 8 input digital (active low).
- **Output Digital**: Mengontrol 13 output digital menggunakan dua buah shift register 74HC595 via SPI.
- **Command Serial**: Pengendalian output melalui perintah satu karakter ('A'-'Z') dari USB CDC.

## Rangkaian Hardware
- **MCU**: STM32F103CBTx
- **Input**: 8 pin digital (active low, pull-up)
- **Output**: 13 channel via 2x 74HC595 (SPI1)
- **USB**: USB FS (PA11/PA12)
- **User Button**: PA0
- **User LED**: PB2

## Struktur Proyek
- `Core/` - Sumber utama aplikasi (main.c, freertos.c, dsb)
- `USB_DEVICE/` - Implementasi USB CDC
- `Middlewares/Third_Party/FreeRTOS/` - FreeRTOS
- `Middlewares/ST/STM32_USB_Device_Library/` - Library USB

## Cara Build & Flash
1. Buka proyek ini di **STM32CubeIDE** (versi 1.8.6 atau lebih baru direkomendasikan).
2. Pilih board/MCU: STM32F103CBTx.
3. Build project (`Ctrl+B`).
4. Hubungkan board ke PC via USB dan flash firmware menggunakan ST-Link.

## Cara Pakai
- Setelah terhubung ke PC, device akan muncul sebagai Virtual COM Port.
- Kirim karakter 'A', 'C', 'E', ... untuk mengaktifkan output 0, 1, 2, ...
- Kirim karakter 'B', 'D', 'F', ... untuk menonaktifkan output 0, 1, 2, ...
- Status input akan dikirimkan ke PC secara periodik (100ms) dalam format karakter.

## File Penting
- `Core/Src/main.c` - Logika utama aplikasi
- `Core/Inc/main.h` - Definisi pin dan fungsi utama
- `USB_DEVICE/App/usbd_cdc_if.c` - Handler USB CDC
- `belajar-rtos-io-usbcdc.ioc` - Konfigurasi CubeMX

## Lisensi
Lihat file LICENSE pada folder CMSIS dan STM32 HAL Driver.

---
Oleh: Ahmad, 2025