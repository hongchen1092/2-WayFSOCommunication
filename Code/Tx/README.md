# 2-Way FSO Communication – Tx Firmware (ESP32-S3)

This directory contains the transmitter firmware for the **2-Way Free-Space Optical Communication (FSO)** system.  
It runs on the **ESP32-S3** and handles **TinyUSB HID host data** transfer over a **UART link** to the receiver module.

---

## 📂 Repository Structure
```
2-WayFSOCommunication/Code/Tx/
├── CMakeLists.txt
├── dependencies.lock
├── sdkconfig
├── common/
│   └── link_proto.h
└── main/
    ├── CMakeLists.txt
    ├── idf_component.yml
    └── main.c
```

---

## ⚙️ Build Environment
- **ESP-IDF:** v5.0 or newer  
- **Toolchain:** xtensa-esp32s3-elf (v14.2.0 recommended)  
- **Python:** ≥ 3.11  
- Works on Windows / Linux with ESP-IDF CLI or VS Code extension  

---

## 🧩 Dependencies
Managed automatically via:
- `main/idf_component.yml` → component manifest  
- `dependencies.lock` → resolved versions  

No `managed_components/` folder is tracked; dependencies restore at configure time.

---

## 🛠️ Build & Flash
```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```

Use the correct COM port or `/dev/tty` path if not auto-detected.

---

## 🔧 Runtime Configuration
- **UART:** TX = GPIO18, RX = GPIO19, Baud = 115200  
- **USB Host:** Requires 5V VBUS for TinyUSB HID operation  
- Logs display `HOST_TX: USB/HID host ready (V2 API)` once initialized  

---

## 🧾 Troubleshooting
- **TinyUSB not found:** `idf.py reconfigure`  
- **Target error:** `idf.py fullclean && idf.py set-target esp32s3`  
- **HID not enumerating:** Verify 5V VBUS and ID pin for host mode  

---

## 🧱 Hardware Checklist
- 5V supply to VBUS enabled  
- Correct D+ / D– routing to internal PHY  
- UART pins wired per config (TX18 → RX19)  

---

© 2025 Udaya Nirmal Fernando Kristhobuge
