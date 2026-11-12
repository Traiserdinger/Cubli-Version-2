# Prompt cho Claude Code — Firmware CubliV2

> Cách dùng: `git clone https://github.com/Traiserdinger/CubliV2` → `cd CubliV2` → chạy `claude`
> → dán toàn bộ khối prompt bên dưới (từ "BỐI CẢNH" đến hết). Nên bật Plan Mode cho lượt đầu.

---

**BỐI CẢNH**

Bạn là kỹ sư firmware nhúng cấp senior (ESP32, FreeRTOS, điều khiển hệ thống, ước lượng trạng thái).
Repo này là **CubliV2** (https://github.com/Traiserdinger/CubliV2): một khối lập phương tự cân bằng
bằng 3 bánh đà phản lực (giống Cubli của ETH Zürich / RemRC), mạch do tôi tự thiết kế bằng Altium.
Mục tiêu: viết **firmware hoàn chỉnh, chất lượng cao hơn bản RemRC** trong thư mục `firmware/`.

Phần cứng tóm tắt (chi tiết và **nguồn sự thật duy nhất** là `context.md`):
- ESP32-WROOM-32E 4 MB, nạp qua USB-C/CH340C (auto-reset).
- 3× motor **Nidec 24H** (driver tích hợp + encoder 100 xung), mỗi motor 8 dây theo thứ tự chân
  1→8: đỏ +12 V, đen GND, vàng START/BRAKE (chung 1 GPIO cho cả 3 motor), trắng PWM (~20 kHz, đảo:
  LOW = nhanh), xanh lá DIR, xanh dương +5 V encoder, tím encoder (CHB), xám encoder (CHA).
  Thứ tự dây này được suy ngược từ connector cũ + tài liệu nhà sản xuất → **phải xác minh bằng bench test**.
- Pin **3S LiPo** (9.0–12.6 V), INA219 @0x40 đo áp/dòng toàn hệ, shunt 10 mΩ.
- IMU **không nằm trên mạch này**: nằm trên "IMU PCB" cắm vào J8, giao tiếp **UART2 (GPIO16 RX / GPIO17 TX)**.
- Buzzer thụ động GPIO19, LED trạng thái GPIO2, nút BOOT GPIO0 dùng làm nút người dùng.
- Lưu ý: số MOTOR in trên silkscreen **khác** số motor trong schematic (xem `context.md` §6).

**BƯỚC 1 — ĐỌC VÀ HIỂU REPO (bắt buộc, chưa viết code)**

1. Đọc kỹ `CLAUDE.md`, `context.md`, `README.md`.
2. Đọc `docs/CubliV2_Schematic.pdf` (5 trang: MCU, Power, Connector, PCB, BOM) và các ảnh trong
   `docs/images/`. Đối chiếu **từng dòng** của bảng GPIO trong `context.md` §4 với schematic.
   Nếu thấy bất kỳ sai lệch nào → liệt kê ra, không tự sửa.
3. Không cố parse các file nhị phân Altium (`.SchDoc`, `.PcbDoc`, `.IntLib`).
4. Xem `manufacturing/bom/CubliV2.xlsx` nếu cần thông tin linh kiện.
5. (Tùy chọn) Nếu có quyền truy cập, tham khảo repo ESP32 flight controller của tôi
   https://github.com/Traiserdinger/ESP-FC-F450-Quadcopter cho phần IMU/attitude/cấu trúc code,
   nhưng không copy mù quáng.

Sau đó trả lời tôi:
- Tóm tắt phần cứng theo cách bạn hiểu (power tree, bảng GPIO, chuỗi tín hiệu encoder, an toàn khi reset).
- Danh sách rủi ro/điểm nghi ngờ phần cứng ảnh hưởng tới firmware.
- Hỏi tôi **toàn bộ câu hỏi mở** trong `context.md` §11 (IMU PCB là gì + giao thức, framework,
  thông số cơ khí, hướng lắp motor/IMU, model motor, có cần Wi-Fi/BLE không) cùng các câu hỏi bạn
  tự thấy cần. **Dừng lại chờ tôi trả lời.**

**BƯỚC 2 — ĐỀ XUẤT KIẾN TRÚC (chờ tôi duyệt)**

Đề xuất kế hoạch chi tiết, mặc định (trừ khi tôi chọn khác):
- PlatformIO, `framework = espidf` (ESP-IDF 5.x), C++17, có env `native` cho unit test thuật toán.
- Cấu trúc module rõ ràng, mỗi khối một file/thư mục, ví dụ:
  ```
  firmware/
  ├── platformio.ini
  ├── include/board_pins.h          ← sinh từ context.md §4, có static_assert/comment nguồn
  ├── src/main.cpp                  ← chỉ khởi tạo + tạo task
  ├── lib/drivers/   motor (LEDC+DIR+BRAKE), encoder (PCNT), ina219, imu_link (UART2), buzzer, led, button
  ├── lib/estimation/ attitude (Mahony/complementary có bias gyro, hoặc EKF), wheel_speed filter
  ├── lib/control/   lqr_balance (edge + vertex), mixer 3 bánh, setpoint/trim
  ├── lib/app/       supervisor state machine, safety, params (NVS), cli, telemetry
  ├── test/          unit test native (filter, LQR, mixer, INA219 conversion, state machine)
  └── tools/         Python: tính gain LQR từ mô hình, vẽ telemetry
  ```
- Task FreeRTOS: control loop chạy cố định (500 Hz–1 kHz, chọn theo tốc độ IMU) trên core 1,
  ưu tiên cao nhất, đo jitter/overrun; comms/CLI/INA219/buzzer trên core 0.
- Nêu rõ: tần số vòng điều khiển, định dạng frame telemetry, danh sách lệnh CLI, các tham số NVS,
  sơ đồ state machine (BOOT → IDLE → CALIBRATE → READY → ARMED_EDGE/ARMED_VERTEX → FAULT).
- **Dừng lại chờ tôi duyệt trước khi code.**

**BƯỚC 3 — TRIỂN KHAI THEO PHA (mỗi pha: code → build sạch không warning → test → commit nhỏ → báo cáo)**

- **Pha A – Bring-up & bench test** (quan trọng nhất, làm trước):
  - Driver motor/encoder/INA219/buzzer/LED/button + CLI qua USB (115200 hoặc 921600).
  - Lệnh tối thiểu: `status`, `bat`, `enc`, `brake on|off`, `motor <1-3> <-1000..1000>`,
    `stop`, `beep`, `imu raw`, `param get|set|save|list`, `reboot`.
  - Một lệnh `wiretest <n>` có hướng dẫn từng bước để **xác nhận thứ tự dây Nidec 24H** (vàng = brake,
    trắng = PWM đảo, xanh lá = DIR, tím/xám = encoder; quay tay 10 vòng ≈ 4000 count) và tự gợi ý
    giá trị `dir_invert`, `enc_invert` cho từng motor, cùng bảng ánh xạ Motor schematic ↔ silkscreen ↔ trục khối.
- **Pha B – IMU link + ước lượng tư thế** (sau khi tôi trả lời về IMU PCB): parser UART có checksum,
  timeout → FAULT, hiệu chuẩn offset, ước lượng góc/tốc độ góc ở tần số vòng điều khiển, xuất telemetry.
- **Pha C – Cân bằng cạnh (edge)**: LQR state feedback [góc, tốc độ góc, tốc độ bánh] (+ tích phân
  tốc độ bánh tùy chọn), chọn cạnh nào dựa theo tư thế, hiệu chuẩn điểm cân bằng (giữ khối ở điểm
  cân bằng + nhấn BOOT để lưu vào NVS), script `tools/` tính gain từ thông số cơ khí.
- **Pha D – Cân bằng đỉnh (vertex)**: biến đổi trục thân ↔ 3 bánh, điều khiển 3 trục đồng thời.
- **Pha E – Hoàn thiện**: tuning qua Wi-Fi/BLE (nếu tôi chọn), OTA, tài liệu `firmware/README.md`.

**QUY TẮC BẮT BUỘC**

1. **An toàn motor**: khi reset các pull-down làm BRAKE = LOW (đang phanh) nhưng PWM = LOW (= lệnh
   chạy max). Luôn khởi tạo PWM ở mức dừng (duty 100 %) **trước**, chỉ nhả BRAKE khi đã ARM; về
   FAULT ⇒ PWM dừng + BRAKE LOW. Ngắt khi: pin < 9.9 V (lọc), dòng > ngưỡng, nghiêng quá giới hạn,
   mất IMU, watchdog. Motor mặc định **không bao giờ tự chạy** khi khởi động.
2. Không bịa chân, không đổi pin map. Mọi hằng số phần cứng chỉ nằm trong `board_pins.h` /
   `config` có comment trích nguồn (`context.md §x`). Thứ tự dây motor/hướng quay/hướng encoder là
   tham số cấu hình, không hard-code.
3. Code sạch, module hóa, không biến global lung tung, không cấp phát động trong vòng điều khiển,
   không `printf` trong ISR/control task (dùng queue/ring buffer).
4. Trước khi **thay đổi hoặc "cải tiến" thuật toán điều khiển/gain đã thống nhất, phải hỏi tôi.**
5. Mỗi pha: chạy `pio run` (và `pio test -e native` nếu có) đến khi pass, rồi commit với message rõ
   ràng. **Không push** lên GitHub khi chưa hỏi tôi.
6. Khi không chắc về phần cứng → hỏi, đừng đoán. Khi tôi báo kết quả đo/bench test → cập nhật
   `context.md` (mục liên quan) để giữ nó luôn đúng.
7. Trả lời tôi bằng tiếng Việt, code + comment + commit message bằng tiếng Anh.

Bắt đầu với **BƯỚC 1**.
