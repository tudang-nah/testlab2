# Mô phỏng biến tần PowerFlex 525 – BTN#2

Web tĩnh (HTML/CSS/JS thuần, không cần build) mô phỏng bộ thí nghiệm: biến tần + động cơ ESA 632-4 + panel công tắc 01/02/03/05/06, Vr1, DrivePot, keypad/LCD, PID.

## Chạy thử
Mở `index.html` bằng trình duyệt (hoặc `npx serve .`).

## Đưa lên GitHub + Vercel
1. Tạo repo mới trên GitHub, upload 4 file: `index.html`, `style.css`, `app.js`, `README.md` (ở thư mục gốc repo).
2. Vào vercel.com → Add New → Project → Import repo đó.
3. Framework Preset: **Other**; Build Command và Output Directory để trống → Deploy.

## Chức năng
- Keypad đầy đủ: Esc/Sel/Enter/▲▼, Start/Stop/Reverse, vào nhóm b/P/t/A, xem/sửa thanh ghi (Sel đổi chữ số).
- Reset nhà máy P053=2 → lỗi F048, Stop để xóa.
- Nguồn Start P046 (keypad/DI), nguồn tần số P047 (DrivePot, Keypad, 0–10 V, Preset, PID1).
- DI 2-wire (t062=48, t063=50) và 3-wire (49/51), chân 01 STOP tích cực thấp.
- Preset Freq A410–A413 (05 = bit nhỏ), PID A458–A466 (Kp, Ti, Td, Deadband, Preload).
- Mô hình V/Hz, trượt, dòng, ramp P041/P042, giới hạn P043/P044; đồ thị 60 s; ghi số liệu → CSV.
- Nút nạp nhanh cấu hình cho TN1, TN2.1–2.6, TN3.

Lưu ý: mô hình gần đúng cho học tập; thanh ghi b010 (rpm) là tuỳ biến của mô phỏng.
