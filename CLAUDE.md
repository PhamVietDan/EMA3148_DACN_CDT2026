Đọc và làm theo PROJECT_PLAN.md. Tiến độ ghi trong PROGRESS.md.

Khác biệt bố cục so với PROJECT_PLAN.md §3.1 (đã áp dụng từ M0):
- Project CubeMX/CubeIDE nằm ngay ở thư mục gốc (`Drone.ioc`, `Core/`, `Drivers/`, ...) thay vì `firmware/`.
- Code ứng dụng nằm ở `App/` (thư mục gốc), có trong `.cproject` (source + include `../App`).
- Theo yêu cầu người dùng, cả 3 task (`task_ctrl`, `task_slow`, `task_telem`) nằm chung `App/control_logic/tasks.c/.h` thay vì `task_ctrl.c`, `task_slow.c`, `task_telem.c` như PROJECT_PLAN §3.1. Thuật toán/driver vẫn tách file riêng (`math_algorithm/` C thuần, test trên PC).
- Code cũ nằm ở `drone_stm32f405rgt6/` (đóng vai trò `legacy/`): CHỈ ĐỌC, không sửa, không đưa vào build.
- Build dòng lệnh để kiểm tra warning: `tools/build.ps1` (dùng GCC đi kèm STM32CubeIDE), App/ build với `-Wall -Wextra -Wdouble-promotion`.
