#!/bin/bash

# --- Cấu hình ---
SERVICE_NAME="control_system_agv.service"

# --- Bắt đầu Script ---

echo ">> Bắt đầu quá trình cập nhật cho dịch vụ: $SERVICE_NAME"
echo "----------------------------------------------------"

# Bước 1: Chạy lệnh 'make' để biên dịch code
echo ">> Bước 1: Đang chạy 'make' để build tệp thực thi mới..."
make

# Bước 2: Kiểm tra xem 'make' có thành công không
# Lệnh 'if [ $? -eq 0 ]' kiểm tra mã thoát của lệnh cuối cùng. 0 nghĩa là thành công.
if [ $? -eq 0 ]; then
    echo ">> Build thành công!"
    echo "----------------------------------------------------"

    # Bước 3: Khởi động lại dịch vụ systemd
    echo ">> Bước 2: Đang khởi động lại dịch vụ '$SERVICE_NAME'..."
    sudo systemctl daemon-reload
    sleep 2
    sudo systemctl restart $SERVICE_NAME

    echo ">> Dịch vụ đã được yêu cầu khởi động lại."
    echo "----------------------------------------------------"

    # Bước 4 (Tùy chọn nhưng rất hữu ích): Kiểm tra trạng thái dịch vụ sau khi khởi động lại
    echo ">> Bước 3: Kiểm tra trạng thái dịch vụ (chờ 2 giây)..."
    sleep 2
    sudo systemctl status $SERVICE_NAME
else
    # Thông báo lỗi nếu 'make' thất bại
    echo ">> LỖI: Quá trình build thất bại! Dịch vụ '$SERVICE_NAME' KHÔNG được khởi động lại."
    echo ">> Vui lòng kiểm tra lại lỗi biên dịch ở trên."
    exit 1 # Thoát script với mã lỗi
fi

echo "----------------------------------------------------"
echo ">> Quá trình hoàn tất."
