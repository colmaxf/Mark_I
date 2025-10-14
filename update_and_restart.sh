#!/bin/bash

# --- Cấu hình ---
SERVICE_NAME="control_system_agv.service"
REAL_USER=${SUDO_USER:-$(whoami)}
# --- Bắt đầu Script ---

echo ">> Bắt đầu quá trình cập nhật cho dịch vụ: $SERVICE_NAME"
echo "----------------------------------------------------"

# Bước 1: Chạy lệnh 'make' để biên dịch code
echo ">> Bước 1: Đang chạy 'make' để build tệp thực thi mới..."
make clean
make
# -f là viết tắt của "file", nó kiểm tra xem đường dẫn có tồn tại và là một file hay không
if [ -f "/usr/local/bin/control_system" ]; then
  echo "File tồn tại. Đang xóa..."
  sudo rm -rf /usr/local/bin/control_system
else
  echo "File không tồn tại, không cần làm gì cả."
fi

echo "copy sang usr/local/bin/control_system"
sudo cp /home/$REAL_USER/Documents/Mark_I/control_system /usr/local/bin/
# Bước 2: Kiểm tra xem 'make' có thành công không
# Lệnh 'if [ $? -eq 0 ]' kiểm tra mã thoát của lệnh cuối cùng. 0 nghĩa là thành công.
if [ $? -eq 0 ]; then
    echo ">> Build thành công!"
    echo "----------------------------------------------------"

    # Bước 3: Khởi động lại dịch vụ systemd
    echo ">> Bước 2: Đang khởi động lại dịch vụ '$SERVICE_NAME'..."
    sudo systemctl stop $SERVICE_NAME
    sleep 2
    sudo systemctl daemon-reload
    sleep 2
    sudo systemctl restart $SERVICE_NAME

    echo ">> Dịch vụ đã được yêu cầu khởi động lại."
    echo "----------------------------------------------------"

    # Bước 4 (Tùy chọn nhưng rất hữu ích): Kiểm tra trạng thái dịch vụ sau khi khởi động lại
    echo ">> Bước 3: Kiểm tra trạng thái dịch vụ (chờ 2 giây)..."
    sleep 2
    sudo systemctl status $SERVICE_NAME
    exit 1
    echo "----------------------------------------------------"
else
    # Thông báo lỗi nếu 'make' thất bại
    echo ">> LỖI: Quá trình build thất bại! Dịch vụ '$SERVICE_NAME' KHÔNG được khởi động lại."
    echo ">> Vui lòng kiểm tra lại lỗi biên dịch ở trên."
    exit 1 # Thoát script với mã lỗi
fi

echo "----------------------------------------------------"
echo ">> Quá trình hoàn tất."
