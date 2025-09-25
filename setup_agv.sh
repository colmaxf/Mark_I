#!/bin/bash

# Script này sẽ tự động cài đặt và cấu hình DLT daemon, DLT logger,
# và dịch vụ control_system cho AGV.
# Chạy script này với quyền sudo: sudo ./setup.sh

# Kiểm tra quyền root
if [ "$EUID" -ne 0 ]; then
  echo "Vui lòng chạy script này với quyền sudo."
  exit 1
fi

echo ">> Bắt đầu quá trình cài đặt tự động..."
echo "===================================================="

# --- Bước 1: Tạo các tệp cấu hình và dịch vụ ---

# Tạo thư mục cần thiết
echo "--> Tạo thư mục /usr/local/bin và /var/log/dlt..."
mkdir -p /usr/local/bin
mkdir -p /var/log/dlt

# 1. Tệp /etc/dlt.conf
echo "--> Tạo tệp /etc/dlt.conf..."
cat > /etc/dlt.conf << 'EOF'
# Cấu hình cho dlt-daemon v2.x

# Chế độ ghi log: 2 = cả mạng và file
LoggingMode = 2

# Đường dẫn lưu file log
#OfflineTracePath = /tmp/dlt_storage/

# Kích thước tối đa mỗi file (bytes) - ví dụ 10MB
OfflineTraceFileSize = 10000000

# Số lượng file tối đa trước khi xoay vòng
OfflineTraceFileNumber = 15

# Bật server TCP cho DLT Viewer
#TCPServer = true
#TCPServerPort = 3490
#TCPServerAddress = 0.0.0.0
########################################
# Network configuration - IMPORTANT!
########################################
# Enable remote connections via TCP/IP
RemoteConnection = 1

# Port for DLT Viewer connections (default: 3490)
RemotePort = 3490

# Accept connections from any IP (0.0.0.0)
BindAddress = 0.0.0.0

# Maximum concurrent connections
MaxConnections = 10
EOF

# 2. Tệp /etc/systemd/system/dlt-daemon.service
echo "--> Tạo tệp /etc/systemd/system/dlt-daemon.service..."
cat > /etc/systemd/system/dlt-daemon.service << 'EOF'
[Unit]
Description=DLT Daemon for AGV System
After=network.target

[Service]
Type=simple
ExecStart=/usr/bin/dlt-daemon -c /etc/dlt.conf
Restart=always
RestartSec=5
User=root

[Install]
WantedBy=multi-user.target
EOF

# 3. Tệp /usr/local/bin/dlt-auto-logger.sh
echo "--> Tạo tệp /usr/local/bin/dlt-auto-logger.sh..."
cat > /usr/local/bin/dlt-auto-logger.sh << 'EOF'
#!/bin/bash

LOG_DIR="/var/log/dlt"
MAX_FILES=15
MAX_SIZE_MB=50
RETRY_COUNT=0
MAX_RETRIES=5

mkdir -p $LOG_DIR

cleanup_old() {
    COUNT=$(ls -1 $LOG_DIR/*.dlt 2>/dev/null | wc -l)
    if [ $COUNT -gt $MAX_FILES ]; then
        ls -t $LOG_DIR/*.dlt | tail -$((COUNT-MAX_FILES)) | xargs rm -f
    fi
}

# Trap signals để cleanup properly
trap "kill $PID 2>/dev/null; exit 0" SIGTERM SIGINT

start_logging() {
    LOG_FILE="$LOG_DIR/agv_$(date +%Y%m%d_%H%M%S).dlt"
    echo "Bắt đầu ghi log vào: $LOG_FILE"
    # Start logging với timeout 24 giờ (86400 giây)
    timeout 86400 dlt-receive -a localhost -o "$LOG_FILE" &
    PID=$!
}

start_logging

# Monitor với error handling
while kill -0 $PID 2>/dev/null; do
    if [ -f "$LOG_FILE" ]; then
        SIZE=$(stat -c%s "$LOG_FILE" 2>/dev/null || echo 0)
        MAX=$((MAX_SIZE_MB * 1024 * 1024))
        
        if [ $SIZE -gt $MAX ]; then
            echo "Log file reached maximum size. Rotating log..."
            kill $PID 2>/dev/null
            wait $PID 2>/dev/null
            cleanup_old

            # Check retry count
            if [ $RETRY_COUNT -ge $MAX_RETRIES ]; then
            echo "Maximum retry count reached, exiting."
            exit 1
            fi

            RETRY_COUNT=$((RETRY_COUNT + 1))
            start_logging
        fi
    fi
    sleep 60  # Kiểm tra mỗi 60 giây
done
EOF

# 4. Tệp /etc/systemd/system/dlt-logger.service
echo "--> Tạo tệp /etc/systemd/system/dlt-logger.service..."
cat > /etc/systemd/system/dlt-logger.service << 'EOF'
[Unit]
Description=DLT Auto Logger
After=dlt-daemon.service
Requires=dlt-daemon.service

[Service]
Type=simple
ExecStart=/usr/local/bin/dlt-auto-logger.sh
Restart=always
RestartSec=5
# Thêm giới hạn tài nguyên
MemoryLimit=500M
CPUQuota=50%
Nice=10
IOWeight=50

[Install]
WantedBy=multi-user.target
EOF

# 5. Tệp /usr/local/bin/dlt-view-logs.sh
echo "--> Tạo tệp /usr/local/bin/dlt-view-logs.sh..."
cat > /usr/local/bin/dlt-view-logs.sh << 'EOF'
#!/bin/bash
echo "Các tệp log DLT hiện có:"
ls -lh /var/log/dlt/*.dlt 2>/dev/null || echo "No log files yet"
echo ""
echo "To view latest log:"
LATEST_LOG=$(ls -t /var/log/dlt/agv_*.dlt 2>/dev/null | head -n 1)
if [ -n "$LATEST_LOG" ]; then
    echo "  dlt-convert -a \"$LATEST_LOG\" | tail -50"
else
    echo "  (No log file found)"
fi
echo ""
echo "To follow logs in real-time:"
echo "  dlt-receive -a localhost"
EOF

# 6. Tệp /etc/cron.d/dlt-cleanup
echo "--> Tạo tệp /etc/cron.d/dlt-cleanup để dọn log cũ..."
cat > /etc/cron.d/dlt-cleanup << 'EOF'
# Dọn dẹp các tệp log DLT cũ hơn 7 ngày vào 2 giờ sáng hàng ngày
0 2 * * * root find /var/log/dlt -name "*.dlt" -mtime +7 -delete
EOF

# 7. Tệp /etc/systemd/system/dlt-permissions.service
echo "--> Tạo tệp /etc/systemd/system/dlt-permissions.service..."
cat > /etc/systemd/system/dlt-permissions.service << 'EOF'
[Unit]
Description=Setup DLT Permissions
After=dlt-logger.service

[Service]
Type=oneshot
ExecStart=/bin/chmod 777 /tmp/dlt
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
EOF

# 8. Tệp /etc/systemd/system/control_system_agv.service
echo "--> Tạo tệp /etc/systemd/system/control_system_agv.service..."
cat > /etc/systemd/system/control_system_agv.service << 'EOF'
[Unit]
Description=My C++ Application Service AGV
# Chỉ cần đợi dịch vụ cuối cùng trong chuỗi DLT và đợi mạng online
#After=network-online.target dlt-permissions.service
After= dlt-permissions.service
# Yêu cầu mạng online và dịch vụ DLT phải chạy thành công
#Wants=network-online.target
Requires=dlt-permissions.service

[Service]
#ExecStart=/home/kautopi/Documents/Mark_I/control_system
ExecStart=/usr/local/bin/control_system
WorkingDirectory=/home/kautopi/Documents/Mark_I
StandardOutput=inherit
StandardError=inherit
Restart=always

[Install]
WantedBy=multi-user.target
EOF

echo "===================================================="
# --- Bước 2: Cấp quyền thực thi ---
echo ">> Bước 2: Cấp quyền thực thi 777 cho các script..."
chmod 777 /usr/local/bin/dlt-auto-logger.sh
chmod 777 /usr/local/bin/dlt-view-logs.sh
echo "OK"
echo "===================================================="

# --- Bước 3: Cài đặt và khởi chạy các dịch vụ ---
echo ">> Bước 3: Tải lại, kích hoạt và khởi động các dịch vụ systemd..."

# Tải lại daemon để nhận diện các file service mới
sudo systemctl daemon-reload

# Kích hoạt các dịch vụ để tự khởi động cùng hệ thống
echo "--> Kích hoạt các dịch vụ..."
sudo systemctl enable dlt-daemon.service
sudo systemctl enable dlt-logger.service
sudo systemctl enable dlt-permissions.service
sudo systemctl enable control_system_agv.service

# Khởi động lại các dịch vụ theo đúng thứ tự
echo "--> Khởi động lại các dịch vụ (có thể mất vài giây)..."
sudo systemctl restart dlt-daemon.service
sleep 3
sudosystemctl restart dlt-logger.service
sleep 3
sudo systemctl restart dlt-permissions.service
sleep 3
sudo systemctl restart control_system_agv.service
echo "OK"
echo "===================================================="

# --- Bước 4: Kiểm tra trạng thái các dịch vụ ---
echo ">> Bước 4: Kiểm tra trạng thái các dịch vụ đã cài đặt..."
echo ""
sudo systemctl status dlt-daemon.service --no-pager
echo "----------------------------------------------------"
sudo systemctl status dlt-logger.service --no-pager
echo "----------------------------------------------------"
sudo systemctl status dlt-permissions.service --no-pager
echo "----------------------------------------------------"
sudo systemctl status control_system_agv.service --no-pager

echo "===================================================="
echo ">> Quá trình cài đặt đã hoàn tất!"
echo "Sử dụng 'dlt-view-logs.sh' để xem các lệnh kiểm tra log."

