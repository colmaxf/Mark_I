#!/bin/bash

# Script này sẽ tự động cài đặt và cấu hình DLT daemon, DLT logger,
# và dịch vụ control_system cho AGV.
# Chạy script này với quyền sudo: sudo ./setup_agv.sh

# Kiểm tra quyền root
if [ "$EUID" -ne 0 ]; then
  echo "Vui lòng chạy script này với quyền sudo."
  exit 1
fi

echo ">> Bắt đầu quá trình cài đặt tự động..."
echo "===================================================="

# --- Bước 0: Dọn dẹp process cũ ---
echo ">> Bước 0: Dọn dẹp các process cũ..."
systemctl stop dlt-daemon.service 2>/dev/null || true
systemctl stop dlt-logger.service 2>/dev/null || true
systemctl stop control_system_agv.service 2>/dev/null || true
pkill -9 dlt-daemon 2>/dev/null || true
pkill -9 dlt-receive 2>/dev/null || true
pkill -9 control_system 2>/dev/null || true
sleep 2
echo "OK"
echo "===================================================="

# --- Bước 1: Tạo các tệp cấu hình và dịch vụ ---

# Tạo thư mục cần thiết
echo "--> Tạo thư mục /usr/local/bin và /var/log/dlt..."
mkdir -p /usr/local/bin
mkdir -p /var/log/dlt
chmod 755 /var/log/dlt

# 1. Tệp /etc/dlt.conf - FIXED VERSION
echo "--> Tạo tệp /etc/dlt.conf..."
cat > /etc/dlt.conf << 'EOF'
########################################################################
# Configuration file of DLT daemon - Compatible with v2.18.10
########################################################################

# Offline trace storage
OfflineTraceDirectory = /var/log/dlt
OfflineTraceFileSize = 10000000
OfflineTraceMaxSize = 150000000

# Verbose mode
Verbose = 1

# Send serial header
SendSerialHeader = 1

# Send context registration
SendContextRegistration = 1

# Send message time
SendMessageTime = 1

# RS232 sync serial header
RS232SyncSerialHeader = 1

# TCP sync serial header  
TCPSyncSerialHeader = 1

# Gateway mode
GatewayMode = 0

# Injection mode
InjectionMode = 1

# Context settings (SUPPORTED options only)
ContextLogLevel = 6
ContextTraceStatus = 1
ForceContextLogLevelAndTraceStatus = 0

# Binding address (listen on all interfaces)
BindAddress = 0.0.0.0

# ECU ID
ECUId = ECU1
EOF

# 2. Tệp /etc/systemd/system/dlt-daemon.service - FIXED VERSION
echo "--> Tạo tệp /etc/systemd/system/dlt-daemon.service..."
cat > /etc/systemd/system/dlt-daemon.service << 'EOF'
[Unit]
Description=DLT Daemon for AGV System
After=network.target

[Service]
Type=simple
# Kill any existing dlt-daemon processes before starting
ExecStartPre=/bin/sh -c 'pkill -9 dlt-daemon || true'
ExecStartPre=/bin/sh -c 'sleep 1'
ExecStartPre=/bin/mkdir -p /var/log/dlt
ExecStartPre=/bin/chmod 755 /var/log/dlt
ExecStart=/usr/bin/dlt-daemon -c /etc/dlt.conf
Restart=always
RestartSec=5
User=root
StandardOutput=journal
StandardError=journal

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

# Đợi dlt-daemon khởi động
echo "Waiting for dlt-daemon to start..."
for i in {1..30}; do
    if pgrep -x dlt-daemon > /dev/null; then
        echo "dlt-daemon is running"
        break
    fi
    sleep 1
done

# Đợi thêm 2 giây để daemon hoàn toàn sẵn sàng
sleep 2

LOG_FILE="$LOG_DIR/agv_$(date +%Y%m%d_%H%M%S).dlt"
echo "Logging to: $LOG_FILE"

# Start logging với timeout
timeout 86400 dlt-receive -a localhost -o "$LOG_FILE" &
PID=$!

# Monitor với error handling
while kill -0 $PID 2>/dev/null; do
    if [ -f "$LOG_FILE" ]; then
        SIZE=$(stat -c%s "$LOG_FILE" 2>/dev/null || echo 0)
        MAX=$((MAX_SIZE_MB * 1024 * 1024))
        
        if [ $SIZE -gt $MAX ]; then
            kill $PID 2>/dev/null
            wait $PID 2>/dev/null
            cleanup_old
            
            # Check retry count
            if [ $RETRY_COUNT -ge $MAX_RETRIES ]; then
                echo "Max retries reached, exiting"
                exit 1
            fi
            
            LOG_FILE="$LOG_DIR/agv_$(date +%Y%m%d_%H%M%S).dlt"
            timeout 86400 dlt-receive -a localhost -o "$LOG_FILE" &
            PID=$!
            RETRY_COUNT=$((RETRY_COUNT + 1))
            echo "Rotated to: $LOG_FILE (retry: $RETRY_COUNT)"
        fi
    fi
    sleep 60  # Check every 60 seconds
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
RestartSec=10
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
echo "==================================================="
echo "DLT Log Viewer"
echo "==================================================="
echo ""
echo "Các tệp log DLT hiện có:"
ls -lh /var/log/dlt/*.dlt 2>/dev/null || echo "  (No log files yet)"
echo ""
echo "---------------------------------------------------"
echo "Để xem log mới nhất:"
LATEST_LOG=$(ls -t /var/log/dlt/agv_*.dlt 2>/dev/null | head -n 1)
if [ -n "$LATEST_LOG" ]; then
    echo "  dlt-convert -a \"$LATEST_LOG\" | tail -50"
    echo ""
    echo "Hoặc xem trực tiếp 20 dòng cuối:"
    echo "---------------------------------------------------"
    dlt-convert -a "$LATEST_LOG" 2>/dev/null | tail -20
else
    echo "  (No log file found)"
fi
echo ""
echo "---------------------------------------------------"
echo "Để theo dõi logs real-time:"
echo "  dlt-receive -a localhost"
echo ""
echo "Trạng thái dịch vụ:"
systemctl is-active dlt-daemon.service | xargs echo "  dlt-daemon:"
systemctl is-active dlt-logger.service | xargs echo "  dlt-logger:"
echo "==================================================="
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
ExecStart=/bin/sh -c "mkdir -p /tmp/dlt /var/log/dlt; chmod 777 /tmp/dlt 2>/dev/null || true; chmod 755 /var/log/dlt; exit 0"
RemainAfterExit=true

[Install]
WantedBy=multi-user.target
EOF

# 8. Tệp /etc/systemd/system/control_system_agv.service
echo "--> Tạo tệp /etc/systemd/system/control_system_agv.service..."
# Lấy user thực tế
REAL_USER=${SUDO_USER:-$(logname 2>/dev/null || whoami)}

cat > /etc/systemd/system/control_system_agv.service << EOF
[Unit]
Description=My C++ Application Service AGV
After=dlt-permissions.service
Wants=dlt-permissions.service

[Service]
ExecStart=/usr/local/bin/control_system
WorkingDirectory=/home/$REAL_USER/Documents/Mark_I
StandardOutput=inherit
StandardError=inherit
Restart=always
RestartSec=5
User=$REAL_USER

[Install]
WantedBy=multi-user.target
EOF

echo "===================================================="
# --- Bước 2: Cấp quyền thực thi ---
echo ">> Bước 2: Cấp quyền thực thi cho các script..."
chmod 755 /usr/local/bin/dlt-auto-logger.sh
chmod 755 /usr/local/bin/dlt-view-logs.sh
echo "OK"
echo "===================================================="

# --- Bước 3: Cài đặt và khởi chạy các dịch vụ ---
echo ">> Bước 3: Tải lại, kích hoạt và khởi động các dịch vụ systemd..."

# Tải lại daemon để nhận diện các file service mới
systemctl daemon-reload

# Kích hoạt các dịch vụ để tự khởi động cùng hệ thống
echo "--> Kích hoạt các dịch vụ..."
systemctl enable dlt-daemon.service
systemctl enable dlt-logger.service
systemctl enable dlt-permissions.service
systemctl enable control_system_agv.service

# Khởi động lại các dịch vụ theo đúng thứ tự
echo "--> Khởi động lại các dịch vụ (có thể mất vài giây)..."
systemctl restart dlt-daemon.service
sleep 3
systemctl restart dlt-logger.service
sleep 3
systemctl restart dlt-permissions.service
sleep 2
systemctl restart control_system_agv.service
sleep 2
echo "OK"
echo "===================================================="

# --- Bước 4: Kiểm tra trạng thái các dịch vụ ---
echo ">> Bước 4: Kiểm tra trạng thái các dịch vụ đã cài đặt..."
echo ""
echo ">>> DLT Daemon Status:"
systemctl status dlt-daemon.service --no-pager -l
echo ""
echo "----------------------------------------------------"
echo ">>> DLT Logger Status:"
systemctl status dlt-logger.service --no-pager -l
echo ""
echo "----------------------------------------------------"
echo ">>> DLT Permissions Status:"
systemctl status dlt-permissions.service --no-pager -l
echo ""
echo "----------------------------------------------------"
echo ">>> Control System AGV Status:"
systemctl status control_system_agv.service --no-pager -l

echo ""
echo "===================================================="
echo ">> Quá trình cài đặt đã hoàn tất!"
echo "===================================================="
echo ""
echo "Các lệnh hữu ích:"
echo "  - Xem logs:              dlt-view-logs.sh"
echo "  - Theo dõi real-time:    dlt-receive -a localhost"
echo "  - Restart dịch vụ:       sudo systemctl restart dlt-daemon"
echo "  - Xem log systemd:       sudo journalctl -u dlt-daemon -f"
echo "  - Kiểm tra port 3490:    sudo netstat -tulpn | grep 3490"
echo ""
echo "===================================================="