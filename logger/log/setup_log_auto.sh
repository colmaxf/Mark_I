#!/bin/bash

# Script tự động tạo file và cấu hình DLT
# Chạy với quyền root: sudo bash setup-dlt.sh

set -e  # Dừng script nếu có lỗi

echo "========================================="
echo "Bắt đầu cấu hình DLT System..."
echo "========================================="

# 1. Tạo file /etc/dlt.conf
echo "1. Tạo file /etc/dlt.conf..."
cat > /etc/dlt.conf << 'EOF'
# Cấu hình cho dlt-daemon v2.x
# Chế độ ghi log: 2 = cả mạng và file
LoggingMode = 2
########################################
# Network configuration - IMPORTANT!
########################################
# Accept connections from any IP (0.0.0.0)
BindAddress = 0.0.0.0
EOF
echo "   ✓ Đã tạo /etc/dlt.conf"

# 2. Tạo file /etc/systemd/system/dlt-daemon.service
echo "2. Tạo file dlt-daemon.service..."
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
echo "   ✓ Đã tạo dlt-daemon.service"

# 3. Tạo file /usr/local/bin/dlt-auto-logger.sh
echo "3. Tạo file dlt-auto-logger.sh..."
cat > /usr/local/bin/dlt-auto-logger.sh << 'EOF'
#!/bin/bash
LOG_DIR="/var/log/dlt"
MAX_FILES=15
MAX_SIZE_MB=50
mkdir -p $LOG_DIR
cleanup_old() {
    COUNT=$(ls -1 $LOG_DIR/*.dlt 2>/dev/null | wc -l)
    if [ $COUNT -gt $MAX_FILES ]; then
        ls -t $LOG_DIR/*.dlt | tail -$((COUNT-MAX_FILES)) | xargs rm -f
    fi
}
LOG_FILE="$LOG_DIR/agv_$(date +%Y%m%d_%H%M%S).dlt"
echo "Logging to: $LOG_FILE"
# Start logging
dlt-receive -a localhost -o "$LOG_FILE" &
PID=$!
# Monitor and rotate
while true; do
    if [ -f "$LOG_FILE" ]; then
        SIZE=$(stat -c%s "$LOG_FILE" 2>/dev/null || echo 0)
        MAX=$((MAX_SIZE_MB * 1024 * 1024))
        
        if [ $SIZE -gt $MAX ]; then
            kill $PID 2>/dev/null
            cleanup_old
            LOG_FILE="$LOG_DIR/agv_$(date +%Y%m%d_%H%M%S).dlt"
            dlt-receive -a localhost -o "$LOG_FILE" &
            PID=$!
            echo "Rotated to: $LOG_FILE"
        fi
    fi
    sleep 30
done
EOF
chmod 777 /usr/local/bin/dlt-auto-logger.sh
echo "   ✓ Đã tạo và phân quyền 777 cho dlt-auto-logger.sh"

# 4. Tạo file /etc/systemd/system/dlt-logger.service
echo "4. Tạo file dlt-logger.service..."
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
[Install]
WantedBy=multi-user.target
EOF
echo "   ✓ Đã tạo dlt-logger.service"

# 5. Tạo file /usr/local/bin/dlt-view-logs.sh
echo "5. Tạo file dlt-view-logs.sh..."
cat > /usr/local/bin/dlt-view-logs.sh << 'EOF'
#!/bin/bash
echo "DLT Log Files:"
ls -lh /var/log/dlt/*.dlt 2>/dev/null || echo "No log files yet"
echo ""
echo "To view latest log:"
echo "  dlt-convert -a /var/log/dlt/agv_*.dlt | tail -50"
echo ""
echo "To monitor live:"
echo "  dlt-receive -a localhost"
EOF
chmod 777 /usr/local/bin/dlt-view-logs.sh
echo "   ✓ Đã tạo và phân quyền 777 cho dlt-view-logs.sh"

# 6. Tạo file /etc/cron.d/dlt-cleanup
echo "6. Tạo file dlt-cleanup cron job..."
cat > /etc/cron.d/dlt-cleanup << 'EOF'
# Clean old DLT logs daily at 2 AM
0 2 * * * root find /var/log/dlt -name "*.dlt" -mtime +7 -delete
EOF
echo "   ✓ Đã tạo cron job dlt-cleanup"

# 7. Tạo file /etc/systemd/system/dlt-permissions.service
echo "7. Tạo file dlt-permissions.service..."
cat > /etc/systemd/system/dlt-permissions.service << 'EOF'
[Unit]
Description=Setup DLT Permissions
After=dlt-logger.service
After=multi-user.target
[Service]
Type=oneshot
ExecStart=/bin/chmod 777 /tmp/dlt
RemainAfterExit=true
[Install]
WantedBy=multi-user.target
EOF
echo "   ✓ Đã tạo dlt-permissions.service"

# 8. Restart các service
echo "========================================="
echo "8. Khởi động lại các service..."
echo "========================================="

echo "   Reload systemd daemon..."
systemctl daemon-reload

echo "   Enable các service..."
systemctl enable dlt-daemon.service
systemctl enable dlt-logger.service
systemctl enable dlt-permissions.service

echo "   Restart dlt-daemon.service..."
systemctl restart dlt-daemon.service
sleep 2

echo "   Restart dlt-logger.service..."
systemctl restart dlt-logger.service
sleep 2

echo "   Restart dlt-permissions.service..."
systemctl restart dlt-permissions.service

echo ""
echo "========================================="
echo "✓ CẤU HÌNH HOÀN TẤT!"
echo "========================================="

# Kiểm tra kết quả
echo ""
echo "========================================="
echo "KIỂM TRA KẾT QUẢ CẤU HÌNH:"
echo "========================================="

# Kiểm tra các file đã tạo
echo ""
echo "1. KIỂM TRA CÁC FILE ĐÃ TẠO:"
echo "-----------------------------------------"
files_to_check=(
    "/etc/dlt.conf"
    "/etc/systemd/system/dlt-daemon.service"
    "/usr/local/bin/dlt-auto-logger.sh"
    "/etc/systemd/system/dlt-logger.service"
    "/usr/local/bin/dlt-view-logs.sh"
    "/etc/cron.d/dlt-cleanup"
    "/etc/systemd/system/dlt-permissions.service"
)

for file in "${files_to_check[@]}"; do
    if [ -f "$file" ]; then
        perms=$(ls -l "$file" | awk '{print $1}')
        size=$(ls -lh "$file" | awk '{print $5}')
        echo "   ✓ $file [Size: $size, Perms: $perms]"
    else
        echo "   ✗ $file - KHÔNG TÌM THẤY!"
    fi
done

# Kiểm tra trạng thái services
echo ""
echo "2. TRẠNG THÁI CÁC SERVICES:"
echo "-----------------------------------------"

services=("dlt-daemon" "dlt-logger" "dlt-permissions")
for service in "${services[@]}"; do
    if systemctl is-active --quiet "$service.service"; then
        echo "   ✓ $service.service: $(systemctl is-active $service.service) - ĐANG CHẠY"
    else
        echo "   ✗ $service.service: $(systemctl is-active $service.service) - KHÔNG CHẠY"
    fi
    
    if systemctl is-enabled --quiet "$service.service"; then
        echo "      └─ Enabled: YES (tự động khởi động)"
    else
        echo "      └─ Enabled: NO"
    fi
done

# Kiểm tra thư mục log
echo ""
echo "3. KIỂM TRA THƯ MỤC LOG:"
echo "-----------------------------------------"
if [ -d "/var/log/dlt" ]; then
    echo "   ✓ /var/log/dlt đã được tạo"
    log_count=$(ls -1 /var/log/dlt/*.dlt 2>/dev/null | wc -l)
    echo "   └─ Số file log hiện tại: $log_count"
else
    echo "   ℹ /var/log/dlt chưa tồn tại (sẽ được tạo khi logger chạy)"
fi

# Kiểm tra thư mục /tmp/dlt
echo ""
echo "4. KIỂM TRA THƯ MỤC /tmp/dlt:"
echo "-----------------------------------------"
if [ -d "/tmp/dlt" ]; then
    perms=$(ls -ld /tmp/dlt | awk '{print $1}')
    echo "   ✓ /tmp/dlt tồn tại [Perms: $perms]"
else
    echo "   ℹ /tmp/dlt chưa tồn tại (sẽ được tạo khi daemon chạy)"
fi

# Kiểm tra process đang chạy
echo ""
echo "5. KIỂM TRA PROCESS ĐANG CHẠY:"
echo "-----------------------------------------"
if pgrep -x "dlt-daemon" > /dev/null; then
    pid=$(pgrep -x "dlt-daemon")
    echo "   ✓ dlt-daemon đang chạy [PID: $pid]"
else
    echo "   ✗ dlt-daemon không tìm thấy process"
fi

if pgrep -f "dlt-auto-logger.sh" > /dev/null; then
    pid=$(pgrep -f "dlt-auto-logger.sh")
    echo "   ✓ dlt-auto-logger.sh đang chạy [PID: $pid]"
else
    echo "   ✗ dlt-auto-logger.sh không tìm thấy process"
fi

if pgrep -x "dlt-receive" > /dev/null; then
    pid=$(pgrep -x "dlt-receive")
    echo "   ✓ dlt-receive đang chạy [PID: $pid]"
else
    echo "   ℹ dlt-receive chưa chạy (sẽ khởi động khi logger bắt đầu)"
fi

# Kiểm tra port listening
echo ""
echo "6. KIỂM TRA PORT LISTENING:"
echo "-----------------------------------------"
if netstat -tuln 2>/dev/null | grep -q ":3490"; then
    echo "   ✓ DLT daemon đang lắng nghe port 3490"
else
    echo "   ℹ Port 3490 chưa được lắng nghe"
fi

# Tóm tắt kết quả
echo ""
echo "========================================="
echo "TÓM TẮT:"
echo "========================================="
error_count=0
for service in "${services[@]}"; do
    if ! systemctl is-active --quiet "$service.service"; then
        ((error_count++))
    fi
done

if [ $error_count -eq 0 ]; then
    echo "✅ TẤT CẢ SERVICES ĐANG HOẠT ĐỘNG BÌNH THƯỜNG!"
else
    echo "⚠️  CÓ $error_count SERVICE KHÔNG HOẠT ĐỘNG!"
    echo "   Chạy lệnh sau để xem chi tiết:"
    echo "   journalctl -xe"
fi

echo ""
echo "========================================="
echo "CÁC LỆNH HỮU ÍCH:"
echo "========================================="
echo "Xem log files:"
echo "  /usr/local/bin/dlt-view-logs.sh"
echo ""
echo "Theo dõi log realtime:"
echo "  dlt-receive -a localhost"
echo ""
echo "Xem chi tiết service:"
echo "  systemctl status dlt-daemon.service -l"
echo "  systemctl status dlt-logger.service -l"
echo ""
echo "Xem system log nếu có lỗi:"
echo "  journalctl -u dlt-daemon -n 50"
echo "  journalctl -u dlt-logger -n 50"
echo "========================================="