#!/bin/bash

# Script cài đặt các dependencies cần thiết trước khi chạy setup.sh
# Chạy với quyền sudo: sudo ./install_dependencies.sh

# Kiểm tra quyền root
if [ "$EUID" -ne 0 ]; then
  echo "Vui lòng chạy script này với quyền sudo."
  exit 1
fi

echo "========================================================="
echo ">> Bắt đầu cài đặt các dependencies cho DLT và Boost..."
echo "========================================================="

# Cập nhật package list
echo "--> Cập nhật package list..."
sudo apt-get update

# Cài đặt DLT (Diagnostic Log and Trace)
echo "--> Cài đặt DLT daemon và development libraries..."
sudo apt-get install -y libdlt-dev dlt-daemon dlt-tools

# Kiểm tra DLT đã cài đặt thành công
if command -v dlt-daemon >/dev/null 2>&1; then
    echo "✓ DLT daemon đã được cài đặt thành công"
    dlt-daemon --version 2>/dev/null || echo "DLT version: $(dpkg -l | grep dlt-daemon | awk '{print $3}')"
else
    echo "✗ Lỗi: DLT daemon không được cài đặt"
    exit 1
fi

# Cài đặt Boost libraries
echo "--> Cài đặt Boost development libraries..."
sudo apt-get install -y libboost-all-dev

# Kiểm tra Boost đã cài đặt thành công
if [ -d "/usr/include/boost" ]; then
    echo "✓ Boost libraries đã được cài đặt thành công"
    BOOST_VERSION=$(dpkg -l | grep libboost-dev | head -n1 | awk '{print $3}')
    echo "Boost version: $BOOST_VERSION"
else
    echo "✗ Lỗi: Boost libraries không được cài đặt"
    exit 1
fi

#Cài đặt thư viện zlib
echo "--> Cài đặt thư viện zlib (cần cho DLT)..."
sudo apt-get install -y zlib1g-dev

# Kiểm tra zlib đã cài đặt thành công
if dpkg -s zlib1g-dev >/dev/null 2>&1; then
    echo "✓ zlib đã được cài đặt thành công"
else
    echo "✗ Lỗi: zlib không được cài đặt"
    exit 1
fi

# Tạo thư mục DLT temp nếu chưa có
echo "--> Tạo thư mục /tmp/dlt..."
sudo mkdir -p /tmp/dlt
sudo chmod 777 /tmp/dlt

echo "========================================================="
echo ">> Kiểm tra các command cần thiết..."
echo "========================================================="

# Kiểm tra các command cần thiết
COMMANDS=("dlt-daemon" "dlt-receive" "dlt-convert" "gcc" "g++")
ALL_OK=true

for cmd in "${COMMANDS[@]}"; do
    if command -v $cmd >/dev/null 2>&1; then
        echo "✓ $cmd: $(which $cmd)"
    else
        echo "✗ $cmd: KHÔNG TÌM THẤY"
        ALL_OK=false
    fi
done

# # Kiểm tra thư viện linking
# echo ""
# echo ">> Kiểm tra thư viện linking..."
# if pkg-config --exists dlt; then
#     echo "✓ DLT pkg-config: $(pkg-config --modversion dlt)"
#     echo "  - CFLAGS: $(pkg-config --cflags dlt)"
#     echo "  - LIBS: $(pkg-config --libs dlt)"
# else
#     echo "✗ DLT pkg-config không khả dụng"
#     echo "  Thử linking thủ công với: -ldlt"
# fi

# if [ -f "/usr/lib/aarch64-linux-gnu/libboost_thread.so" ] || [ -f "/usr/lib/arm-linux-gnueabihf/libboost_thread.so" ] || [ -f "/usr/lib/libboost_thread.so" ]; then
#     echo "✓ Boost thread library available"
#     echo "  - Link với: -lboost_thread -lboost_system"
# else
#     echo "✗ Boost thread library không tìm thấy"
#     ALL_OK=false
# fi

echo "========================================================="
if [ "$ALL_OK" = true ]; then
    echo "✓ TẤT CẢ DEPENDENCIES ĐÃ SẴN SÀNG!"
    echo ""
    echo "Bây giờ bạn có thể chạy:"
    echo "  sudo ./setup.sh"
    echo ""
    echo "Để compile C++ code với DLT và Boost, sử dụng:"
    echo "  g++ -o your_program your_program.cpp -ldlt -lboost_thread -lboost_system -lpthread"
else
    echo "✗ CÓ MỘT SỐ VẤN ĐỀ VỚI DEPENDENCIES!"
    echo "Vui lòng kiểm tra lại các lỗi ở trên."
    exit 1
fi

echo "========================================================="