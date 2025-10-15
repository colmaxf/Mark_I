#!/bin/bash

# Script cài đặt các dependencies cần thiết trước khi chạy setup_agv.sh
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

# Cài đặt tất cả các gói cần thiết trong một lệnh
echo "--> Cài đặt các gói cần thiết (build-essential, git, cmake, boost, dlt, và các thư viện khác)..."
sudo apt-get install -y \
    build-essential \
    git \
    cmake \
    ninja-build \
    clang \
    g++ \
    python3-sphinx \
    lsb-release \
    stow \
    wget \
    unzip \
    libboost-all-dev \
    libcairo2-dev \
    libcurl4-openssl-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    liblua5.3-dev \
    libsuitesparse-dev \
    libprotobuf-dev \
    google-mock \
    libatlas-base-dev \
    libdlt-dev \
    dlt-daemon \
    dlt-tools \
    zlib1g-dev

echo "✓ Quá trình cài đặt gói đã hoàn tất."

# Tạo thư mục DLT temp nếu chưa có
echo "--> Tạo thư mục /tmp/dlt..."
sudo mkdir -p /tmp/dlt
sudo chmod 777 /tmp/dlt

echo "========================================================="
echo ">> Kiểm tra các command cần thiết..."
echo "========================================================="

# Kiểm tra các command cần thiết
COMMANDS=("dlt-daemon" "dlt-receive" "dlt-convert" "gcc" "g++" "git" "cmake" "clang")
ALL_OK=true

for cmd in "${COMMANDS[@]}"; do
    if command -v $cmd >/dev/null 2>&1; then
        echo "✓ $cmd: $(which $cmd)"
    else
        echo "✗ $cmd: KHÔNG TÌM THẤY"
        ALL_OK=false
    fi
done

# Kiểm tra các thư viện chính
LIBRARIES=("libboost-dev" "libdlt-dev" "zlib1g-dev" "libeigen3-dev" "libprotobuf-dev")
for lib in "${LIBRARIES[@]}"; do
    if dpkg -s $lib >/dev/null 2>&1; then
        echo "✓ Thư viện $lib đã được cài đặt."
    else
        echo "✗ Thư viện $lib: KHÔNG TÌM THẤY"
        ALL_OK=false
    fi
done

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