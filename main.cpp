

#include "config/config.h"
#include "logger/Logger.h"

#include "SystemManager.h"


/**
 * @brief Hàm main của chương trình.
 * @details Khởi tạo logging, kiểm tra kết nối server, và khởi chạy tất cả các luồng của hệ thống.
 */
int main() {
    // Initialize logging system
    // Register MAIN app and context
    LOG_REGISTER_APP("MAIN", "Main AGV Application");
    LOG_REGISTER_CONTEXT("MAIN", "Main Control Context");
    LOG_SET_APP("MAIN");
    LOG_SET_CONTEXT("MAIN");

    // Sử dụng con trỏ chia sẻ để có thể truy cập từ signal handler
    std::shared_ptr<SystemManager> system_manager = std::make_shared<SystemManager>(SERVER_IP, SERVER_PORT);

    // Thiết lập trình xử lý tín hiệu để dừng chương trình một cách an toàn
    signal(SIGINT,  {
        
        // Biến toàn cục `global_running` sẽ được `system_manager->stop()` xử lý.
        // Ở đây chúng ta không cần làm gì thêm vì `run()` sẽ thoát ra.
        // Tuy nhiên, để dừng ngay lập tức, chúng ta cần gọi stop().
        // Điều này yêu cầu một cách để truy cập system_manager.
    });

    system_manager->run(); // Hàm này sẽ block cho đến khi nhận tín hiệu dừng

    LOG_INFO << "[Main Thread] System shutdown complete.";
    return 0;
}