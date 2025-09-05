#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <sstream>
#include <mutex>
#include <thread>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <cstring>
#include <dlt/dlt.h>
#include <map>
#include <memory>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <dirent.h>
#include <unistd.h>
#include <stack>

enum LogLevel { 
    LOG_LEVEL_INFO = DLT_LOG_INFO, 
    LOG_LEVEL_WARNING = DLT_LOG_WARN, 
    LOG_LEVEL_ERROR = DLT_LOG_ERROR 
};

class Logger;

// ========== ModuleLogger Class ==========
// Mỗi module (LibA, LibB) sẽ có một instance của ModuleLogger
// để tự động quản lý app_id và context
class ModuleLogger {
public:
    ModuleLogger(const std::string& app_id, const std::string& default_context = "");
    
    // Set context mặc định cho module
    void setDefaultContext(const std::string& context);
    
    // Tạo LogStream với app_id và context của module
    class LogStream {
    public:
        LogStream(const std::string& app_id, const std::string& context, 
                  LogLevel level, int line, const char* function);
        ~LogStream();
        
        template <typename T>
        LogStream& operator<<(const T& value) {
            m_stream << value;
            return *this;
        }
        
    private:
        std::string m_app_id;
        std::string m_context;
        LogLevel m_level;
        int m_line;
        const char* m_function;
        std::stringstream m_stream;
    };
    
    // Helper methods để tạo LogStream
    LogStream info(int line, const char* function, const std::string& context = "");
    LogStream warning(int line, const char* function, const std::string& context = "");
    LogStream error(int line, const char* function, const std::string& context = "");
    
private:
    std::string m_app_id;
    std::string m_default_context;
};

// ========== Main Logger Class (Singleton) ==========
class Logger {
public:
    static Logger& get_instance();
    
    // Đăng ký app và context
    bool register_app(const std::string& app_id, const std::string& app_description);
    bool register_context(const std::string& context_id, const std::string& context_description, 
                         const std::string& app_id = "");
    
    // Log với app_id và context cụ thể
    void log(LogLevel level, int line, const char* function, 
             const std::string& message,
             const std::string& app_id,
             const std::string& context_id);
    
    // Configuration
    void set_log_directory(const std::string& path);
    void enable_offline_logging(bool enable);
    void set_max_file_size(unsigned int size);
    void set_max_file_count(unsigned int count);
    
    // Lấy app hiện tại (để ModuleLogger có thể dùng)
    std::string get_current_registered_app() const { return m_last_registered_app; }

private:
    Logger();
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    struct AppContext {
        std::string app_id;
        std::string app_description;
        std::map<std::string, DltContext*> contexts;
        std::string default_context;
    };
    
    // Member variables
    std::map<std::string, AppContext*> m_apps;
    std::string m_last_registered_app;  // Track app vừa được đăng ký
    bool m_offline_logging_enabled;
    unsigned int m_max_file_size;
    unsigned int m_max_file_count;
    std::string m_log_directory;
    std::string m_current_trace_file;
    pid_t m_dlt_receiver_pid;
    mutable std::mutex m_mutex;
    
    // Helper functions
    std::string get_thread_id();
    void create_directory_if_not_exists(const std::string& path);
    void init_dlt_with_file_logging();
    void start_dlt_receiver();
    void stop_dlt_receiver();
    void rotate_log_file();
    void cleanup_old_logs();
    std::string get_current_date_as_string();
    std::string generate_trace_filename();
    
    DLT_DECLARE_CONTEXT(default_context);
};

// ========== MACRO HELPERS CHO MODULE ==========

// Macro để khởi tạo module logger trong class
#define DECLARE_MODULE_LOGGER() \
    private: \
        ModuleLogger m_logger;

// Macro để khởi tạo trong constructor
#define INIT_MODULE_LOGGER(app_id) \
    m_logger(app_id)

// Macros để log với module logger
#define MODULE_LOG_INFO \
    m_logger.info(__LINE__, __FUNCTION__)

#define MODULE_LOG_WARNING \
    m_logger.warning(__LINE__, __FUNCTION__)

#define MODULE_LOG_ERROR \
    m_logger.error(__LINE__, __FUNCTION__)

// Macros để log với context cụ thể
#define MODULE_LOG_INFO_CTX(ctx) \
    m_logger.info(__LINE__, __FUNCTION__, ctx)

#define MODULE_LOG_WARNING_CTX(ctx) \
    m_logger.warning(__LINE__, __FUNCTION__, ctx)

#define MODULE_LOG_ERROR_CTX(ctx) \
    m_logger.error(__LINE__, __FUNCTION__, ctx)

// ========== MACROS CHUNG CHO ĐĂNG KÝ ==========
#define LOG_REGISTER_APP(app_id, desc) \
    Logger::get_instance().register_app(app_id, desc)

#define LOG_REGISTER_CONTEXT(ctx_id, desc) \
    Logger::get_instance().register_context(ctx_id, desc)

// Đăng ký context cho app cụ thể
#define LOG_REGISTER_CONTEXT_FOR_APP(ctx_id, desc, app_id) \
    Logger::get_instance().register_context(ctx_id, desc, app_id)

// ========== GLOBAL LOG MACROS (không cần app_id) ==========
// Dùng cho main hoặc code không thuộc module nào
#define LOG_INFO \
    ModuleLogger::LogStream("MAIN", "DEFAULT", LOG_LEVEL_INFO, __LINE__, __FUNCTION__)

#define LOG_WARNING \
    ModuleLogger::LogStream("MAIN", "DEFAULT", LOG_LEVEL_WARNING, __LINE__, __FUNCTION__)

#define LOG_ERROR \
    ModuleLogger::LogStream("MAIN", "DEFAULT", LOG_LEVEL_ERROR, __LINE__, __FUNCTION__)

// Alias cho compatibility
using DLTLogger = Logger;

#endif // LOGGER_H