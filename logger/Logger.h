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

enum LogLevel { 
    LOG_LEVEL_INFO = DLT_LOG_INFO, 
    LOG_LEVEL_WARNING = DLT_LOG_WARN, 
    LOG_LEVEL_ERROR = DLT_LOG_ERROR 
};

class Logger;

class LogStream {
public:
    LogStream(Logger& logger, LogLevel level, int line, const char* function);
    ~LogStream();

    template <typename T>
    LogStream& operator<<(const T& value) {
        m_stream << value;
        return *this;
    }

private:
    Logger& m_logger_ref;
    LogLevel level;
    std::stringstream m_stream;
    int m_line;
    const char* m_function;
};

// Sử dụng tên Logger thay vì DLTLogger để giữ compatibility
class Logger {
public:
    static Logger& get_instance();
    
    bool register_app(const std::string& app_id, const std::string& app_description);
    bool register_context(const std::string& context_id, const std::string& context_description);
    void set_current_app(const std::string& app_id);
    void set_current_context(const std::string& context_id);
    void log(LogLevel level, int line, const char* function, const std::string& message);
    void set_log_directory(const std::string& path);
    void enable_offline_logging(bool enable);
    void set_max_file_size(unsigned int size);
    void set_max_file_count(unsigned int count);

private:
    Logger();
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    // DLT contexts cho mỗi app
    struct AppContext {
        std::string app_id;
        std::string app_description;
        std::map<std::string, DltContext*> contexts;
        std::string current_context;
    };
    
    // Member variables
    std::map<std::string, AppContext*> m_apps;
    std::string m_current_app;
    bool m_offline_logging_enabled;
    unsigned int m_max_file_size;
    unsigned int m_max_file_count;
    std::string m_log_directory;
    std::string m_current_trace_file;
    pid_t m_dlt_receiver_pid;
    std::mutex m_mutex;
    
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
    
    // DLT context mặc định
    DLT_DECLARE_CONTEXT(default_context);
};

// Macro helpers
#define LOG_INFO LogStream(Logger::get_instance(), LOG_LEVEL_INFO, __LINE__, __FUNCTION__)
#define LOG_WARNING LogStream(Logger::get_instance(), LOG_LEVEL_WARNING, __LINE__, __FUNCTION__)
#define LOG_ERROR LogStream(Logger::get_instance(), LOG_LEVEL_ERROR, __LINE__, __FUNCTION__)

#define LOG_REGISTER_APP(app_id, desc) Logger::get_instance().register_app(app_id, desc)
#define LOG_REGISTER_CONTEXT(ctx_id, desc) Logger::get_instance().register_context(ctx_id, desc)
#define LOG_SET_APP(app_id) Logger::get_instance().set_current_app(app_id)
#define LOG_SET_CONTEXT(ctx_id) Logger::get_instance().set_current_context(ctx_id)

// Alias cho compatibility nếu có code dùng DLTLogger
using DLTLogger = Logger;

#endif // LOGGER_H