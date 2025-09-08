#ifndef LOGGER_H
#define LOGGER_H

#include <dlt/dlt.h>
#include <string>
#include <sstream>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <chrono>

// Log levels matching DLT
enum class LogLevel {
    VERBOSE = DLT_LOG_VERBOSE,
    DEBUG = DLT_LOG_DEBUG,
    INFO = DLT_LOG_INFO,
    WARNING = DLT_LOG_WARN,
    ERROR = DLT_LOG_ERROR,
    FATAL = DLT_LOG_FATAL
};

// Forward declaration
class Logger;

// Log stream class for convenient << operator usage
class LogStream {
public:
    LogStream(Logger& logger, LogLevel level, const char* file, int line);
    ~LogStream();
    
    template<typename T>
    LogStream& operator<<(const T& value) {
        buffer_ << value;
        return *this;
    }
    
private:
    Logger& logger_;
    LogLevel level_;
    std::stringstream buffer_;
    const char* file_;
    int line_;
};

// Module logger wrapper for automatic context management
class ModuleLogger {
public:
    ModuleLogger(const std::string& app_id);
    
    void setDefaultContext(const std::string& context_id);
    LogStream log(LogLevel level, const char* file, int line);
    
private:
    std::string app_id_;
    std::string default_context_;
};

// Main Logger class - Singleton pattern
class Logger {
public:
    // Singleton access
    static Logger& get_instance();
    
    // Delete copy constructor and assignment operator
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    
    // Application and context registration
    bool register_app(const std::string& app_id, const std::string& description);
    bool register_context(const std::string& app_id, const std::string& context_id, const std::string& description);
    
    // Logging functions
    void log(const std::string& app_id, const std::string& context_id, 
             LogLevel level, const std::string& message,
             const char* file = nullptr, int line = 0);
    
    // Get current app/context for thread
    void set_current_app(const std::string& app_id);
    void set_current_context(const std::string& context_id);
    std::string get_current_app() const;
    std::string get_current_context() const;
    
    // Check if DLT is initialized
    bool is_initialized() const { return dlt_initialized_; }
    
private:
    Logger();
    ~Logger();
    
    // DLT context management
    struct ContextInfo {
        DLT_DECLARE_CONTEXT(handle)
        std::string app_id;
        std::string context_id;
        std::string description;
        bool registered = false;
    };
    
    // Initialize DLT
    bool init_dlt();
    void cleanup_dlt();
    
    // Thread-local storage for current app/context
    static thread_local std::string current_app_;
    static thread_local std::string current_context_;
    
    // Members
    std::mutex mutex_;
    std::unordered_map<std::string, std::unordered_map<std::string, std::unique_ptr<ContextInfo>>> contexts_;
    std::unordered_map<std::string, bool> registered_apps_;
    bool dlt_initialized_;
};

// Convenience macros for module usage
#define DECLARE_MODULE_LOGGER() \
    private: \
        mutable ModuleLogger m_logger;

#define INIT_MODULE_LOGGER(app_id) \
    m_logger(app_id)

#define MODULE_LOG_VERBOSE \
    m_logger.log(LogLevel::VERBOSE, __FILE__, __LINE__)

#define MODULE_LOG_DEBUG \
    m_logger.log(LogLevel::DEBUG, __FILE__, __LINE__)

#define MODULE_LOG_INFO \
    m_logger.log(LogLevel::INFO, __FILE__, __LINE__)

#define MODULE_LOG_WARNING \
    m_logger.log(LogLevel::WARNING, __FILE__, __LINE__)

#define MODULE_LOG_ERROR \
    m_logger.log(LogLevel::ERROR, __FILE__, __LINE__)

#define MODULE_LOG_FATAL \
    m_logger.log(LogLevel::FATAL, __FILE__, __LINE__)

// Global logging macros
#define LOG_VERBOSE \
    LogStream(Logger::get_instance(), LogLevel::VERBOSE, __FILE__, __LINE__)

#define LOG_DEBUG \
    LogStream(Logger::get_instance(), LogLevel::DEBUG, __FILE__, __LINE__)

#define LOG_INFO \
    LogStream(Logger::get_instance(), LogLevel::INFO, __FILE__, __LINE__)

#define LOG_WARNING \
    LogStream(Logger::get_instance(), LogLevel::WARNING, __FILE__, __LINE__)

#define LOG_ERROR \
    LogStream(Logger::get_instance(), LogLevel::ERROR, __FILE__, __LINE__)

#define LOG_FATAL \
    LogStream(Logger::get_instance(), LogLevel::FATAL, __FILE__, __LINE__)

// Registration helper macros
#define LOG_REGISTER_APP(app_id, desc) \
    Logger::get_instance().register_app(app_id, desc)

#define LOG_REGISTER_CONTEXT(ctx_id, desc) \
    Logger::get_instance().register_context(Logger::get_instance().get_current_app(), ctx_id, desc)

#define LOG_SET_APP(app_id) \
    Logger::get_instance().set_current_app(app_id)

#define LOG_SET_CONTEXT(ctx_id) \
    Logger::get_instance().set_current_context(ctx_id)

#endif // LOGGER_H