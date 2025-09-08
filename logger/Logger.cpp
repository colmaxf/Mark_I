#include "Logger.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <cstring>

// Thread-local storage initialization
thread_local std::string Logger::current_app_ = "MAIN";
thread_local std::string Logger::current_context_ = "MAIN";

// ========================================
// LogStream Implementation
// ========================================
LogStream::LogStream(Logger& logger, LogLevel level, const char* file, int line)
    : logger_(logger), level_(level), file_(file), line_(line) {
}

LogStream::~LogStream() {
    std::string message = buffer_.str();
    if (!message.empty()) {
        logger_.log(logger_.get_current_app(), 
                   logger_.get_current_context(),
                   level_, message, file_, line_);
    }
}

// ========================================
// ModuleLogger Implementation
// ========================================
ModuleLogger::ModuleLogger(const std::string& app_id) 
    : app_id_(app_id), default_context_("CORE") {
}

void ModuleLogger::setDefaultContext(const std::string& context_id) {
    default_context_ = context_id;
}

LogStream ModuleLogger::log(LogLevel level, const char* file, int line) {
    Logger& logger = Logger::get_instance();
    
    // Temporarily set app and context for this log
    logger.set_current_app(app_id_);
    logger.set_current_context(default_context_);
    
    return LogStream(logger, level, file, line);
}

// ========================================
// Logger Singleton Implementation
// ========================================
Logger& Logger::get_instance() {
static Logger* instance = nullptr;
    static std::once_flag init_flag;
    
    std::call_once(init_flag, []() {
        instance = new Logger();
    });
    
    return *instance;
}

Logger::Logger() : dlt_initialized_(false) {
    // Initialize DLT
    init_dlt();
}

Logger::~Logger() {
    cleanup_dlt();
}

bool Logger::init_dlt() {
    if (dlt_initialized_) {
        return true;
    }
    
    // DLT only allows 1 APP ID per process
    // We use different CONTEXT IDs to distinguish modules
    DLT_REGISTER_APP("AGVS", "AGV System - All Modules");
    
    // Set log level and trace status
    dlt_set_application_ll_ts_limit(DLT_LOG_DEBUG, DLT_TRACE_STATUS_ON);
    
    // Configure for network mode
    dlt_verbose_mode();
    
    std::cout << "========================================" << std::endl;
    std::cout << "DLT Logger Initialized" << std::endl;
    std::cout << "Network: Port 3490 (for DLT Viewer)" << std::endl;
    std::cout << "File logging handled by dlt-logger service" << std::endl;
    std::cout << "Log files: /var/log/dlt/agv_*.dlt" << std::endl;
    std::cout << "========================================" << std::endl;
    
    dlt_initialized_ = true;
    return true;
}

void Logger::cleanup_dlt() {
    if (!dlt_initialized_) {
        return;
    }
    
    // Unregister all contexts
    for (auto& app_pair : contexts_) {
        for (auto& ctx_pair : app_pair.second) {
            if (ctx_pair.second->registered) {
                DLT_UNREGISTER_CONTEXT(ctx_pair.second->handle);
            }
        }
    }
    
    // Unregister application
    DLT_UNREGISTER_APP();
    
    dlt_initialized_ = false;
}

bool Logger::register_app(const std::string& app_id, const std::string&) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (registered_apps_.find(app_id) != registered_apps_.end()) {
        return true; // Already registered
    }
    
    // Mark as registered (for internal tracking)
    registered_apps_[app_id] = true;
    
    // Set as current app for this thread
    current_app_ = app_id;
    
    return true;
}

bool Logger::register_context(const std::string& app_id, 
                              const std::string& context_id, 
                              const std::string& description) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Check if context already exists
    if (contexts_.find(app_id) != contexts_.end() &&
        contexts_[app_id].find(context_id) != contexts_[app_id].end()) {
        return true; // Already registered
    }
    
    // Create new context info
    auto ctx_info = std::make_unique<ContextInfo>();
    ctx_info->app_id = app_id;
    ctx_info->context_id = context_id;
    ctx_info->description = description;
    
    // Create Context ID combining APP and CONTEXT to distinguish modules
    // Examples: MAIN -> MN, LIDA -> LD, MCPR -> MC
    std::string dlt_ctx_id;
    if (app_id == "MAIN") {
        dlt_ctx_id = "MN" + context_id.substr(0, 2);
    } else if (app_id == "LIDA" || app_id == "LIDAR") {
        dlt_ctx_id = "LD" + context_id.substr(0, 2);
    } else if (app_id == "MCPR") {
        dlt_ctx_id = "MC" + context_id.substr(0, 2);
    } else {
        // Default: first 2 chars of app + first 2 chars of context
        dlt_ctx_id = app_id.substr(0, 2) + context_id.substr(0, 2);
    }
    
    // Ensure only 4 characters (DLT limit)
    dlt_ctx_id = dlt_ctx_id.substr(0, 4);
    
    // Register with DLT using combined description
    std::string full_desc = "[" + app_id + "/" + context_id + "] " + description;
    DLT_REGISTER_CONTEXT(ctx_info->handle, dlt_ctx_id.c_str(), full_desc.c_str());
    ctx_info->registered = true;
    
    // Set default log level
    dlt_set_application_ll_ts_limit(DLT_LOG_DEBUG, DLT_TRACE_STATUS_ON);
    
    // Store context info
    contexts_[app_id][context_id] = std::move(ctx_info);
    
    return true;
}

void Logger::log(const std::string& app_id, const std::string& context_id,
                LogLevel level, const std::string& message,
                const char* file, int line) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Auto-register if needed
    auto app_it = contexts_.find(app_id);
    if (app_it == contexts_.end()) {
        register_app(app_id, app_id + " Application");
        register_context(app_id, context_id, context_id + " Context");
        app_it = contexts_.find(app_id);
    }
    
    auto ctx_it = app_it->second.find(context_id);
    if (ctx_it == app_it->second.end()) {
        register_context(app_id, context_id, context_id + " Context");
        ctx_it = app_it->second.find(context_id);
    }
    
    if (ctx_it == app_it->second.end() || !ctx_it->second->registered) {
        std::cerr << "Context not registered: " << app_id << "/" << context_id << std::endl;
        return;
    }
    
    // Prepare log message with metadata
    std::stringstream log_msg;
    
    // Add file and line info if available
    if (file != nullptr && line > 0) {
        std::string filename(file);
        size_t last_slash = filename.find_last_of("/\\");
        if (last_slash != std::string::npos) {
            filename = filename.substr(last_slash + 1);
        }
        log_msg << "[" << filename << ":" << line << "] ";
    }
    
    // Add app/context info for clarity in DLT viewer
    log_msg << "[" << app_id << "/" << context_id << "] " << message;
    
    // Log to DLT - will be captured by:
    // 1. DLT Viewer via network (port 3490)
    // 2. dlt-logger service to file (/var/log/dlt/agv_*.dlt)
    DLT_LOG(ctx_it->second->handle, 
            static_cast<DltLogLevelType>(level),
            DLT_STRING(log_msg.str().c_str()));
    
    // Console output for debugging (only warnings and errors in release)
    #ifdef DEBUG
    // In debug mode, show all logs
    const char* level_str = "";
    const char* color_code = "";
    const char* reset_code = "\033[0m";
    
    switch(level) {
        case LogLevel::VERBOSE: 
            level_str = "VERBOSE"; 
            color_code = "\033[37m"; // White
            break;
        case LogLevel::DEBUG: 
            level_str = "DEBUG"; 
            color_code = "\033[36m"; // Cyan
            break;
        case LogLevel::INFO: 
            level_str = "INFO"; 
            color_code = "\033[32m"; // Green
            break;
        case LogLevel::WARNING: 
            level_str = "WARNING"; 
            color_code = "\033[33m"; // Yellow
            break;
        case LogLevel::ERROR: 
            level_str = "ERROR"; 
            color_code = "\033[31m"; // Red
            break;
        case LogLevel::FATAL: 
            level_str = "FATAL"; 
            color_code = "\033[35m"; // Magenta
            break;
    }
    
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::cout << color_code
              << "[" << std::put_time(std::localtime(&time_t), "%H:%M:%S");
    std::cout << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
    std::cout << "[" << level_str << "] " << log_msg.str() 
              << reset_code << std::endl;
    #else
    // In release mode, only show warnings and errors
    if (level >= LogLevel::WARNING) {
    const char* level_str = "";
    const char* color = "";
    
    switch(level) {
        case LogLevel::WARNING: 
            level_str = "WARN";
            color = "\033[33m";  // Yellow
            break;
        case LogLevel::ERROR:
            level_str = "ERROR";
            color = "\033[31m";  // Red
            break;
        case LogLevel::FATAL:
            level_str = "FATAL";
            color = "\033[35m";  // Magenta
            break;
        default:
            level_str = "INFO";   // Nếu không phải WARNING/ERROR/FATAL
            color = "\033[32m";   // Green
            break;
    }
    
    std::cerr << color << "[" << level_str << "] " 
              << log_msg.str() << "\033[0m" << std::endl;
    }
    #endif
}

void Logger::set_current_app(const std::string& app_id) {
    current_app_ = app_id;
}

void Logger::set_current_context(const std::string& context_id) {
    current_context_ = context_id;
}

std::string Logger::get_current_app() const {
    return current_app_;
}

std::string Logger::get_current_context() const {
    return current_context_;
}