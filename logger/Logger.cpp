#include "Logger.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <sys/stat.h>
#include <cstring>  // For memset

namespace fs = std::filesystem;

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
    std::string prev_app = logger.get_current_app();
    std::string prev_ctx = logger.get_current_context();
    
    logger.set_current_app(app_id_);
    logger.set_current_context(default_context_);
    
    // Return by moving to avoid copy
    return LogStream(logger, level, file, line);
}

// ========================================
// Logger Singleton Implementation
// ========================================
Logger& Logger::get_instance() {
    static Logger instance;
    return instance;
}

Logger::Logger() 
    : log_directory_("./logger/log/"),
      max_file_count_(15),
      max_file_size_(100 * 1024 * 1024), // 100MB default
      offline_logging_enabled_(true),
      network_logging_enabled_(false),
      dlt_initialized_(false),
      daemon_ip_("0.0.0.0"),
      daemon_port_(3490) {
    
    // Create log directory if it doesn't exist
    fs::create_directories(log_directory_);
    
    // Initialize DLT
    init_dlt();
    
    // Clean up old logs on startup
    cleanup_old_logs();
}

Logger::~Logger() {
    cleanup_dlt();
}

bool Logger::init_dlt() {
    if (dlt_initialized_) {
        return true;
    }
    
    // Configure DLT for network mode if enabled
    if (network_logging_enabled_) {
        // Set verbose mode to see all log levels
        dlt_verbose_mode();
        
        // Enable network trace via environment variables
        setenv("DLT_INITIAL_LOG_LEVEL", "6", 1); // Set to verbose
        setenv("DLT_LOG_MODE", "2", 1); // 2 = both file and network
    }
    
    // Register application with DLT daemon
    DLT_REGISTER_APP("AGVS", "AGV System Logger");
    
    // Set log level and trace status for the application
    dlt_set_application_ll_ts_limit(DLT_LOG_DEBUG, DLT_TRACE_STATUS_ON);
    
    // Enable offline logging if configured
    if (offline_logging_enabled_) {
        std::string log_file = generate_log_filename();
        current_log_file_ = log_directory_ + log_file;
        
        // Create offline trace file path
        std::string offline_trace_file = log_directory_ + "offline_trace.txt";
        
        // Note: Standard DLT doesn't have direct offline trace API
        // Offline logging is handled by the DLT daemon configuration
        // The daemon should be configured to save logs to files
        
        std::cout << "DLT logging enabled. Files will be saved to: " << log_directory_ << std::endl;
        std::cout << "Configure DLT daemon for offline tracing in /etc/dlt.conf" << std::endl;
    }
    
    // Enable network logging if configured
    if (network_logging_enabled_) {
        std::cout << "DLT network logging enabled - accessible via DLT Viewer" << std::endl;
        std::cout << "Connect DLT Viewer to: " << daemon_ip_ << ":" << daemon_port_ << std::endl;
    }
    
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
    
    // Mark as registered
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
    
    // Create combined context ID for DLT (4 chars max)
    std::string dlt_ctx_id = context_id.substr(0, 4);
    
    // Register with DLT using standard API
    DLT_REGISTER_CONTEXT(ctx_info->handle, dlt_ctx_id.c_str(), description.c_str());
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
    
    // Find context
    auto app_it = contexts_.find(app_id);
    if (app_it == contexts_.end()) {
        // Auto-register app if not exists
        register_app(app_id, app_id + " Application");
        register_context(app_id, context_id, context_id + " Context");
        app_it = contexts_.find(app_id);
    }
    
    auto ctx_it = app_it->second.find(context_id);
    if (ctx_it == app_it->second.end()) {
        // Auto-register context if not exists
        register_context(app_id, context_id, context_id + " Context");
        ctx_it = app_it->second.find(context_id);
    }
    
    if (ctx_it == app_it->second.end() || !ctx_it->second->registered) {
        std::cerr << "Context not registered: " << app_id << "/" << context_id << std::endl;
        return;
    }
    
    // Prepare log message with file and line info if available
    std::string log_msg = message;
    if (file != nullptr && line > 0) {
        // Extract just the filename from the full path
        std::string filename(file);
        size_t last_slash = filename.find_last_of("/\\");
        if (last_slash != std::string::npos) {
            filename = filename.substr(last_slash + 1);
        }
        log_msg = "[" + filename + ":" + std::to_string(line) + "] " + message;
    }
    
    // Log to DLT
    DLT_LOG(ctx_it->second->handle, 
            static_cast<DltLogLevelType>(level),
            DLT_STRING(log_msg.c_str()));
    
    // Also log to console in debug mode
    #ifdef DEBUG
    const char* level_str = "";
    switch(level) {
        case LogLevel::VERBOSE: level_str = "VERBOSE"; break;
        case LogLevel::DEBUG: level_str = "DEBUG"; break;
        case LogLevel::INFO: level_str = "INFO"; break;
        case LogLevel::WARNING: level_str = "WARNING"; break;
        case LogLevel::ERROR: level_str = "ERROR"; break;
        case LogLevel::FATAL: level_str = "FATAL"; break;
    }
    
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::cout << "[" << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    std::cout << "." << std::setfill('0') << std::setw(3) << ms.count() << "] ";
    std::cout << "[" << app_id << "/" << context_id << "] ";
    std::cout << "[" << level_str << "] " << log_msg << std::endl;
    #endif
    
    // Check if we need to rotate logs (manual rotation for file-based logging)
    rotate_logs();
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

void Logger::set_log_directory(const std::string& dir) {
    std::lock_guard<std::mutex> lock(mutex_);
    log_directory_ = dir;
    if (log_directory_.back() != '/') {
        log_directory_ += '/';
    }
    fs::create_directories(log_directory_);
}

void Logger::set_max_file_count(int count) {
    std::lock_guard<std::mutex> lock(mutex_);
    max_file_count_ = count;
}

void Logger::set_max_file_size(size_t size_mb) {
    std::lock_guard<std::mutex> lock(mutex_);
    max_file_size_ = size_mb * 1024 * 1024; // Convert MB to bytes
}

void Logger::enable_offline_logging(bool enable) {
    std::lock_guard<std::mutex> lock(mutex_);
    offline_logging_enabled_ = enable;
    
    if (dlt_initialized_) {
        if (enable) {
            std::cout << "Offline logging enabled. Logs will be saved via DLT daemon." << std::endl;
        } else {
            std::cout << "Offline logging disabled." << std::endl;
        }
    }
}

void Logger::enable_network_logging(bool enable, const std::string& daemon_ip, int port) {
    std::lock_guard<std::mutex> lock(mutex_);
    network_logging_enabled_ = enable;
    daemon_ip_ = daemon_ip;
    daemon_port_ = port;
    
    if (dlt_initialized_ && enable) {
        std::cout << "Network logging enabled. DLT Viewer can connect to: " 
                  << daemon_ip_ << ":" << daemon_port_ << std::endl;
    }
}

void Logger::set_log_mode(int mode) {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // Set DLT log mode
    // 0 = OFF, 1 = EXTERNAL (network only), 2 = BOTH (file and network)
    
    if (mode == 2) { // BOTH mode
        enable_offline_logging(true);
        network_logging_enabled_ = true;
    } else if (mode == 1) { // EXTERNAL mode (network only)
        enable_offline_logging(false);
        network_logging_enabled_ = true;
    } else { // OFF mode
        enable_offline_logging(false);
        network_logging_enabled_ = false;
    }
}

std::string Logger::generate_log_filename() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << "trace_" << std::put_time(std::localtime(&time_t), "%Y%m%d") << ".dlt";
    return ss.str();
}

void Logger::rotate_logs() {
    // Manual log rotation based on file size
    // This is a simplified version since DLT handles its own log files
    
    // Check if current log file exists and its size
    if (!current_log_file_.empty() && fs::exists(current_log_file_)) {
        auto file_size = fs::file_size(current_log_file_);
        
        // If file exceeds max size, create a new one
        if (file_size >= max_file_size_) {
            // Generate new filename with timestamp
            auto now = std::chrono::system_clock::now();
            auto time_t = std::chrono::system_clock::to_time_t(now);
            
            std::stringstream ss;
            ss << "trace_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S") << ".dlt";
            current_log_file_ = log_directory_ + ss.str();
            
            std::cout << "Log rotated to: " << current_log_file_ << std::endl;
            
            // Clean up old files
            cleanup_old_logs();
        }
    }
}

void Logger::cleanup_old_logs() {
    std::vector<fs::path> log_files;
    
    // Collect all .dlt files in the log directory
    try {
        for (const auto& entry : fs::directory_iterator(log_directory_)) {
            if (entry.path().extension() == ".dlt") {
                log_files.push_back(entry.path());
            }
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error accessing log directory: " << e.what() << std::endl;
        return;
    }
    
    // Sort files by last write time (oldest first)
    std::sort(log_files.begin(), log_files.end(),
              [](const fs::path& a, const fs::path& b) {
                  return fs::last_write_time(a) < fs::last_write_time(b);
              });
    
    // Remove oldest files if we exceed max count
    while (log_files.size() > static_cast<size_t>(max_file_count_)) {
        try {
            fs::remove(log_files.front());
            std::cout << "Removed old log file: " << log_files.front() << std::endl;
            log_files.erase(log_files.begin());
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Error removing log file: " << e.what() << std::endl;
            break;
        }
    }
}