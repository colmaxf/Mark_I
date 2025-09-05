#include "Logger.h"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <string>
#include <vector>
#include <algorithm>
#include <cstdio>
#include <dirent.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <signal.h>

// Constants
const unsigned int DEFAULT_MAX_FILE_SIZE = 10 * 1024 * 1024;
const unsigned int DEFAULT_MAX_FILE_COUNT = 5;
const std::string LOG_FILE_PREFIX = "trace_";
const std::string LOG_FILE_SUFFIX = ".dlt";

// ========== ModuleLogger Implementation ==========
ModuleLogger::ModuleLogger(const std::string& app_id, const std::string& default_context)
    : m_app_id(app_id), m_default_context(default_context.empty() ? "DEFAULT" : default_context) {
}

void ModuleLogger::setDefaultContext(const std::string& context) {
    m_default_context = context;
}

ModuleLogger::LogStream ModuleLogger::info(int line, const char* function, const std::string& context) {
    return LogStream(m_app_id, 
                     context.empty() ? m_default_context : context,
                     LOG_LEVEL_INFO, line, function);
}

ModuleLogger::LogStream ModuleLogger::warning(int line, const char* function, const std::string& context) {
    return LogStream(m_app_id,
                     context.empty() ? m_default_context : context,
                     LOG_LEVEL_WARNING, line, function);
}

ModuleLogger::LogStream ModuleLogger::error(int line, const char* function, const std::string& context) {
    return LogStream(m_app_id,
                     context.empty() ? m_default_context : context,
                     LOG_LEVEL_ERROR, line, function);
}

// ========== ModuleLogger::LogStream Implementation ==========
ModuleLogger::LogStream::LogStream(const std::string& app_id, const std::string& context,
                                    LogLevel level, int line, const char* function)
    : m_app_id(app_id), m_context(context), m_level(level), 
      m_line(line), m_function(function) {
}

ModuleLogger::LogStream::~LogStream() {
    Logger::get_instance().log(m_level, m_line, m_function, 
                                m_stream.str(), m_app_id, m_context);
}

// ========== Logger Implementation ==========
Logger::Logger() 
    : m_offline_logging_enabled(true),
      m_max_file_size(DEFAULT_MAX_FILE_SIZE),
      m_max_file_count(DEFAULT_MAX_FILE_COUNT),
      m_log_directory("./logger/log/"),
      m_dlt_receiver_pid(0) {
    
    create_directory_if_not_exists(m_log_directory);
    init_dlt_with_file_logging();
    DLT_REGISTER_CONTEXT(default_context, "DEF", "Default Context");
}

Logger::~Logger() {
    stop_dlt_receiver();
    
    for (auto& app_pair : m_apps) {
        for (auto& ctx_pair : app_pair.second->contexts) {
            dlt_unregister_context(ctx_pair.second);
            delete ctx_pair.second;
        }
        delete app_pair.second;
    }
    
    DLT_UNREGISTER_CONTEXT(default_context);
    DLT_UNREGISTER_APP();
}

Logger& Logger::get_instance() {
    static Logger instance;
    return instance;
}

bool Logger::register_app(const std::string& app_id, const std::string& app_description) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (m_apps.find(app_id) != m_apps.end()) {
        return false;  // Already registered
    }
    
    if (dlt_register_app(app_id.c_str(), app_description.c_str()) != DLT_RETURN_OK) {
        std::cerr << "Logger Error: Failed to register DLT app: " << app_id << std::endl;
        return false;
    }
    
    AppContext* app = new AppContext();
    app->app_id = app_id;
    app->app_description = app_description;
    app->default_context = "DEFAULT";
    
    m_apps[app_id] = app;
    m_last_registered_app = app_id;  // Lưu lại app vừa đăng ký
    
    return true;
}

bool Logger::register_context(const std::string& context_id, const std::string& context_description,
                               const std::string& app_id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    // Sử dụng app_id được truyền vào hoặc app được đăng ký gần nhất
    std::string target_app = app_id.empty() ? m_last_registered_app : app_id;
    
    if (target_app.empty()) {
        std::cerr << "Logger Error: No app registered yet" << std::endl;
        return false;
    }
    
    auto app_it = m_apps.find(target_app);
    if (app_it == m_apps.end()) {
        std::cerr << "Logger Error: App not found: " << target_app << std::endl;
        return false;
    }
    
    if (app_it->second->contexts.find(context_id) != app_it->second->contexts.end()) {
        return false;  // Context already exists
    }
    
    DltContext* new_context = new DltContext();
    if (!new_context) {
        return false;
    }

    if (dlt_register_context(new_context, context_id.c_str(), context_description.c_str()) != DLT_RETURN_OK) {
        delete new_context;
        std::cerr << "Logger Error: Failed to register DLT context: " << context_id << std::endl;
        return false;
    }
    
    app_it->second->contexts[context_id] = new_context;
    
    // Set as default context if it's the first one
    if (app_it->second->contexts.size() == 1) {
        app_it->second->default_context = context_id;
    }
    
    return true;
}

void Logger::log(LogLevel level, int line, const char* function, 
                 const std::string& message,
                 const std::string& app_id,
                 const std::string& context_id) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    DltContext* ctx_ptr = nullptr;
    
    // Find the appropriate DLT context
    auto app_it = m_apps.find(app_id);
    if (app_it != m_apps.end()) {
        // Try to find the specific context
        auto ctx_it = app_it->second->contexts.find(context_id);
        if (ctx_it != app_it->second->contexts.end()) {
            ctx_ptr = ctx_it->second;
        } else {
            // Use default context of the app if specific context not found
            ctx_it = app_it->second->contexts.find(app_it->second->default_context);
            if (ctx_it != app_it->second->contexts.end()) {
                ctx_ptr = ctx_it->second;
            }
        }
    }
    
    // Use default context if no suitable context found
    if (!ctx_ptr) {
        ctx_ptr = &default_context;
    }

    DltLogLevelType dlt_level = static_cast<DltLogLevelType>(level);
    std::string thread_id = get_thread_id();
    
    // Log to DLT
    DLT_LOG(*ctx_ptr, dlt_level, 
        DLT_STRING(message.c_str()),
        DLT_STRING("func"), DLT_STRING(function),
        DLT_STRING("line"), DLT_INT(line),
        DLT_STRING("tid"), DLT_STRING(thread_id.c_str()),
        DLT_STRING("app"), DLT_STRING(app_id.c_str()),
        DLT_STRING("ctx"), DLT_STRING(context_id.c_str())
    );
    
    rotate_log_file();
}

void Logger::set_log_directory(const std::string& path) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    stop_dlt_receiver();
    
    m_log_directory = path;
    
    if (!m_log_directory.empty() && m_log_directory.back() == '/') {
        m_log_directory.pop_back();
    }
    
    if (!m_log_directory.empty()) {
        create_directory_if_not_exists(m_log_directory);
        
        if (m_offline_logging_enabled) {
            start_dlt_receiver();
        }
    }
}

void Logger::enable_offline_logging(bool enable) {
    std::lock_guard<std::mutex> lock(m_mutex);
    
    if (m_offline_logging_enabled == enable) {
        return;
    }
    
    m_offline_logging_enabled = enable;
    
    if (enable) {
        start_dlt_receiver();
    } else {
        stop_dlt_receiver();
    }
}

void Logger::set_max_file_size(unsigned int size) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_max_file_size = size;
}

void Logger::set_max_file_count(unsigned int count) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_max_file_count = count;
    cleanup_old_logs();
}

std::string Logger::get_thread_id() {
    std::stringstream ss;
    ss << std::this_thread::get_id();
    return ss.str();
}

std::string Logger::get_current_date_as_string() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y_%m_%d_%H%M%S");
    return ss.str();
}

std::string Logger::generate_trace_filename() {
    return LOG_FILE_PREFIX + get_current_date_as_string() + LOG_FILE_SUFFIX;
}

void Logger::create_directory_if_not_exists(const std::string& path) {
    size_t pos = 0;
    std::string sub_path;
    do {
        pos = path.find('/', pos + 1);
        sub_path = path.substr(0, pos);
        if (sub_path.empty()) {
            continue;
        }
        if (mkdir(sub_path.c_str(), 0755) && errno != EEXIST) {
            std::cerr << "Logger Error: Failed to create directory '" 
                      << sub_path << "': " << strerror(errno) << std::endl;
            return;
        }
    } while (pos != std::string::npos);
}

void Logger::init_dlt_with_file_logging() {
    if (!m_offline_logging_enabled) {
        return;
    }
    
    DLT_REGISTER_APP("DEFAULT", "Default DLT Application");
    dlt_set_log_mode(DLT_USER_MODE_BOTH);
    dlt_verbose_mode();
    
    start_dlt_receiver();
    cleanup_old_logs();
}

void Logger::start_dlt_receiver() {
    if (m_dlt_receiver_pid > 0) {
        if (kill(m_dlt_receiver_pid, 0) == 0) {
            return;
        }
    }
    
    m_current_trace_file = m_log_directory + "/" + generate_trace_filename();
    
    m_dlt_receiver_pid = fork();
    
    if (m_dlt_receiver_pid == 0) {
        freopen("/dev/null", "w", stdout);
        freopen("/dev/null", "w", stderr);
        
        execl("/usr/bin/dlt-receive", "dlt-receive",
              "-o", m_current_trace_file.c_str(),
              "localhost", 
              nullptr);
              
        execl("/usr/local/bin/dlt-receive", "dlt-receive",
              "-o", m_current_trace_file.c_str(),
              "localhost", 
              nullptr);
        
        exit(1);
    } else if (m_dlt_receiver_pid > 0) {
        std::cout << "Logger Info: Started dlt-receive (PID: " << m_dlt_receiver_pid 
                  << ") writing to: " << m_current_trace_file << std::endl;
        usleep(100000);
    } else {
        std::cerr << "Logger Warning: Failed to start dlt-receive. "
                  << "Please run manually: dlt-receive -o " << m_current_trace_file 
                  << " localhost" << std::endl;
    }
}

void Logger::stop_dlt_receiver() {
    if (m_dlt_receiver_pid > 0) {
        kill(m_dlt_receiver_pid, SIGTERM);
        
        int status;
        waitpid(m_dlt_receiver_pid, &status, 0);
        
        m_dlt_receiver_pid = 0;
        std::cout << "Logger Info: Stopped dlt-receive" << std::endl;
    }
}

void Logger::rotate_log_file() {
    struct stat file_stat;
    if (stat(m_current_trace_file.c_str(), &file_stat) == 0) {
        if (static_cast<unsigned int>(file_stat.st_size) > m_max_file_size) {
            stop_dlt_receiver();
            m_current_trace_file = m_log_directory + "/" + generate_trace_filename();
            start_dlt_receiver();
            cleanup_old_logs();
        }
    }
}

void Logger::cleanup_old_logs() {
    std::vector<std::string> log_files;
    const std::string& search_path = m_log_directory;
    
    DIR* dir = opendir(search_path.c_str());
    if (dir) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string filename = entry->d_name;
            if (filename.find(LOG_FILE_SUFFIX) != std::string::npos &&
                filename.find(LOG_FILE_PREFIX) == 0) {
                log_files.push_back(filename);
            }
        }
        closedir(dir);
    } else {
        return;
    }
    
    if (log_files.size() > m_max_file_count) {
        std::sort(log_files.begin(), log_files.end());
        
        size_t files_to_delete = log_files.size() - m_max_file_count;
        for (size_t i = 0; i < files_to_delete; ++i) {
            std::string file_to_delete = m_log_directory + "/" + log_files[i];
            if (std::remove(file_to_delete.c_str()) != 0) {
                std::cerr << "Logger Warning: Failed to delete old log file: " 
                          << file_to_delete << std::endl;
            }
        }
    }
}