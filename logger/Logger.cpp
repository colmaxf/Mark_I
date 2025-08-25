
#include "Logger.h"
#include <iostream>
#include <iomanip>
#include <chrono> // C++ handle timstamp
#include <string>
#include <vector>
#include <algorithm>
#include <cstdio> // For std::remove

// For Linux directory listing
#include <dirent.h>
#include <sys/types.h>

LogStream::LogStream(Logger& logger, LogLevel level, int line, const char* function)
	: m_logger_ref(logger), m_level(level), m_line(line), m_function(function) {}

LogStream::~LogStream() {
    m_logger_ref.log(m_level, m_line, m_function, m_stream.str());
}

// Constants for log management
const int MAX_LOG_FILES = 15;
const int FILES_TO_DELETE = 5;
const std::string LOG_FILE_SUFFIX = "_logsys.txt";

std::string Logger::get_current_date_as_string() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y_%m_%d");
    return ss.str();
}

void Logger::create_directory_if_not_exists(const std::string& path) {
    // Hàm này tạo thư mục một cách đệ quy.
    // Ví dụ: nếu path là "a/b/c", nó sẽ tạo "a", rồi "a/b", rồi "a/b/c".
    size_t pos = 0;
    std::string sub_path;
    do {
        pos = path.find('/', pos + 1);
        sub_path = path.substr(0, pos);
        // Bỏ qua sub_path rỗng (có thể xảy ra với đường dẫn bắt đầu bằng '/')
        if (sub_path.empty()) {
            continue;
        }
        if (mkdir(sub_path.c_str(), 0755) && errno != EEXIST) {
            // Không thể dùng logger ở đây vì có thể đang trong quá trình khởi tạo.
            std::cerr << "Logger Error: Failed to create directory '" << sub_path << "': " << strerror(errno) << std::endl;
            return; // Dừng lại khi có lỗi đầu tiên
        }
    } while (pos != std::string::npos);
}

void Logger::set_log_directory(const std::string& path) {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_log_directory = path;
    // Xóa dấu gạch chéo ở cuối nếu có để đảm bảo định dạng nhất quán
    if (!m_log_directory.empty() && m_log_directory.back() == '/') {
        m_log_directory.pop_back();
    }

    if (!m_log_directory.empty()) {
        create_directory_if_not_exists(m_log_directory);
    }
}

void Logger::cleanup_old_logs() {
    std::vector<std::string> log_files;
    //const std::string search_path = "."; // Current directory
    const std::string& search_path = m_log_directory;

    DIR* dir = opendir(search_path.c_str());
    if (dir) {
        struct dirent* entry;
        while ((entry = readdir(dir)) != nullptr) {
            std::string filename = entry->d_name;
            if (filename.length() >= LOG_FILE_SUFFIX.length() &&
                filename.substr(filename.length() - LOG_FILE_SUFFIX.length()) == LOG_FILE_SUFFIX) {
                log_files.push_back(filename);
            }
        }
        closedir(dir);
    }else {
        // Thư mục có thể chưa tồn tại, điều này không sao.
        return;
    }

    if (log_files.size() >= MAX_LOG_FILES) {
        std::sort(log_files.begin(), log_files.end());
        for (int i = 0; i < FILES_TO_DELETE; ++i) {
            //std::remove(log_files[i].c_str());
            std::string file_to_delete = m_log_directory + "/" + log_files[i];
            std::remove(file_to_delete.c_str());
        }
    }
}

void Logger::update_log_file_if_needed() {
    std::string today_date_str = get_current_date_as_string();
    if (m_current_date_string != today_date_str) {
        m_current_date_string = today_date_str;

        if (m_log_file.is_open()) {
            m_log_file.close();
        }

        //std::string filename = m_current_date_string + LOG_FILE_SUFFIX;
        std::string filename = m_log_directory + "/" + m_current_date_string + LOG_FILE_SUFFIX;
        m_log_file.open(filename.c_str(), std::ios::out | std::ios::app);
        if (!m_log_file.is_open()) {
            std::cerr << "Logger Error: Failed to open log file: " << filename << std::endl;
        }

        // Perform cleanup when a new file is created
        cleanup_old_logs();
    }
}

Logger& Logger::get_instance() {
    // Meyers' Singleton: thread-safe in C++11+, no memory leak.
    static Logger instance;
    return instance;
}
// Logger::Logger() {
//     // The log file will be opened on the first call to log()
//     // via update_log_file_if_needed().
//     // Initialize with an empty date string to force an update on first log.
//     m_current_date_string = "";
// }
Logger::Logger() : m_log_directory(".") { // Mặc định là thư mục hiện tại
    m_current_date_string = ""; // Buộc cập nhật trong lần gọi log đầu tiên
}

Logger::~Logger() {
    if (m_log_file.is_open()) { m_log_file.close(); }
}

// This function is no longer recommended as file management is now automatic.
void Logger::set_log_file(const std::string& filename) {
    // std::lock_guard<std::mutex> lock(m_mutex);
    // if (m_log_file.is_open()) { m_log_file.close(); }
    // m_log_file.open(filename.c_str(), std::ios::out | std::ios::app);
}


void Logger::log(LogLevel level, int line, const char* function, const std::string& message) {
    std::lock_guard<std::mutex> lock(m_mutex);

    // Check and update log file if the date has changed
    update_log_file_if_needed();

    if (m_log_file.is_open()) {
        m_log_file << get_current_timestamp()
                   << " " << get_level_string(level)
				   << " [" << function << ":" << line << "]"
				   << " " << message << std::endl;
    }
}

std::string Logger::get_level_string(LogLevel level) {
    switch (level) {
        case LOG_LEVEL_INFO:    return "[INFO]   ";
        case LOG_LEVEL_WARNING: return "[WARNING]";
        case LOG_LEVEL_ERROR:   return "[ERROR]  ";
        default:                return "[UNKNOWN]";
    }
}
std::string Logger::get_current_timestamp() {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
    return ss.str();
}
