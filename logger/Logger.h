
#ifndef LOGGER_H
#define LOGGER_H

#include <string>
#include <fstream>
#include <sstream>
#include <mutex> 
#include <sys/stat.h>  // For mkdir
#include <errno.h>     // For errno
#include <cstring>     // For strerror

enum LogLevel { LOG_LEVEL_INFO, LOG_LEVEL_WARNING, LOG_LEVEL_ERROR };

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
    LogLevel m_level;
    std::stringstream m_stream;


	int m_line;
    const char* m_function;
};

class Logger {
public:
    static Logger& get_instance();
    void set_log_file(const std::string& filename);

    void log(LogLevel level, int line, const char* function, const std::string& message);

    /**
     * @brief Đặt thư mục nơi các file log sẽ được lưu trữ.
     * @param path Đường dẫn thư mục (ví dụ: "loglib/log"). Thư mục sẽ được tạo nếu chưa tồn tại.
     */
    void set_log_directory(const std::string& path);

private:
    Logger();
    ~Logger();
    Logger(const Logger&);

    std::string get_level_string(LogLevel level);
    std::string get_current_timestamp();

    std::ofstream m_log_file;
    std::mutex m_mutex;

    void create_directory_if_not_exists(const std::string& path);
    std::string m_log_directory;
    std::string m_current_date_string;

    void update_log_file_if_needed();
    void cleanup_old_logs();
    std::string get_current_date_as_string();
};


#define LOG_INFO LogStream(Logger::get_instance(), LOG_LEVEL_INFO, __LINE__, __FUNCTION__)
#define LOG_WARNING LogStream(Logger::get_instance(), LOG_LEVEL_WARNING, __LINE__, __FUNCTION__)
#define LOG_ERROR LogStream(Logger::get_instance(), LOG_LEVEL_ERROR, __LINE__, __FUNCTION__)

#endif // LOGGER_H
