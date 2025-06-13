#include "xt_utils.h"
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/timeb.h>
#include <time.h>
#include <unistd.h>

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <sys/time.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

// 辅助函数：保证发送所有数据
ssize_t send_all(int sockfd, const void *data, size_t length)
{
    size_t total = 0;
    const char *ptr = static_cast<const char *>(data);
    while (total < length)
    {
        int sent = send(sockfd, ptr + total, static_cast<int>(length - total), 0);
        if (sent == -1 || sent <= 0)
        {
            return -1;
        }
        total += sent;
    }
    return static_cast<ssize_t>(total);
}

// 辅助函数：保证接收指定长度的数据
ssize_t recv_all(int sockfd, void *buffer, size_t length)
{
    size_t total = 0;
    char *ptr = static_cast<char *>(buffer);
    while (total < length)
    {
        int recvd = recv(sockfd, ptr + total, static_cast<int>(length - total), 0);
        if (recvd == -1 || recvd <= 0)
        {
            return -1;
        }
        total += recvd;
    }
    return static_cast<ssize_t>(total);
}

uint64_t htonll(uint64_t value)
{
// Linux 下如果系统为小端，也需要转换
#if __BYTE_ORDER == __LITTLE_ENDIAN
    uint32_t high = htonl(static_cast<uint32_t>(value >> 32));
    uint32_t low = htonl(static_cast<uint32_t>(value & 0xFFFFFFFFULL));
    return (static_cast<uint64_t>(low) << 32) | high;
#else
    return value;
#endif
}

uint64_t ntohll(uint64_t value)
{

#if __BYTE_ORDER == __LITTLE_ENDIAN
    uint32_t high = ntohl(static_cast<uint32_t>(value >> 32));
    uint32_t low = ntohl(static_cast<uint32_t>(value & 0xFFFFFFFFULL));
    return (static_cast<uint64_t>(low) << 32) | high;
#else
    return value;
#endif
}

std::string to_lower(const std::string &str)
{
    auto tmp_str = str;
    std::transform(tmp_str.begin(), tmp_str.end(), tmp_str.begin(), [](char &c)
                   { return std::tolower(c); });

    return tmp_str;
}

int xtmkdir(const std::string path)
{
    if (access(path.c_str(), F_OK) == 0)
    {
        // XT_LOGT(INFO, "mkdir", "The path of saveing path has existed(%s) ", path.c_str());
        return 0;
    }
    else
    {
        if (mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == 0)
        {
            // XT_LOGT(INFO, "mkdir", "The path of saveing path ok(%s) ", path.c_str());
            return 0;
        }
    }
    return -1;
}

time_t generate_timestamp_ms()
{
    // auto timestampms = std::chrono::duration_cast<std::chrono::milliseconds>(
    //     std::chrono::system_clock::now().time_since_epoch());
    // return timestampms.count();

    struct timeb t;
    ftime(&t);
    return 1000 * t.time + t.millitm;
}

// std::string& trim(std::string& s)
// {
//     if (s.empty())
//     {
//         return s;
//     }
//     s.erase(0, s.find_first_not_of(" "));
//     s.erase(s.find_last_not_of(" ") + 1);
//     return s;
// }

std::vector<std::string> load_labels(std::string lable_file)
{
    std::ifstream infile(lable_file);
    std::vector<std::string> labels;
    if (infile.is_open())
    {
        std::string line;
        while (getline(infile, line))
        {
            // trim(line, '/r');
            remove_whitespace(line);
            labels.push_back(line);
        }
    }
    else
    {
        // XT_LOGT(ERROR, "utils", "load lable file fail (%s)", lable_file.c_str());
    }
    return labels;
}

bool is_dir(std::string path)
{
    struct stat buf;
    if (lstat(path.c_str(), &buf) < 0)
    {
        return false;
    }
    int ret = __S_IFDIR & buf.st_mode;
    if (ret)
    {
        return true;
    }
    return false;
}

// string path = "./my_directory/my_file.txt";
// return: "./my_directory"
std::string get_directory(std::string path)
{
    if (is_dir(path))
    {
        return path;
    }

    std::string directory;
    const size_t last_slash_idx = path.rfind('/');
    if (std::string::npos != last_slash_idx)
    {
        directory = path.substr(0, last_slash_idx);
    }
    return directory;
}

// string path = "./my_directory/my_file.txt";
// ret: "my_file.txt"
std::string get_file_name_from_path(std::string path)
{
    if (is_dir(path))
    {
        return "isdir";
    }

    std::string filename;
    const size_t last_slash_idx = path.rfind('/');
    if (std::string::npos != last_slash_idx)
    {
        filename = path.substr(last_slash_idx + 1);
    }
    else
    {
        // only filename
        filename = path;
    }
    return filename;
}

// void generate_ts_(char* buf, int buf_size)
// {
//     time_t tloc;
//     struct tm tm_log;
//     struct timespec ts;
//     char strmsec[6];  //.nnnZ\0

//     clock_gettime(CLOCK_REALTIME, &ts);
//     memcpy(&tloc, (void*)(&ts.tv_sec), sizeof(time_t));
//     gmtime_r(&tloc, &tm_log);
//     strftime(buf, buf_size, "%Y-%m-%dT%H:%M:%S", &tm_log);
//     int ms = ts.tv_nsec / 1000000;
//     g_snprintf(strmsec, sizeof(strmsec), ".%.3dZ", ms);
//     strncat(buf, strmsec, buf_size);
// }

void generate_ts(std::string &buff)
{
    struct timeval time;
    gettimeofday(&time, NULL);
    time_t seconds = static_cast<time_t>(time.tv_sec);
    int remainder = static_cast<int>(time.tv_usec);
    char buffer[64] = {0};
    struct tm local_time;
    localtime_r(&seconds, &local_time);
    strftime(buffer, sizeof(buffer), "%Y-%m-%d-%H:%M:%S", &local_time);
    sprintf(buffer, "%s.%d", buffer, remainder);
    buff = buffer;
}

bool exists(const std::string filepath) { return access(filepath.c_str(), R_OK) == 0; }
void remove_whitespace(std::string &s)
{
    s.erase(std::remove_if(s.begin(), s.end(), [](unsigned char c)
                           { return std::isspace(c); }),
            s.end());
}
void trim(std::string &str, char c)
{
    int write_pos = 0;
    for (int read_pos = 0; read_pos < str.size(); ++read_pos)
    {
        if (str[read_pos] != c)
        {
            str[write_pos++] = str[read_pos];
        }
    }
    str.resize(write_pos); // 调整字符串大小
}

bool is_all_digits(const std::string &str)
{
    if (str.empty())
    {
        // 空字符串返回false
        return false;
    }
    for (char c : str)
    {
        if (!isdigit(static_cast<unsigned char>(c)))
        {
            return false; // 发现非数字字符
        }
    }
    return true;
}

int find_first_non_digit_position(const std::string &str)
{
    for (int i = 0; i < str.size(); ++i)
    {
        if (!isdigit(static_cast<unsigned char>(str[i])))
        {
            return i; // 返回第一个非数字索引
        }
    }
    return -1; // 全为数字时返回 npos
}

int find_first_digit_position(const std::string &str)
{
    for (int i = 0; i < str.size(); ++i)
    {
        if (isdigit(static_cast<unsigned char>(str[i])))
        {
            return i; // 返回第一个数字索引
        }
    }
    return -1; // 全为数字时返回 npos
}

std::vector<std::string> split(const std::string &s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    for (char c : s)
    {
        if (c == delimiter)
        {
            if (!token.empty())
            { // 仅当token非空时保存
                tokens.push_back(token);
                token.clear();
            }
        }
        else
        {
            token += c;
        }
    }
    if (!token.empty())
    {
        tokens.push_back(token);
    }
    return tokens;
}
namespace fs = boost::filesystem;

std::vector<std::string> list_files_in_directory(const std::string &directory_path)
{
    std::vector<std::string> files;
    try
    {
        for (const auto &entry : fs::directory_iterator(directory_path))
        {
            if (fs::is_regular_file(entry.status()))
            {
                files.push_back(entry.path().string());
            }
        }
    }
    catch (const fs::filesystem_error &e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
    }
    return files;
}
