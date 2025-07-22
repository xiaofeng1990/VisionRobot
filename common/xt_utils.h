#ifndef MASTER_COMMON_XT_UTILS_H_
#define MASTER_COMMON_XT_UTILS_H_
#include <sys/timeb.h>
#include <time.h>

#include <algorithm>
#include <string>
#include <vector>

uint64_t htonll(uint64_t value);
uint64_t ntohll(uint64_t value);
ssize_t send_all(int sockfd, const void *data, size_t length);
ssize_t recv_all(int sockfd, void *buffer, size_t length);
std::string to_lower(const std::string &str);
int xtmkdir(const std::string path);
int download_file(const std::string &url, std::string &save_path);
time_t generate_timestamp_ms();
// std::string& trim(std::string& s);
std::vector<std::string> load_labels(std::string lable_file);
bool is_dir(std::string path);
std::string get_directory(std::string path);
std::string get_file_name_from_path(std::string path);
void generate_ts(std::string &buff);
bool exists(const std::string filepath);

void remove_whitespace(std::string &s);
void trim(std::string &str, char c);
bool is_all_digits(const std::string &str);
int find_first_non_digit_position(const std::string &str);
int find_first_digit_position(const std::string &str);
std::vector<std::string> split(const std::string &s, char delimiter);

std::vector<std::string> list_files_in_directory(const std::string &directory_path);
bool remove_file(const std::string &file_path);
#endif // MASTER_COMMON_XT_UTILS_H_