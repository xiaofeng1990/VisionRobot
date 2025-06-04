#ifndef MASTER_COMMON_XT_UTILS_H_
#define MASTER_COMMON_XT_UTILS_H_
#include <sys/timeb.h>
#include <time.h>

#include <algorithm>
#include <string>
#include <vector>

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

#endif // MASTER_COMMON_XT_UTILS_H_