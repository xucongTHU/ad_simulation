// Copyright 2023 The XUCONG Authors. All Rights Reserved.

#include "common/util/proto_util.h"

#include <dirent.h>
#include <fcntl.h>
#include <glob.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <fstream>
#include <string>

namespace stoic {
namespace common {

using std::istreambuf_iterator;
using std::string;
using std::vector;

bool SetProtoToASCIIFile(const google::protobuf::Message &message,
                         int file_descriptor) {
  using google::protobuf::TextFormat;
  using google::protobuf::io::FileOutputStream;
  using google::protobuf::io::ZeroCopyOutputStream;
  if (file_descriptor < 0) {
    std::cout << "Invalid file descriptor." << std::endl;
    return false;
  }
  ZeroCopyOutputStream *output = new FileOutputStream(file_descriptor);
  bool success = TextFormat::Print(message, output);
  delete output;
  close(file_descriptor);
  return success;
}

bool SetProtoToASCIIFile(const google::protobuf::Message &message,
                         const std::string &file_name) {
  int fd = open(file_name.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU);
  if (fd < 0) {
    std::cout << "Unable to open file " << file_name << " to write." << std::endl;
    return false;
  }
  return SetProtoToASCIIFile(message, fd);
}

bool GetProtoFromASCIIFile(const std::string &file_name,
                           google::protobuf::Message *message) {
  using google::protobuf::TextFormat;
  using google::protobuf::io::FileInputStream;
  using google::protobuf::io::ZeroCopyInputStream;
  int file_descriptor = open(file_name.c_str(), O_RDONLY);
  if (file_descriptor < 0) {
    std::cout << "Failed to open file " << file_name << " in text mode." << std::endl;
    // Failed to open;
    return false;
  }

  ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
  bool success = TextFormat::Parse(input, message);
  if (!success) {
    std::cout << "Failed to parse file " << file_name << " as text proto." << std::endl;
  }
  delete input;
  close(file_descriptor);
  return success;
}

bool SetProtoToBinaryFile(const google::protobuf::Message &message,
                          const std::string &file_name) {
  std::fstream output(file_name,
                      std::ios::out | std::ios::trunc | std::ios::binary);
  return message.SerializeToOstream(&output);
}

bool GetProtoFromBinaryFile(const std::string &file_name,
                            google::protobuf::Message *message) {
  std::fstream input(file_name, std::ios::in | std::ios::binary);
  if (!input.good()) {
    std::cout << "Failed to open file " << file_name << " in binary mode." << std::endl;
    return false;
  }
  if (!message->ParseFromIstream(&input)) {
    std::cout << "Failed to parse file " << file_name << " as binary proto." << std::endl;
    return false;
  }
  return true;
}

bool GetProtoFromFile(const std::string &file_name,
                      google::protobuf::Message *message) {
  // Try the binary parser first if it's much likely a binary proto.
  static const std::string kBinExt = ".bin";
  if (std::equal(kBinExt.rbegin(), kBinExt.rend(), file_name.rbegin())) {
    return GetProtoFromBinaryFile(file_name, message) ||
           GetProtoFromASCIIFile(file_name, message);
  }

  return GetProtoFromASCIIFile(file_name, message) ||
         GetProtoFromBinaryFile(file_name, message);
}

bool GetContent(const std::string &file_name, std::string *content) {
  std::ifstream fin(file_name);
  if (!fin) {
    return false;
  }

  std::stringstream str_stream;
  str_stream << fin.rdbuf();
  *content = str_stream.str();
  return true;
}

std::string GetAbsolutePath(const std::string &prefix,
                            const std::string &relative_path) {
  if (relative_path.empty()) {
    return prefix;
  }
  // If prefix is empty or relative_path is already absolute.
  if (prefix.empty() || relative_path.front() == '/') {
    return relative_path;
  }

  if (prefix.back() == '/') {
    return prefix + relative_path;
  }
  return prefix + "/" + relative_path;
}

bool PathExists(const std::string &path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0;
}

bool DirectoryExists(const std::string &directory_path) {
  struct stat info;
  return stat(directory_path.c_str(), &info) == 0 && (info.st_mode & S_IFDIR);
}

std::vector<std::string> Glob(const std::string &pattern) {
  glob_t globs = {};
  std::vector<std::string> results;
  if (glob(pattern.c_str(), GLOB_TILDE, nullptr, &globs) == 0) {
    for (size_t i = 0; i < globs.gl_pathc; ++i) {
      results.emplace_back(globs.gl_pathv[i]);
    }
  }
  globfree(&globs);
  return results;
}

bool CopyFile(const std::string &from, const std::string &to) {
  std::ifstream src(from, std::ios::binary);
  if (!src) {
    std::cerr << "Source path could not be normally opened: " << from << std::endl;
    std::string command = "cp -r " + from + " " + to;
    std::cout << command;
    const int ret = std::system(command.c_str());
    if (ret == 0) {
      std::cout << "Copy success, command returns " << ret << std::endl;
      return true;
    } else {
      std::cout << "Copy error, command returns " << ret << std::endl;
      return false;
    }
  }

  std::ofstream dst(to, std::ios::binary);
  if (!dst) {
    std::cout << "Target path is not writable: " << to << std::endl;
    return false;
  }

  dst << src.rdbuf();
  return true;
}

bool CopyDir(const std::string &from, const std::string &to) {
  DIR *directory = opendir(from.c_str());
  if (directory == nullptr) {
    std::cout << "Cannot open directory " << from;
    return false;
  }

  bool ret = true;
  if (EnsureDirectory(to)) {
    struct dirent *entry;
    while ((entry = readdir(directory)) != nullptr) {
      // skip directory_path/. and directory_path/..
      if (!strcmp(entry->d_name, ".") || !strcmp(entry->d_name, "..")) {
        continue;
      }
      const std::string sub_path_from = from + "/" + entry->d_name;
      const std::string sub_path_to = to + "/" + entry->d_name;
      if (entry->d_type == DT_DIR) {
        ret &= CopyDir(sub_path_from, sub_path_to);
      } else {
        ret &= CopyFile(sub_path_from, sub_path_to);
      }
    }
  } else {
    std::cout << "Cannot create target directory " << to << std::endl;
    ret = false;
  }
  closedir(directory);
  return ret;
}

bool Copy(const std::string &from, const std::string &to) {
  return DirectoryExists(from) ? CopyDir(from, to) : CopyFile(from, to);
}

bool EnsureDirectory(const std::string &directory_path) {
  std::string path = directory_path;
  for (size_t i = 1; i < directory_path.size(); ++i) {
    if (directory_path[i] == '/') {
      // Whenever a '/' is encountered, create a temporary view from
      // the start of the path to the character right before this.
      path[i] = 0;

      if (mkdir(path.c_str(), S_IRWXU) != 0) {
        if (errno != EEXIST) {
          return false;
        }
      }

      // Revert the temporary view back to the original.
      path[i] = '/';
    }
  }

  // Make the final (full) directory.
  if (mkdir(path.c_str(), S_IRWXU) != 0) {
    if (errno != EEXIST) {
      return false;
    }
  }

  return true;
}

bool RemoveAllFiles(const std::string &directory_path) {
  DIR *directory = opendir(directory_path.c_str());
  if (directory == nullptr) {
    std::cout << "Cannot open directory " << directory_path;
    return false;
  }

  struct dirent *file;
  while ((file = readdir(directory)) != nullptr) {
    // skip directory_path/. and directory_path/..
    if (!strcmp(file->d_name, ".") || !strcmp(file->d_name, "..")) {
      continue;
    }
    // build the path for each file in the folder
    std::string file_path = directory_path + "/" + file->d_name;
    if (unlink(file_path.c_str()) < 0) {
      std::cout << "Fail to remove file " << file_path << ": " << strerror(errno) << std::endl;
      closedir(directory);
      return false;
    }
  }
  closedir(directory);
  return true;
}

std::vector<std::string> ListSubPaths(const std::string &directory_path,
                                      const unsigned char d_type) {
  std::vector<std::string> result;
  DIR *directory = opendir(directory_path.c_str());
  if (directory == nullptr) {
    std::cout << "Cannot open directory " << directory_path << std::endl;
    return result;
  }

  struct dirent *entry;
  while ((entry = readdir(directory)) != nullptr) {
    // Skip "." and "..".
    if (entry->d_type == d_type && strcmp(entry->d_name, ".") != 0 &&
        strcmp(entry->d_name, "..") != 0) {
      result.emplace_back(entry->d_name);
    }
  }
  closedir(directory);
  return result;
}

std::string GetFileName(const std::string &path, const bool remove_extension) {
  std::string::size_type start = path.rfind('/');
  if (start == std::string::npos) {
    start = 0;
  } else {
    // Move to the next char after '/'.
    ++start;
  }

  std::string::size_type end = std::string::npos;
  if (remove_extension) {
    end = path.rfind('.');
    // The last '.' is found before last '/', ignore.
    if (end != std::string::npos && end < start) {
      end = std::string::npos;
    }
  }
  const auto len = (end != std::string::npos) ? end - start : end;
  return path.substr(start, len);
}

std::string GetCurrentPath() {
  char tmp[PATH_MAX];
  return getcwd(tmp, sizeof(tmp)) ? std::string(tmp) : std::string("");
}

bool GetType(const string &filename, FileType *type) {
  struct stat stat_buf;
  if (lstat(filename.c_str(), &stat_buf) != 0) {
    return false;
  }
  if (S_ISDIR(stat_buf.st_mode) != 0) {
    *type = TYPE_DIR;
  } else if (S_ISREG(stat_buf.st_mode) != 0) {
    *type = TYPE_FILE;
  } else {
    std::cerr << "failed to get type: " << filename << std::endl;
    return false;
  }
  return true;
}

bool DeleteFile(const string &filename) {
  if (!PathExists(filename)) {
    return true;
  }
  FileType type;
  if (!GetType(filename, &type)) {
    return false;
  }
  if (type == TYPE_FILE) {
    if (remove(filename.c_str()) != 0) {
      std::cout << "failed to remove file: " << filename << std::endl;
      return false;
    }
    return true;
  }
  DIR *dir = opendir(filename.c_str());
  if (dir == nullptr) {
    std::cerr << "failed to opendir: " << filename << std::endl;
    return false;
  }
  dirent *dir_info = nullptr;
  while ((dir_info = readdir(dir)) != nullptr) {
    if (strcmp(dir_info->d_name, ".") == 0 ||
        strcmp(dir_info->d_name, "..") == 0) {
      continue;
    }
    string temp_file = filename + "/" + string(dir_info->d_name);
    FileType temp_type;
    if (!GetType(temp_file, &temp_type)) {
      std::cerr << "failed to get file type: " << temp_file << std::endl;
      closedir(dir);
      return false;
    }
    if (type == TYPE_DIR) {
      DeleteFile(temp_file);
    }
    remove(temp_file.c_str());
  }
  closedir(dir);
  remove(filename.c_str());
  return true;
}

bool CreateDir(const string &dir) {
  int ret = mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
  if (ret != 0) {
    std::cerr << "failed to create dir. [dir: " << dir
          << "] [err: " << strerror(errno) << "]" << std::endl;
    return false;
  }
  return true;
}

}  // namespace common
}  // namespace stoic
