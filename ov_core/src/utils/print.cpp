/*
 * Sqrt-VINS: A Sqrt-filter-based Visual-Inertial Navigation System
 * Copyright (C) 2025-2026 Yuxiang Peng
 * Copyright (C) 2025-2026 Chuchu Chen
 * Copyright (C) 2025-2026 Kejian Wu
 * Copyright (C) 2018-2026 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this program. If not, see
 * <https://www.gnu.org/licenses/>.
 */





#include "print.h"

#if defined(__ANDROID__) || defined(ANDROID)
#include <android/log.h>
#define ANDROID_LOG_TAG "SqrtVINS"
#endif

using namespace ov_core;

// Need to define the static variable for everything to work
Printer::PrintLevel Printer::current_print_level = PrintLevel::INFO;

void Printer::setPrintLevel(const std::string &level) {
  if (level == "ALL")
    setPrintLevel(PrintLevel::ALL);
  else if (level == "DEBUG")
    setPrintLevel(PrintLevel::DEBUG);
  else if (level == "INFO")
    setPrintLevel(PrintLevel::INFO);
  else if (level == "WARNING")
    setPrintLevel(PrintLevel::WARNING);
  else if (level == "ERROR")
    setPrintLevel(PrintLevel::ERROR_LVL);
  else if (level == "SILENT")
    setPrintLevel(PrintLevel::SILENT);
  else {
#if defined(__ANDROID__) || defined(ANDROID)
    __android_log_print(ANDROID_LOG_ERROR, ANDROID_LOG_TAG,
                        "Invalid print level requested: %s", level.c_str());
    __android_log_print(ANDROID_LOG_ERROR, ANDROID_LOG_TAG,
                        "Valid levels are: ALL, DEBUG, INFO, WARNING, ERROR, SILENT");
#else
    std::cout << "Invalid print level requested: " << level << std::endl;
    std::cout << "Valid levels are: ALL, DEBUG, INFO, WARNING, ERROR, SILENT"
              << std::endl;
#endif
    std::exit(EXIT_FAILURE);
  }
}

void Printer::setPrintLevel(PrintLevel level) {
  Printer::current_print_level = level;
  const char *level_str = "UNKNOWN";
  switch (current_print_level) {
  case PrintLevel::ALL: level_str = "ALL"; break;
  case PrintLevel::DEBUG: level_str = "DEBUG"; break;
  case PrintLevel::INFO: level_str = "INFO"; break;
  case PrintLevel::WARNING: level_str = "WARNING"; break;
  case PrintLevel::ERROR_LVL: level_str = "ERROR"; break;
  case PrintLevel::SILENT: level_str = "SILENT"; break;
  default:
    std::cout << std::endl;
    std::cout << "Invalid print level requested: " << level << std::endl;
    std::cout << "Valid levels are: ALL, DEBUG, INFO, WARNING, ERROR, SILENT"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
#if defined(__ANDROID__) || defined(ANDROID)
  __android_log_print(ANDROID_LOG_INFO, ANDROID_LOG_TAG,
                      "Setting printing level to: %s", level_str);
#else
  std::cout << "Setting printing level to: " << level_str << std::endl;
#endif
}

void Printer::debugPrint(PrintLevel level, const char location[],
                         const char line[], const char *format, ...) {
  // Only print for the current debug level
  if (static_cast<int>(level) <
      static_cast<int>(Printer::current_print_level)) {
    return;
  }

#if defined(__ANDROID__) || defined(ANDROID)
  // Map PrintLevel to Android log priority
  int android_level;
  switch (level) {
  case PrintLevel::ALL:       android_level = ANDROID_LOG_VERBOSE; break;
  case PrintLevel::DEBUG:     android_level = ANDROID_LOG_DEBUG; break;
  case PrintLevel::INFO:      android_level = ANDROID_LOG_INFO; break;
  case PrintLevel::WARNING:   android_level = ANDROID_LOG_WARN; break;
  case PrintLevel::ERROR_LVL: android_level = ANDROID_LOG_ERROR; break;
  default:                    android_level = ANDROID_LOG_INFO; break;
  }

  // Build location prefix for debug mode
  std::string prefix;
  if (static_cast<int>(Printer::current_print_level) <=
      static_cast<int>(Printer::PrintLevel::DEBUG)) {
    std::string path(location);
    std::string base_filename = path.substr(path.find_last_of("/\\") + 1);
    if (base_filename.size() > MAX_FILE_PATH_LEGTH)
      base_filename = base_filename.substr(base_filename.size() - MAX_FILE_PATH_LEGTH);
    prefix = base_filename + ":" + line + " ";
  }

  // Format the message and send to logcat
  va_list args;
  va_start(args, format);
  char buf[2048];
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  __android_log_print(android_level, ANDROID_LOG_TAG, "%s%s", prefix.c_str(), buf);
#else
  // Print the location info first for our debug output
  if (static_cast<int>(Printer::current_print_level) <=
      static_cast<int>(Printer::PrintLevel::DEBUG)) {
    std::string path(location);
    std::string base_filename = path.substr(path.find_last_of("/\\") + 1);
    if (base_filename.size() > MAX_FILE_PATH_LEGTH) {
      printf("%s", base_filename
                       .substr(base_filename.size() - MAX_FILE_PATH_LEGTH,
                               base_filename.size())
                       .c_str());
    } else {
      printf("%s", base_filename.c_str());
    }
    printf(":%s ", line);
  }

  // Print the rest of the args
  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);
#endif
}
