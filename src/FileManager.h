#ifndef FILE_MANAGER_H
#define FILE_MANAGER_H

#include <Arduino.h>
#include <SD.h>

// File management constants
#define MAX_FILES 2000
#define MAX_FILENAME_LENGTH 64

// File information structure
struct FileInfo {
  String filename;
  uint32_t size;
  bool is_current;
};

// Function declarations
bool is_valid_filename(const String& filename);
String get_filename_by_id(uint32_t file_id);
uint32_t get_file_count();
bool file_exists(const String& filename);
FileInfo get_file_info(uint32_t file_id, const String& current_log_file);

#endif // FILE_MANAGER_H
