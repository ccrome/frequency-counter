#include "FileManager.h"

bool is_valid_filename(const String& filename) {
  return (filename.endsWith(".jsonl") ||
          filename.endsWith(".JSONL") ||
          filename.endsWith(".nmea")  ||
          filename.endsWith(".NMEA")
  );
}


String get_filename_by_id(uint32_t file_id) {
  if (!SD.exists("/")) {
    return "";
  }
  
  File root = SD.open("/");
  if (!root) {
    return "";
  }
  
  // Collect all valid filenames
  String filenames[MAX_FILES];
  uint32_t file_count = 0;
  File entry = root.openNextFile();
  while (entry && file_count < MAX_FILES) {
    String filename = entry.name();
    
    if (is_valid_filename(filename)) {
      filenames[file_count] = filename;
      file_count++;
    }
    entry.close();
    entry = root.openNextFile();
  }
  root.close();
  
  if (file_count == 0 || file_id >= file_count) {
    return "";
  }
  
  // Sort filenames alphabetically
  for (uint32_t i = 0; i < file_count - 1; i++) {
    for (uint32_t j = 0; j < file_count - i - 1; j++) {
      if (filenames[j] > filenames[j + 1]) {
        String temp = filenames[j];
        filenames[j] = filenames[j + 1];
        filenames[j + 1] = temp;
      }
    }
  }
  
  return filenames[file_id];
}

uint32_t get_file_count() {
  if (!SD.exists("/")) {
    return 0;
  }
  
  File root = SD.open("/");
  if (!root) {
    return 0;
  }
  
  uint32_t file_count = 0;
  File entry = root.openNextFile();
  while (entry && file_count < MAX_FILES) {
    String filename = entry.name();
    
    if (is_valid_filename(filename)) {
      file_count++;
    }
    entry.close();
    entry = root.openNextFile();
  }
  root.close();
  
  return file_count;
}

bool file_exists(const String& filename) {
  return SD.exists(filename.c_str());
}

FileInfo get_file_info(uint32_t file_id, const String& current_log_file) {
  FileInfo info;
  info.filename = "";
  info.size = 0;
  info.is_current = false;
  
  String filename = get_filename_by_id(file_id);
  
  if (filename.length() == 0) {
    return info;
  }
  
  info.filename = filename;
  
  // Get file size and check if it's current
  File file = SD.open(filename.c_str(), FILE_READ);
  if (file) {
    info.size = file.size();
    file.close();
  }
  
  info.is_current = filename.equals(current_log_file);
  
  return info;
}

