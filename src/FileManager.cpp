#include "FileManager.h"

bool is_valid_filename(const String& filename) {
  return (filename.endsWith(".jsonl") ||
          filename.endsWith(".JSONL") ||
          filename.endsWith(".nmea")  ||
          filename.endsWith(".NMEA")
  );
}

void list_log_files(const String& current_log_file) {
  if (!SD.exists("/")) {
    Serial.println("SD card not available.\r");
    return;
  }
  
  File root = SD.open("/");
  if (!root) {
    Serial.println("Failed to open root directory.\r");
    return;
  }
  
  Serial.println("Available log files:\r");
  Serial.println("ID  Filename          Size (bytes)  Modified\r");
  Serial.println("--  ----------------  ------------  --------\r");
  
  uint32_t file_count = 0;
  File entry = root.openNextFile();
  while (entry && file_count < MAX_FILES) {
    String filename = entry.name();
    
    // Only show .jsonl files (our log files)
    if (is_valid_filename(filename)) {
      Serial.printf("%2lu  %-16s  %12lu  ", 
                    file_count, 
                    filename.c_str(), 
                    entry.size());
      
      // Show if this is the current active log file
      if (filename.equals(current_log_file)) {
        Serial.printf("(current)\r\n");
      } else {
        Serial.printf("\r\n");
      }
      file_count++;
    }
    entry.close();
    entry = root.openNextFile();
  }
  root.close();
  
  if (file_count == 0) {
    Serial.println("No log files found.\r");
  } else {
    Serial.printf("\r\nTotal: %lu log files\r\n", file_count);
    Serial.println("Use 'd<ID>' to download a specific file (e.g., 'd0' for first file)\r");
  }
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
  
  if (file_id >= file_count) {
    return "";
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
