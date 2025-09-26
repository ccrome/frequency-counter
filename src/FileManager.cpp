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
  
  // Create temporary file to store filename list
  const char* temp_filename = "/.filelist.tmp";
  File temp_file = SD.open(temp_filename, FILE_WRITE);
  if (!temp_file) {
    Serial.println("Failed to create temporary file for sorting.\r");
    return;
  }
  
  File root = SD.open("/");
  if (!root) {
    temp_file.close();
    SD.remove(temp_filename);
    Serial.println("Failed to open root directory.\r");
    return;
  }
  
  // First pass: write all valid filenames to temp file
  uint32_t file_count = 0;
  File entry = root.openNextFile();
  while (entry) {
    String filename = entry.name();
    
    // Only collect valid log files (skip our temp file)
    if (is_valid_filename(filename) && !filename.equals(".filelist.tmp")) {
      uint32_t size = entry.size();
      temp_file.printf("%s,%lu\n", filename.c_str(), size);
      file_count++;
    }
    entry.close();
    entry = root.openNextFile();
  }
  root.close();
  temp_file.close();
  
  if (file_count == 0) {
    SD.remove(temp_filename);
    Serial.println("No log files found.\r");
    return;
  }
  
  // Read back and sort (using simple insertion sort for file-based sorting)
  // For now, use in-memory sorting with reasonable limit
  const uint32_t SORT_LIMIT = 500;  // Reasonable limit for Teensy RAM
  String filenames[SORT_LIMIT];
  uint32_t file_sizes[SORT_LIMIT];
  uint32_t actual_count = 0;
  
  temp_file = SD.open(temp_filename, FILE_READ);
  if (temp_file) {
    while (temp_file.available() && actual_count < SORT_LIMIT) {
      String line = temp_file.readStringUntil('\n');
      line.trim();
      if (line.length() > 0) {
        int comma_pos = line.indexOf(',');
        if (comma_pos > 0 && comma_pos < (int)(line.length() - 1)) {
          filenames[actual_count] = line.substring(0, comma_pos);
          String size_str = line.substring(comma_pos + 1);
          size_str.trim();  // Remove any whitespace
          file_sizes[actual_count] = strtoul(size_str.c_str(), NULL, 10);  // More robust than toInt()
          actual_count++;
        }
      }
    }
    temp_file.close();
  }
  
  // Clean up temp file
  SD.remove(temp_filename);
  
  // Sort the collected files
  for (uint32_t i = 0; i < actual_count - 1; i++) {
    for (uint32_t j = 0; j < actual_count - i - 1; j++) {
      if (filenames[j] > filenames[j + 1]) {
        // Swap filenames
        String temp_name = filenames[j];
        filenames[j] = filenames[j + 1];
        filenames[j + 1] = temp_name;
        
        // Swap corresponding sizes
        uint32_t temp_size = file_sizes[j];
        file_sizes[j] = file_sizes[j + 1];
        file_sizes[j + 1] = temp_size;
      }
    }
  }
  
  // Display sorted results
  Serial.println("Available log files (sorted):\r");
  Serial.println("ID  Size      Filename\r");
  Serial.println("--  --------  --------\r");
  
  for (uint32_t i = 0; i < actual_count; i++) {
    Serial.printf("%2lu  %8lu  %s", 
                  i, 
                  file_sizes[i],
                  filenames[i].c_str());
    
    // Show if this is the current active log file
    if (filenames[i].equals(current_log_file)) {
      Serial.printf(" (current)\r\n");
    } else {
      Serial.printf("\r\n");
    }
  }
  
  if (file_count > actual_count) {
    Serial.printf("(showing first %lu of %lu files)\r\n", actual_count, file_count);
  }
  
  Serial.printf("\r\nTotal: %lu log files\r\n", file_count);
  Serial.println("Use 'd<ID>' to download a specific file (e.g., 'd0' for first file)\r");
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
  
  // Sort filenames alphabetically (same as list_log_files)
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

uint32_t delete_old_log_files(const String& current_log_file) {
  if (!SD.exists("/")) {
    return 0;
  }
  
  if (current_log_file.length() == 0) {
    return 0;
  }
  
  // Delete all files except current
  File root = SD.open("/");
  if (!root) {
    return 0;
  }
  
  uint32_t deleted_count = 0;
  
  File entry = root.openNextFile();
  while (entry) {
    String filename = entry.name();
    entry.close();
    
    // Only process valid log files
    if (is_valid_filename(filename)) {
      if (!filename.equals(current_log_file)) {
        if (SD.remove(filename.c_str())) {
          deleted_count++;
        }
      }
    }
    
    entry = root.openNextFile();
  }
  root.close();
  
  return deleted_count;
}
