#ifndef OTA_HANDLER_H
#define OTA_HANDLER_H

#include <Arduino.h>
#include <Update.h>
#include <esp_ota_ops.h>
#include <esp_partition.h>
#include <SPIFFS.h>
#include "ota_upload_page.h"

class OTAHandler {
private:
  bool isUpdating = false;
  bool updateSucceeded = false;
  size_t updateSize = 0;
  unsigned long lastProgressTime = 0;
  bool spiffsUpdating = false;
  bool spiffsSucceeded = false;
  File spiffsFile;
  
public:
  OTAHandler() {}
  
  // Initialize OTA handler and register web endpoints
  void begin(AsyncWebServer& server) {
    // OTA firmware upload page
    server.on("/update-page", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/html", OTA_UPLOAD_HTML);
    });
    
    // OTA firmware upload endpoint
    server.on("/update", HTTP_POST, 
      [this](AsyncWebServerRequest *request) {
        // This is called after upload completes
        bool success = updateSucceeded;
        updateSucceeded = false;  // Reset for next attempt
        AsyncWebServerResponse *response = request->beginResponse(success ? 200 : 500, "text/plain",
          success ? "OK" : "FAILED");
        response->addHeader("Connection", "close");
        response->addHeader("Access-Control-Allow-Origin", "*");
        request->send(response);
        
        if (success) {
          Serial.println("\n✓ OTA Update completed successfully!");
          Serial.println("  Rebooting in 2 seconds...");
          delay(2000);
          ESP.restart();
        } else {
          Serial.println("\n✗ OTA Update failed (no data received or write error)!");
          isUpdating = false;
        }
      },
      [this](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
        // This is called for each data chunk
        if (!index) {
          // First chunk - initialize update
          Serial.printf("\n=== OTA Update Starting ===\n");
          Serial.printf("Filename: %s\n", filename.c_str());
          Serial.printf("File size: %d bytes\n", request->contentLength());
          
          // Check free space BEFORE attempting update
          const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
          if (!next) {
            Serial.println("✗ No OTA partition available!");
            isUpdating = false;
            return;
          }
          
          Serial.printf("Target partition: %s at 0x%X, size: %d bytes\n", 
                       next->label, next->address, next->size);
          
          if (request->contentLength() > next->size) {
            Serial.printf("✗ File too large: %d > %d bytes\n", request->contentLength(), next->size);
            isUpdating = false;
            return;
          }
          
          // Begin OTA update
          if (!Update.begin(request->contentLength(), U_FLASH)) {
            Serial.println("✗ OTA update begin failed!");
            Serial.print("  Error: ");
            Update.printError(Serial);
            isUpdating = false;
            return;
          }
          
          isUpdating = true;
          updateSize = 0;
          lastProgressTime = millis();
          Serial.println("✓ OTA partition initialized and ready to receive data");
        }
        
        if (isUpdating) {
          // Write data chunk
          if (Update.write(data, len) == len) {
            updateSize += len;
            
            // Print progress every 1 second
            if (millis() - lastProgressTime > 1000) {
              float percent = (updateSize * 100.0) / request->contentLength();
              Serial.printf("  Progress: %d / %d bytes (%.1f%%)\n", updateSize, request->contentLength(), percent);
              lastProgressTime = millis();
            }
          } else {
            Serial.printf("✗ Write failed at offset %d\n", updateSize);
            Serial.print("  Error: ");
            Update.printError(Serial);
            isUpdating = false;
            return;
          }
        }
        
        if (final) {
          if (!isUpdating) return;  // Upload was aborted earlier, nothing to finalize
          if (!Update.end(true)) {
            Serial.println("✗ Error finalizing OTA update");
            Serial.print("  Error: ");
            Update.printError(Serial);
            isUpdating = false;
            return;
          }
          
          Serial.printf("✓ Update finalized: %d bytes total\n", updateSize);
          
          // After Update.end(true), check which partition is now active
          const esp_partition_t *running = esp_ota_get_running_partition();
          const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
          
          Serial.printf("  Current running partition: %s at 0x%X\n", 
                       running ? running->label : "NONE", running ? running->address : 0);
          Serial.printf("  Next partition designated: %s at 0x%X\n", 
                       next ? next->label : "NONE", next ? next->address : 0);
          
          Serial.println("✓ OTA update complete - device will reboot with new firmware");
          updateSucceeded = true;
          isUpdating = false;
        }
      }
    );
    
    // SPIFFS file upload endpoint — updates dashboard.html or any SPIFFS file, no reboot needed
    server.on("/update-spiffs", HTTP_POST,
      [this](AsyncWebServerRequest *request) {
        bool success = spiffsSucceeded;
        spiffsSucceeded = false;
        AsyncWebServerResponse *response = request->beginResponse(success ? 200 : 500, "text/plain",
          success ? "OK" : "FAILED");
        response->addHeader("Connection", "close");
        response->addHeader("Access-Control-Allow-Origin", "*");
        request->send(response);
        if (success) {
          Serial.println("\n\u2713 SPIFFS file updated — live immediately, no reboot needed");
        } else {
          Serial.println("\n\u2717 SPIFFS file upload failed!");
        }
      },
      [this](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final) {
        if (!index) {
          String path = "/" + filename;
          Serial.printf("\n=== SPIFFS Upload Starting ===\n");
          Serial.printf("File: %s (%u bytes)\n", path.c_str(), request->contentLength());
          spiffsFile = SPIFFS.open(path, "w");
          if (!spiffsFile) {
            Serial.printf("\u2717 Failed to open %s for writing\n", path.c_str());
            spiffsUpdating = false;
            return;
          }
          spiffsUpdating = true;
        }
        if (spiffsUpdating && len) {
          if (spiffsFile.write(data, len) != len) {
            Serial.println("\u2717 SPIFFS write error");
            spiffsFile.close();
            spiffsUpdating = false;
            return;
          }
        }
        if (final) {
          if (spiffsFile) spiffsFile.close();
          if (spiffsUpdating) {
            Serial.printf("\u2713 SPIFFS file written: %u bytes total\n", index + len);
            spiffsSucceeded = true;
            spiffsUpdating = false;
          }
        }
      }
    );

    // OTA info endpoint to check current version and available space
    server.on("/ota/info", HTTP_GET, [this](AsyncWebServerRequest *request) {
      DynamicJsonDocument doc(512);
      
      // Get current partition info
      const esp_partition_t *running = esp_ota_get_running_partition();
      const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
      
      doc["current_partition"] = String(running->label);
      doc["current_partition_address"] = String(running->address, HEX);
      doc["next_partition"] = String(next ? next->label : "none");
      doc["next_partition_address"] = next ? String(next->address, HEX) : "none";
      
      // Free space for update
      uint32_t free_space = next ? next->size : 0;
      doc["available_space"] = free_space;
      doc["max_allowed_size"] = free_space;
      
      // Flash info
      doc["flash_size"] = ESP.getFlashChipSize();
      doc["flash_speed"] = ESP.getFlashChipSpeed();
      
      // Version info (these can be customized)
      doc["version"] = "1.0.0";  // Update this manually or via build flags
      doc["build_date"] = __DATE__ " " __TIME__;
      
      String response;
      serializeJson(doc, response);
      request->send(200, "application/json", response);
    });
    
    Serial.println("✓ OTA web endpoints registered:");
    Serial.println("  GET  /update-page - OTA firmware update interface");
    Serial.println("  POST /update - Upload firmware");    Serial.println("  POST /update-spiffs - Upload SPIFFS file (dashboard etc.)");    Serial.println("  GET  /ota/info - Get OTA partition info");
  }
  
  bool isOTAInProgress() const {
    return isUpdating;
  }
};

#endif // OTA_HANDLER_H
