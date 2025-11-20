#ifndef CL_VERSION_CONTROL_H
#define CL_VERSION_CONTROL_H

#include <avr/pgmspace.h>
#include <Arduino.h>

// Version control struct stored in PROGMEM
struct CodeVersionContainer {
    const char SKU[8];              // Fixed-size array for SKU
    const char projectVersion[16]; // Fixed-size array for project version
    const float codeVersion;       // Code version
};

// Initialize struct with values and store in PROGMEM
const CodeVersionContainer codeVersion PROGMEM = {
    "PLANT",       // SKU
    "V1",          // Project version
    1.1             // Code version
};

// Function to read data from PROGMEM
void readCodeVersion() {
    char SKU[8];
    char projectVersion[16];
    float version;

    memcpy_P(SKU, codeVersion.SKU, sizeof(SKU));
    memcpy_P(projectVersion, codeVersion.projectVersion, sizeof(projectVersion));
    version = pgm_read_float(&codeVersion.codeVersion);

    Serial.print("SKU: "); Serial.println(SKU);
    Serial.print("Project Version: "); Serial.println(projectVersion);
    Serial.print("Code Version: "); Serial.println(version);
}

#endif // CL_VERSION_CONTROL_H
