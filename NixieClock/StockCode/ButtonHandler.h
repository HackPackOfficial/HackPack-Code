#pragma once

// Buttons module public API (no deps on your enums or globals)
void setupButtons();   // attach button callbacks
void tickButtons();    // poll buttons each loop
