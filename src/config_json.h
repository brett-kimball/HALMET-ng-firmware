#ifndef HALMET_CONFIG_JSON_H
#define HALMET_CONFIG_JSON_H

#include <ArduinoJson.h>
#include <String.h>

namespace halmet_config {

// Return merged configuration (compile defaults + persisted overrides)
// as a JSON string. If any errors occur, returns an empty string.
String get_merged_config_string();

// Save the provided JSON string as the overrides file (persisted_config.json).
// Returns true on success; on failure returns false and writes an error message
// into error_out (if provided).
bool save_overrides_from_string(const String& s, String* error_out = nullptr);

// Set or clear the calibration sub-object for a specific input (e.g. "A01").
// If json_sub is empty, the calibration override is removed (reverts to defaults).
bool set_input_calibration(const String& input_id, const String& json_sub, String* error_out = nullptr);

// Read the persisted overrides into dest_doc. Returns true on success.
bool load_overrides_into(JsonDocument& dest_doc);

}  // namespace halmet_config

#endif
