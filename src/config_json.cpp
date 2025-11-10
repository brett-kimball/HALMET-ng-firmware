#include "config_json.h"
#include <SPIFFS.h>
#include <FS.h>

using namespace halmet_config;

static const char* k_overrides_path = "/config/persisted_config.json";
static const char* k_defaults_path = "/const/compile_defaults.json";

// Read a file into a String. Returns empty string on error.
static String read_file_to_string(const char* path) {
  if (!SPIFFS.exists(path)) return String("");
  File f = SPIFFS.open(path, FILE_READ);
  if (!f) return String("");
  String s = f.readString();
  f.close();
  return s;
}

String halmet_config::get_merged_config_string() {
  // Read defaults and overrides
  String defaults = read_file_to_string(k_defaults_path);
  String overrides = read_file_to_string(k_overrides_path);

  // Parse
  DynamicJsonDocument ddef(16 * 1024);
  DynamicJsonDocument dover(16 * 1024);
  DynamicJsonDocument dout(32 * 1024);

  DeserializationError err;
  if (defaults.length() > 0) {
    err = deserializeJson(ddef, defaults);
    if (err) {
      // ignore defaults parse errors and continue with overrides only
      ddef.clear();
    }
  }
  if (overrides.length() > 0) {
    err = deserializeJson(dover, overrides);
    if (err) {
      // invalid overrides -> return empty to indicate failure
      return String("");
    }
  }

  // Merge: start with defaults then merge overrides by copying keys
  dout.clear();
  // copy defaults if present
  if (!ddef.isNull()) {
    dout.set(ddef.as<JsonVariant>());
  }
  if (!dover.isNull()) {
    // shallow/recursive merge helper
    std::function<void(JsonVariant, JsonVariant)> merge = [&](JsonVariant dst, JsonVariant src){
      if (src.is<JsonObject>()) {
        for (JsonPair kv : src.as<JsonObject>()) {
          const char* key = kv.key().c_str();
          JsonVariant val = kv.value();
          if (val.is<JsonObject>()) {
            if (!dst.containsKey(key)) dst.createNestedObject(key);
            merge(dst[key], val);
          } else {
            dst[key] = val;
          }
        }
      } else {
        // arrays/values: replace
        dst.set(src);
      }
    };
    merge(dout.as<JsonVariant>(), dover.as<JsonVariant>());
  }

  String out;
  serializeJson(dout, out);
  return out;
}

bool halmet_config::save_overrides_from_string(const String& s, String* error_out) {
  // Validate JSON
  DynamicJsonDocument doc(16 * 1024);
  DeserializationError err = deserializeJson(doc, s);
  if (err) {
    if (error_out) *error_out = String("JSON parse error: ") + err.c_str();
    return false;
  }

  // Ensure directory exists
  if (!SPIFFS.exists("/config")) {
    SPIFFS.mkdir("/config");
  }

  // Atomic write: write to tmp and rename
  const char* tmp = "/config/persisted_config.json.tmp";
  File f = SPIFFS.open(tmp, FILE_WRITE);
  if (!f) {
    if (error_out) *error_out = String("Failed to open temp file for write");
    return false;
  }
  size_t written = serializeJson(doc, f);
  f.close();
  if (written == 0) {
    if (error_out) *error_out = String("Failed to write overrides file");
    SPIFFS.remove(tmp);
    return false;
  }
  // rename
  SPIFFS.remove(k_overrides_path);
  SPIFFS.rename(tmp, k_overrides_path);
  return true;
}

bool halmet_config::set_input_calibration(const String& input_id, const String& json_sub, String* error_out) {
  // Load overrides into doc
  DynamicJsonDocument doc(16 * 1024);
  if (!load_overrides_into(doc)) {
    // start fresh
    doc.clear();
  }

  JsonObject root = doc.to<JsonObject>();
  JsonObject inputs = root.containsKey("inputs") ? root["inputs"].as<JsonObject>() : root.createNestedObject("inputs");

  if (json_sub.length() == 0) {
    // remove calibration override for input_id
    if (inputs.containsKey(input_id.c_str())) {
      JsonObject in = inputs[input_id.c_str()].as<JsonObject>();
      if (in.containsKey("calibration")) in.remove("calibration");
    }
  } else {
    // parse sub-object
    DynamicJsonDocument sub(8 * 1024);
    DeserializationError err = deserializeJson(sub, json_sub);
    if (err) {
      if (error_out) *error_out = String("Calibration JSON parse error: ") + err.c_str();
      return false;
    }
    JsonVariant vsub = sub.as<JsonVariant>();
    JsonObject in = inputs.containsKey(input_id.c_str()) ? inputs[input_id.c_str()].as<JsonObject>() : inputs.createNestedObject(input_id.c_str());
    in.remove("calibration");
    in.createNestedObject("calibration").set(vsub);
  }

  // Save doc back
  String out;
  serializeJson(doc, out);
  return save_overrides_from_string(out, error_out);
}

bool halmet_config::load_overrides_into(JsonDocument& dest_doc) {
  String overrides = "";
  if (SPIFFS.exists(k_overrides_path)) {
    overrides = read_file_to_string(k_overrides_path);
  }
  if (overrides.length() == 0) return false;
  DeserializationError err = deserializeJson(dest_doc, overrides);
  if (err) return false;
  return true;
}
