#pragma once

#include "sensesp/ui/config_item.h"
#include "sensesp/ui/ui_controls.h"
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace halmet {

using ConfigBuilder = std::function<std::shared_ptr<sensesp::ConfigItemBase>()>;
using CreatedItemsCallback = std::function<void(const std::vector<std::shared_ptr<sensesp::ConfigItemBase>>&)>;

// Register an analog context name (e.g. "A01") so the global selector can
// include it in the options list. Call this from analog input setup before
// calling ensure_config_manager().
void register_context_name(const String& context_name);

// Ensure the global configuration selector exists. Safe to call multiple times.
void ensure_config_manager();

// Backwards-compatible name used by older code. Calls ensure_config_manager.
void ensure_configuration_manager();

// Register a set of builders for a named configuration context (e.g. "A01").
// When the selector equals that context, the manager will invoke the builders
// (creating ConfigItem objects) and will call the optional created_items_cb
// with the created shared_ptrs. If the context is already selected when
// registering, builders will be invoked immediately.
void register_context_builders(const String& context_name,
                               const std::vector<ConfigBuilder>& builders,
                               CreatedItemsCallback created_items_cb = nullptr);

// Query whether a given context is currently selected in the global selector.
bool is_context_selected(const String& context_name);

}  // namespace halmet
