#include "config_manager.h"
#include "sensesp/system/valueproducer.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/ui_controls.h"

namespace halmet {

static bool cfg_inited = false;
static std::shared_ptr<sensesp::SelectConfig> cfg_selector = nullptr;
static std::map<String, std::vector<ConfigBuilder>> pending_builders;
static std::map<String, std::vector<CreatedItemsCallback>> pending_callbacks;
static std::vector<String> extra_context_names;

void register_context_name(const String& context_name) {
  // Avoid duplicates
  for (auto& s : extra_context_names) if (s == context_name) return;
  extra_context_names.push_back(context_name);
}

static std::vector<String> build_selector_options() {
  // Provide a fixed, user-friendly ordering of configuration contexts.
  std::vector<String> opts;
  opts.push_back("Source Configuration");
  opts.push_back("Analog Sources");
  opts.push_back("Digital Sources");
  opts.push_back("SignalK");
  opts.push_back("NMEA2000");
  opts.push_back("OLED Display");
  opts.push_back("Global Configs");
  return opts;
}

void ensure_config_manager() {
  if (cfg_inited) return;
  cfg_inited = true;

  auto selector_default = String("Global Configs");
  auto selector_path = String("/Inputs/Configure");
  auto selector_opts = build_selector_options();
  cfg_selector = std::make_shared<sensesp::SelectConfig>(selector_default, String("Configuration Manager"), selector_path, selector_opts, sensesp::SelectType::kSelect);
  auto* selector_title = new String("Configuration Manager");
  auto* selector_desc = new String("Choose a configuration context. Save and reboot to make the selected context's advanced options visible. Select 'Global Configs' to edit system-wide toggles.");
  // Make the configuration manager selector the top-most item on the page.
  // Other config items should use higher sort_order values so they appear
  // below the selector.
  #if !DISABLE_LEGACY_CONFIG_ITEMS
  ConfigItem(cfg_selector)->set_title(*selector_title)->set_description(*selector_desc)->set_sort_order(0);
  #endif

  // One-shot delay to invoke builders for the selected context
  sensesp::event_loop()->onDelay(1000, []() {
    if (!cfg_selector) return;
    String sel = cfg_selector->get_value();
    // Build items for the selected context itself
    std::vector<std::shared_ptr<sensesp::ConfigItemBase>> created_items;
    if (pending_builders.count(sel)) {
      auto& vec = pending_builders[sel];
      for (auto& b : vec) {
        auto it = b();
        if (it) created_items.push_back(it);
      }
      // Invoke any registered callbacks for this context
      if (pending_callbacks.count(sel)) {
        for (auto& cb : pending_callbacks[sel]) {
          cb(created_items);
        }
      }
    }

    // If a grouped context is selected, also invoke builders for child
    // contexts (for example, selecting "Analog Sources" should also
    // instantiate builders registered under "A01","A02",...)
    if (sel == "Analog Sources" || sel == "Digital Sources") {
      // Determine prefix to match
      String prefix = (sel == "Analog Sources") ? String("A") : String("D");
      for (auto& ctx : extra_context_names) {
        if (ctx.length() >= 1 && ctx.startsWith(prefix)) {
          if (pending_builders.count(ctx)) {
            auto& vec2 = pending_builders[ctx];
            std::vector<std::shared_ptr<sensesp::ConfigItemBase>> child_created;
            for (auto& b : vec2) {
              auto it = b();
              if (it) child_created.push_back(it);
            }
            if (pending_callbacks.count(ctx)) {
              for (auto& cb : pending_callbacks[ctx]) {
                cb(child_created);
              }
            }
          }
        }
      }
    }
  });
}

void ensure_configuration_manager() { ensure_config_manager(); }

void register_context_builders(const String& context_name,
                               const std::vector<ConfigBuilder>& builders,
                               CreatedItemsCallback created_items_cb) {
  // Register builders for the named context. Append builders if the context
  // was previously registered so multiple modules can contribute items to
  // the same context (e.g. A01..A04 builders appended to "Analog Sources").
  if (pending_builders.count(context_name)) {
    auto& vec = pending_builders[context_name];
    vec.insert(vec.end(), builders.begin(), builders.end());
  } else {
    pending_builders[context_name] = builders;
  }
  // Store the created-items callback so it can be invoked when builders are
  // instantiated for the selected context.
  if (created_items_cb) {
    pending_callbacks[context_name].push_back(created_items_cb);
  }
}

bool is_context_selected(const String& context_name) {
  if (!cfg_selector) return false;
  return cfg_selector->get_value() == context_name;
}

}  // namespace halmet
