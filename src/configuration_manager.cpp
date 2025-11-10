// Backwards-compatibility shim.
// The real implementation lives in config_manager.{h,cpp}.
// This file intentionally defines no symbols to avoid duplicate definitions.

#include "config_manager.h"

// Intentionally empty. Existing code may include "configuration_manager.h";
// that header now forwards to the new API in config_manager.h.
