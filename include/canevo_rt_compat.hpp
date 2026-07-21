#pragma once

#include <spdlog/spdlog.h>

#ifndef CANEVO_HAVE_NECRO
#define CANEVO_HAVE_NECRO 0
#endif

#if CANEVO_HAVE_NECRO
#include <qiuniu/wrappers.h>
#else
#ifndef __RT
#define __RT(expr) (expr)
#endif
#endif

inline const char *CanevoHardRealtimeStatus() {
#if CANEVO_HAVE_NECRO
  return "ON (__RT -> qiuniu)";
#else
  return "OFF (POSIX fallback)";
#endif
}

inline void LogCanevoHardRealtimeOnce() {
  static bool logged = false;
  if (logged) {
    return;
  }
  logged = true;
  spdlog::info("[CanEvo SDK] NIIC hard realtime: {}",
               CanevoHardRealtimeStatus());
}
