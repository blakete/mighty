#include "goal_selector/goal_fence.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <nlohmann/json.hpp>

namespace goal_selector {

namespace {

// Even-odd test against one open ring (same rule as geofence_ws points_in_polygon).
bool insideRing(const std::vector<FencePoint>& r, double x, double y) {
  bool inside = false;
  for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
    const FencePoint& a = r[j];
    const FencePoint& b = r[i];
    if ((a.y > y) != (b.y > y)) {
      const double x_at = a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y);
      if (x < x_at) inside = !inside;
    }
  }
  return inside;
}

double segmentDist2(const FencePoint& a, const FencePoint& b, double x, double y) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double l2 = dx * dx + dy * dy;
  const double t = l2 > 0.0 ? std::clamp(((x - a.x) * dx + (y - a.y) * dy) / l2, 0.0, 1.0) : 0.0;
  const double px = a.x + t * dx - x, py = a.y + t * dy - y;
  return px * px + py * py;
}

bool closerThan(const std::vector<FencePoint>& r, double x, double y, double d2) {
  for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++)
    if (segmentDist2(r[j], r[i], x, y) < d2) return true;
  return false;
}

bool parseRing(const nlohmann::json& j, std::vector<FencePoint>& out) {
  if (!j.is_array() || j.size() < 3) return false;
  out.clear();
  for (const auto& p : j) {
    if (!p.is_array() || p.size() != 2 || !p[0].is_number() || !p[1].is_number()) return false;
    const double x = p[0].get<double>(), y = p[1].get<double>();
    if (!std::isfinite(x) || !std::isfinite(y)) return false;
    out.push_back({x, y});
  }
  return true;
}

}  // namespace

bool goalFenceAllows(const GoalFence& f, double x, double y, double extra_m) {
  if (f.keep_in.size() < 3 || !insideRing(f.keep_in, x, y)) return false;
  for (const auto& r : f.keep_out)
    if (r.size() >= 3 && insideRing(r, x, y)) return false;
  const double need = f.offset_m + std::max(0.0, extra_m);
  const double need2 = need * need;
  if (closerThan(f.keep_in, x, y, need2)) return false;
  for (const auto& r : f.keep_out)
    if (r.size() >= 2 && closerThan(r, x, y, need2)) return false;
  return true;
}

bool parseGoalFence(const std::string& json, GoalFence& out, std::string& frame_id, std::string& error) {
  const nlohmann::json j = nlohmann::json::parse(json, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object() || !j.contains("active") || !j["active"].is_boolean()) {
    error = "not a goal_fence message";
    return false;
  }
  GoalFence f;
  error.clear();
  f.active = j["active"].get<bool>();
  f.ready = j.contains("ready") && j["ready"].is_boolean() && j["ready"].get<bool>();
  frame_id = j.contains("frame_id") && j["frame_id"].is_string() ? j["frame_id"].get<std::string>() : "";
  if (f.ready) {
    auto bad = [&](const char* why) {
      if (f.ready) error = why;
      f.ready = false;
    };
    if (frame_id.empty()) bad("missing frame_id");
    if (!j.contains("offset_m") || !j["offset_m"].is_number() ||
        !std::isfinite(j["offset_m"].get<double>()) || j["offset_m"].get<double>() <= 0.0)
      bad("offset_m missing, non-finite or <= 0");
    else
      f.offset_m = j["offset_m"].get<double>();
    if (!j.contains("keep_in") || !parseRing(j["keep_in"], f.keep_in)) bad("keep_in malformed");
    if (j.contains("keep_out")) {
      if (!j["keep_out"].is_array()) {
        bad("keep_out is not a list");
      } else {
        for (const auto& r : j["keep_out"]) {
          std::vector<FencePoint> ring;
          if (!parseRing(r, ring)) {
            bad("keep_out ring malformed");
            break;
          }
          f.keep_out.push_back(std::move(ring));
        }
      }
    }
    if (!f.ready) {
      f.keep_in.clear();
      f.keep_out.clear();
    }
  }
  out = std::move(f);
  return true;
}

}  // namespace goal_selector
