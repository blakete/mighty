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

namespace {

// Crossing x positions of one ring on the horizontal line y, by insideRing()'s rule: insideRing(x, y)
// is true iff an odd number of them are > x.
void ringCrossings(const std::vector<FencePoint>& r, double y, std::vector<double>& xs) {
  xs.clear();
  for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
    const FencePoint& a = r[j];
    const FencePoint& b = r[i];
    if ((a.y > y) != (b.y > y)) xs.push_back(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
  }
  std::sort(xs.begin(), xs.end());
}

// insideRing(x, y) for a row's sorted crossings.
bool insideByCrossings(const std::vector<double>& xs, double x) {
  const auto above = xs.end() - std::upper_bound(xs.begin(), xs.end(), x);  // crossings with x < x_at
  return (above & 1) != 0;
}

// Mark non-forbidden cells closer than band_m to the edges of ring r (closed) as band.
void markBand(const std::vector<FencePoint>& r, int dim_x, int dim_y, double res, double origin_x,
              double origin_y, double band_m, std::vector<uint8_t>& out) {
  const double band2 = band_m * band_m;
  for (size_t i = 0, j = r.size() - 1; i < r.size(); j = i++) {
    const FencePoint& a = r[j];
    const FencePoint& b = r[i];
    // Cells whose centre lies in the edge's bbox grown by band_m.
    const int x0 = std::max(0, static_cast<int>(std::ceil((std::min(a.x, b.x) - band_m - origin_x) / res - 0.5)));
    const int x1 = std::min(dim_x - 1, static_cast<int>(std::floor((std::max(a.x, b.x) + band_m - origin_x) / res - 0.5)));
    const int y0 = std::max(0, static_cast<int>(std::ceil((std::min(a.y, b.y) - band_m - origin_y) / res - 0.5)));
    const int y1 = std::min(dim_y - 1, static_cast<int>(std::floor((std::max(a.y, b.y) + band_m - origin_y) / res - 0.5)));
    for (int y = y0; y <= y1; ++y) {
      const double cy = origin_y + (y + 0.5) * res;
      for (int x = x0; x <= x1; ++x) {
        uint8_t& cell = out[static_cast<size_t>(x) + static_cast<size_t>(dim_x) * y];
        if (cell != kFenceAllowed) continue;
        if (segmentDist2(a, b, origin_x + (x + 0.5) * res, cy) < band2) cell = kFenceBand;
      }
    }
  }
}

}  // namespace

void rasterizeGoalFence(const GoalFence& f, int dim_x, int dim_y, double res, double origin_x,
                        double origin_y, double band_m, std::vector<uint8_t>& out) {
  out.assign(static_cast<size_t>(std::max(0, dim_x)) * std::max(0, dim_y), kFenceAllowed);
  if (!f.ready || f.keep_in.size() < 3 || dim_x <= 0 || dim_y <= 0 || res <= 0.0) return;

  // Forbidden: row by row, the same even-odd crossings goalFenceAllows() evaluates per point.
  std::vector<double> in_xs, out_xs;
  for (int y = 0; y < dim_y; ++y) {
    const double cy = origin_y + (y + 0.5) * res;
    ringCrossings(f.keep_in, cy, in_xs);
    uint8_t* row = out.data() + static_cast<size_t>(dim_x) * y;
    for (int x = 0; x < dim_x; ++x)
      if (!insideByCrossings(in_xs, origin_x + (x + 0.5) * res)) row[x] = kFenceForbidden;
    for (const auto& ring : f.keep_out) {
      if (ring.size() < 3) continue;
      ringCrossings(ring, cy, out_xs);
      if (out_xs.empty()) continue;
      for (int x = 0; x < dim_x; ++x)
        if (insideByCrossings(out_xs, origin_x + (x + 0.5) * res)) row[x] = kFenceForbidden;
    }
  }
  if (band_m <= 0.0) return;
  markBand(f.keep_in, dim_x, dim_y, res, origin_x, origin_y, band_m, out);
  for (const auto& ring : f.keep_out)
    if (ring.size() >= 2) markBand(ring, dim_x, dim_y, res, origin_x, origin_y, band_m, out);
}

bool parseGateEcho(const std::string& json, bool& engaged) {
  const nlohmann::json j = nlohmann::json::parse(json, nullptr, /*allow_exceptions=*/false);
  if (j.is_discarded() || !j.is_object() || !j.contains("engaged") || !j["engaged"].is_boolean()) return false;
  engaged = j["engaged"].get<bool>();
  return true;
}

}  // namespace goal_selector
