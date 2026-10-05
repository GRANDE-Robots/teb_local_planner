#ifndef TEB_LOCAL_PLANNER_SWEPT_FOOTPRINT_H_
#define TEB_LOCAL_PLANNER_SWEPT_FOOTPRINT_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <mutex>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace teb_local_planner {

// One collision rule for planned segments and the command actually selected.
// All queries use this fixed observed-map snapshot, never simulator truth.
class SweptFootprint {
 public:
  using Polygon = std::vector<geometry_msgs::msg::Point>;

  SweptFootprint(nav2_costmap_2d::Costmap2D& map, const Polygon& hull,
                 const Polygon& padded, bool allow_unknown)
      : hull_(hull), padded_(padded) {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map.getMutex());
    resolution_ = map.getResolution();
    width_ = map.getSizeInCellsX();
    height_ = map.getSizeInCellsY();
    occupied_.assign(width_ * height_, false);
    observed_.assign(width_ * height_, false);
    component_.assign(width_ * height_, false);
    initial_raw_overlap_.assign(width_ * height_, false);
    left_ = map.getOriginX(); bottom_ = map.getOriginY();
    right_ = left_ + resolution_ * width_;
    top_ = bottom_ + resolution_ * height_;
    valid_ = std::isfinite(left_ + bottom_ + right_ + top_) &&
        std::isfinite(resolution_) && resolution_ > 0 && right_ > left_ &&
        top_ > bottom_ && convex(hull_) && convex(padded_);
    if (!valid_) return;
    epsilon_ = 128 * std::numeric_limits<double>::epsilon() *
        (1 + std::abs(left_) + std::abs(bottom_) + std::abs(right_) + std::abs(top_));
    for (std::size_t i = 0; i < padded_.size(); ++i) {
      const auto& a = padded_[i]; const auto& b = padded_[(i + 1) % padded_.size()];
      const auto& c = padded_[(i + 2) % padded_.size()];
      const double ex = b.x - a.x, ey = b.y - a.y;
      const double sign = ex * (c.y - a.y) - ey * (c.x - a.x) > 0 ? 1 : -1;
      for (const auto& p : hull_)
        if (sign * (ex * (p.y - a.y) - ey * (p.x - a.x)) < -epsilon_ * std::hypot(ex, ey)) {
          valid_ = false; return;
        }
    }
    for (unsigned int x = 0; x < width_; ++x)
      for (unsigned int y = 0; y < height_; ++y) {
        const auto cost = map.getCost(x, y);
        observed_[index(x, y)] = cost != nav2_costmap_2d::NO_INFORMATION;
        if (cost < nav2_costmap_2d::LETHAL_OBSTACLE ||
            (allow_unknown && cost == nav2_costmap_2d::NO_INFORMATION)) continue;
        Cell cell;
        cell.mx = x; cell.my = y;
        cell.unknown = cost == nav2_costmap_2d::NO_INFORMATION;
        map.mapToWorld(x, y, cell.x, cell.y);
        occupied_[index(x, y)] = true;
        cells_.push_back(cell);
      }
  }

  const std::string& reason() const { return reason_; }
  const Polygon& footprint() const { return padded_; }
  bool initialOverlap() const { return initial_overlap_; }

  bool begin(double x, double y, double yaw) {
    reason_.clear(); planes_.clear(); begun_ = false; initial_overlap_ = false;
    std::fill(component_.begin(), component_.end(), false);
    std::fill(initial_raw_overlap_.begin(), initial_raw_overlap_.end(), false);
    if (!valid_ || !std::isfinite(x + y + yaw)) return fail("invalid_geometry");
    pose_ = {x, y, std::atan2(std::sin(yaw), std::cos(yaw))};
    const Polygon raw = at(hull_, pose_), padded = at(padded_, pose_);
    if (!insideMap(padded)) return fail("initial_footprint_outside_map");
    std::vector<Cell> contacts;
    for (const auto& cell : cells_) {
      const double raw_gap = separation(raw, cell);
      // The observed map and localization can place the initial hull inside a
      // known occupied boundary. Capture that boundary as the baseline rather
      // than inventing a penetration-depth cutoff: subsequent motion may keep
      // or improve the initial support, but it may never worsen it. Unknown
      // space has no measured boundary from which safe separation can be
      // established, so raw-hull overlap with unknown space remains invalid.
      if (raw_gap < -epsilon_) {
        if (cell.unknown) return fail("initial_hull_in_unknown_space");
        initial_raw_overlap_[index(cell.mx, cell.my)] = true;
      }
      // Unknown space is a hard boundary for the physical hull, but it has no
      // measured surface against which a preferred padding distance can be
      // defined. Known obstacles retain the full padded-footprint rule.
      if (!cell.unknown && separation(padded, cell) < -epsilon_)
        contacts.push_back(cell);
    }

    // Infer contact planes from exposed occupied-grid faces. The former
    // per-cell closest-axis rule could turn corner cells on one straight wall
    // into a fictitious perpendicular wall. Exposed faces describe the
    // boundary that collision checking actually sees: one plane for a wall,
    // and two planes only where the occupied geometry contains a corner.
    std::vector<Plane> candidates;
    const auto add_face = [&](const Cell& cell, int dx, int dy,
                              double nx, double ny) {
      if (cell.unknown ||
          !observedFree(static_cast<int>(cell.mx) + dx,
                        static_cast<int>(cell.my) + dy)) return;
      // The face normal points from the occupied cell into observed free
      // space. The raw hull must remain on, or improve from, its initial side
      // of this boundary; the padded hull may overlap while it separates.
      const double boundary = nx * cell.x + ny * cell.y +
          resolution_ / 2 * (std::abs(nx) + std::abs(ny));
      const Range raw_range = projection(raw, nx, ny);
      const Range padded_range = projection(padded, nx, ny);
      double center = 0;
      for (const auto& point : raw) center += nx * point.x + ny * point.y;
      center /= raw.size();
      if (center < boundary - epsilon_ ||
          padded_range.low > boundary + epsilon_) return;
      // Existing free clearance is available for a maneuver. If the raw hull
      // begins outside the wall, its hard lower bound is the physical boundary,
      // not its initial clearance. If localization places it inside, retain
      // that initial support as the worst allowed value without deepening it.
      Plane plane{nx, ny, boundary, std::min(raw_range.low, boundary)};
      for (auto& old : candidates)
        if (old.nx == nx && old.ny == ny) {
          old.boundary = std::max(old.boundary, boundary);
          old.minimum = std::max(old.minimum, plane.minimum);
          return;
        }
      candidates.push_back(plane);
    };
    for (const auto& cell : contacts) {
      add_face(cell, -1, 0, -1, 0);
      add_face(cell, 1, 0, 1, 0);
      add_face(cell, 0, -1, 0, -1);
      add_face(cell, 0, 1, 0, 1);
    }

    // Keep every distinct exposed boundary normal. Greedily covering contact
    // cells can collapse a real corner into one half-space because cells from
    // the second wall also lie behind the first wall's infinite plane. At most
    // four axis-aligned normals exist in this raster map, so retaining all of
    // them is both smaller code and the correct corner constraint.
    for (const auto& cell : contacts) {
      bool resolved = false;
      for (const auto& candidate : candidates)
        if (behind(cell, candidate)) { resolved = true; break; }
      if (!resolved) return fail("initial_contact_boundary_unresolved");
    }
    planes_ = candidates;

    // Limit relaxed padding checks to known occupied components that touch the
    // initial padded hull. Unrelated obstacles remain subject to the ordinary
    // padded swept-footprint check.
    std::vector<std::size_t> frontier;
    for (const auto& contact : contacts) {
      if (contact.unknown) continue;
      const std::size_t seed = index(contact.mx, contact.my);
      if (!component_[seed]) {
        component_[seed] = true;
        frontier.push_back(seed);
      }
    }
    // Rasterized diagonal walls form one physical obstacle even when their
    // occupied squares touch only at a corner.  Treat the full 8-connected
    // component as the initially contacted surface so a safe separating arc
    // is not rejected by the next diagonal cell as a new obstacle.
    const int steps[8][2] = {
        {-1, -1}, {-1, 0}, {-1, 1}, {0, -1},
        {0, 1}, {1, -1}, {1, 0}, {1, 1}};
    for (std::size_t cursor = 0; cursor < frontier.size(); ++cursor) {
      const std::size_t current = frontier[cursor];
      const int x = static_cast<int>(current / height_);
      const int y = static_cast<int>(current % height_);
      for (const auto& step : steps) {
        const int nx = x + step[0], ny = y + step[1];
        if (nx < 0 || ny < 0 || nx >= static_cast<int>(width_) ||
            ny >= static_cast<int>(height_)) continue;
        const std::size_t next = index(static_cast<unsigned int>(nx),
                                       static_cast<unsigned int>(ny));
        if (!component_[next] && occupied_[next] && observed_[next]) {
          component_[next] = true;
          frontier.push_back(next);
        }
      }
    }
    initial_overlap_ = !contacts.empty();
    begun_ = true;
    return true;
  }

  // Body-frame constant velocity integrates an exact SE(2) arc. The optional
  // world-frame form preserves straight-center interpolation used by other
  // robot models while applying the identical swept-footprint rule.
  bool advance(double vx, double vy, double omega, double duration,
               bool world_velocity = false) {
    if (!begun_ || !std::isfinite(vx + vy + omega + duration) || duration <= 0)
      return fail("invalid_motion");
    Motion motion{pose_, vx, vy, omega, duration, world_velocity};
    if (std::abs(omega * duration) > 8 * pi()) return fail("motion_rotation_unbounded");
    const auto events = supportEvents(motion);
    for (std::size_t i = 1; i < events.size(); ++i) {
      const double interval = events[i] - events[i - 1];
      if (interval <= 0) continue;
      Motion part{pose_, vx, vy, omega, interval, world_velocity};
      if (!advanceMotion(part)) return false;
    }
    return true;
  }

  // Extra unknown-space clearance belongs only to the terminal sensing pose.
  // Use Euclidean polygon/cell distance; SAT separation alone underestimates
  // diagonal clearance. Ordinary navigation requests zero extra margin.
  bool terminalSensingClearance(double margin) {
    if (!begun_ || !std::isfinite(margin) || margin < 0)
      return fail("invalid_terminal_unknown_clearance");
    if (margin == 0) return true;
    const Polygon polygon = at(padded_, pose_);
    // A frontier sensing endpoint must have cleared the initial known buffer
    // overlap, even though an ordinary escape route may still be separating.
    for (const auto& cell : cells_)
      if (!cell.unknown && separation(polygon,cell) < -epsilon_)
        return fail("terminal_padded_hull_in_obstacle");
    double xmin=right_, xmax=left_, ymin=top_, ymax=bottom_;
    for (const auto& p : polygon) {
      if (p.x-margin < left_-epsilon_ || p.x+margin > right_+epsilon_ ||
          p.y-margin < bottom_-epsilon_ || p.y+margin > top_+epsilon_)
        return fail("terminal_unknown_clearance_outside_map");
      xmin=std::min(xmin,p.x); xmax=std::max(xmax,p.x);
      ymin=std::min(ymin,p.y); ymax=std::max(ymax,p.y);
    }
    const auto point_segment = [](const geometry_msgs::msg::Point& p,
                                  const geometry_msgs::msg::Point& a,
                                  const geometry_msgs::msg::Point& b) {
      const double dx=b.x-a.x, dy=b.y-a.y;
      const double t=std::max(0.0,std::min(1.0,
          ((p.x-a.x)*dx+(p.y-a.y)*dy)/(dx*dx+dy*dy)));
      return std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
    };
    const int x0=std::max(0,int(std::floor((xmin-margin-left_)/resolution_)));
    const int x1=std::min(int(width_)-1,int(std::floor((xmax+margin-left_)/resolution_)));
    const int y0=std::max(0,int(std::floor((ymin-margin-bottom_)/resolution_)));
    const int y1=std::min(int(height_)-1,int(std::floor((ymax+margin-bottom_)/resolution_)));
    for (int y=y0;y<=y1;++y) for (int x=x0;x<=x1;++x) {
      if (observed_[index(x,y)]) continue;
      Cell cell;
      cell.x=left_+(x+.5)*resolution_; cell.y=bottom_+(y+.5)*resolution_;
      if (separation(polygon,cell)<=epsilon_)
        return fail("terminal_padded_hull_overlaps_unknown_space");
      Polygon square(4);
      for (int j=0;j<4;++j) {
        square[j].x=cell.x+resolution_*.5*((j==0 || j==3)?-1:1);
        square[j].y=cell.y+resolution_*.5*(j<2?-1:1);
      }
      double distance=std::numeric_limits<double>::infinity();
      for (std::size_t i=0;i<polygon.size();++i) for (int j=0;j<4;++j) {
        distance=std::min(distance,point_segment(
            polygon[i],square[j],square[(j+1)%4]));
        distance=std::min(distance,point_segment(
            square[j],polygon[i],polygon[(i+1)%polygon.size()]));
      }
      if (distance+epsilon_<margin)
        return fail("terminal_unknown_clearance_violation");
    }
    return true;
  }

 private:
  struct Pose { double x, y, yaw; };
  struct Cell {
    double x = 0, y = 0;
    unsigned int mx = 0, my = 0;
    bool unknown = false;
  };
  struct Plane { double nx = 0, ny = 0, boundary = 0, minimum = 0; };
  struct Range { double low, high; };
  struct Motion { Pose from; double vx, vy, omega, duration; bool world; };
  Polygon hull_, padded_;
  std::vector<Cell> cells_;
  std::vector<bool> occupied_, observed_, component_, initial_raw_overlap_;
  std::vector<Plane> planes_;
  Pose pose_{0, 0, 0};
  unsigned int width_ = 0, height_ = 0;
  double resolution_ = 0, left_ = 0, bottom_ = 0, right_ = 0, top_ = 0;
  double epsilon_ = 0;
  bool valid_ = false, begun_ = false, initial_overlap_ = false;
  std::string reason_;

  bool advanceMotion(const Motion& motion) {
    const auto xb = bounds(padded_, motion, 1, 0, 0, motion.duration);
    const auto yb = bounds(padded_, motion, 0, 1, 0, motion.duration);
    if (xb.low < left_ - epsilon_ || xb.high > right_ + epsilon_ ||
        yb.low < bottom_ - epsilon_ || yb.high > top_ + epsilon_)
      return fail("swept_footprint_outside_map");
    for (const auto& plane : planes_)
      if (bounds(hull_, motion, plane.nx, plane.ny, 0, motion.duration).low <
          plane.minimum - epsilon_)
        return fail("initial_clearance_worsened");
    for (const auto& cell : cells_) {
      const double half = resolution_ / 2;
      if (cell.x + half < xb.low || cell.x - half > xb.high ||
          cell.y + half < yb.low || cell.y - half > yb.high) continue;
      const std::size_t cell_index = index(cell.mx, cell.my);
      if (cell.unknown) {
        if (!intervalClear(hull_, motion, cell, 0, motion.duration, 0, false))
          return false;
        continue;
      }
      bool in_contact_halfspace = false;
      bool on_contact_boundary = false;
      for (const auto& plane : planes_)
        if (behind(cell, plane)) {
          in_contact_halfspace = true;
          on_contact_boundary =
              on_contact_boundary || onBoundary(cell, plane);
        }
      if (component_[cell_index] && in_contact_halfspace) {
        // Padding is a preferred buffer, so it may overlap the initially
        // contacted wall while the physical hull separates. A previously
        // untouched protrusion or interior surface still receives a raw-hull
        // swept check; only the coherent contacted face and cells already
        // intersected at the initial estimate use the monotone support bound.
        if (initial_raw_overlap_[cell_index] || on_contact_boundary) continue;
        if (!intervalClear(hull_, motion, cell, 0, motion.duration, 0, false))
          return false;
        continue;
      }
      if (!intervalClear(padded_, motion, cell, 0, motion.duration, 0, true))
        return false;
    }
    pose_ = position(motion, motion.duration);
    if (!planes_.empty()) {
      const Polygon padded = at(padded_, pose_);
      planes_.erase(std::remove_if(planes_.begin(), planes_.end(),
          [&](const Plane& plane) {
            return projection(padded, plane.nx, plane.ny).low >=
                plane.boundary - epsilon_;
          }), planes_.end());
    }
    return true;
  }

  static double pi() { return 3.14159265358979323846; }
  bool fail(const char* reason) { reason_ = reason; return false; }

  std::size_t index(unsigned int x, unsigned int y) const {
    return static_cast<std::size_t>(x) * height_ + y;
  }

  bool observedFree(int x, int y) const {
    if (x < 0 || y < 0 || x >= static_cast<int>(width_) ||
        y >= static_cast<int>(height_)) return false;
    const std::size_t cell = index(static_cast<unsigned int>(x),
                                   static_cast<unsigned int>(y));
    return observed_[cell] && !occupied_[cell];
  }

  bool onBoundary(const Cell& cell, const Plane& plane) const {
    const int dx = static_cast<int>(plane.nx);
    const int dy = static_cast<int>(plane.ny);
    if (!observedFree(static_cast<int>(cell.mx) + dx,
                      static_cast<int>(cell.my) + dy)) return false;
    const double face = plane.nx * cell.x + plane.ny * cell.y +
        resolution_ / 2 * (std::abs(plane.nx) + std::abs(plane.ny));
    return std::abs(face - plane.boundary) <= epsilon_;
  }

  bool behind(const Cell& cell, const Plane& plane) const {
    return plane.nx * cell.x + plane.ny * cell.y + resolution_ / 2 *
        (std::abs(plane.nx) + std::abs(plane.ny)) <= plane.boundary + epsilon_;
  }

  static Range projection(const Polygon& polygon, double nx, double ny) {
    Range range{std::numeric_limits<double>::infinity(),
                -std::numeric_limits<double>::infinity()};
    for (const auto& point : polygon) {
      const double value = nx * point.x + ny * point.y;
      range.low = std::min(range.low, value);
      range.high = std::max(range.high, value);
    }
    return range;
  }

  static bool convex(const Polygon& polygon) {
    if (polygon.size() < 3) return false;
    double winding = 0;
    for (std::size_t i = 0; i < polygon.size(); ++i) {
      const auto& a = polygon[i]; const auto& b = polygon[(i + 1) % polygon.size()];
      if (!std::isfinite(a.x + a.y) || !std::isfinite(b.x + b.y) ||
          std::hypot(b.x - a.x, b.y - a.y) == 0) return false;
      for (std::size_t j = 0; j < polygon.size(); ++j) {
        if (j == i || j == (i + 1) % polygon.size()) continue;
        const double side = (b.x - a.x) * (polygon[j].y - a.y) -
                            (b.y - a.y) * (polygon[j].x - a.x);
        if (!std::isfinite(side) || side == 0) return false;
        if (winding == 0) winding = side;
        if (side * winding <= 0) return false;
      }
    }
    return true;
  }

  static Polygon at(const Polygon& polygon, const Pose& pose) {
    Polygon result = polygon;
    const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
    for (auto& p : result) {
      const double x = p.x, y = p.y;
      p.x = pose.x + c * x - s * y; p.y = pose.y + s * x + c * y;
    }
    return result;
  }

  bool insideMap(const Polygon& polygon) const {
    for (const auto& p : polygon)
      if (p.x < left_ - epsilon_ || p.x > right_ + epsilon_ ||
          p.y < bottom_ - epsilon_ || p.y > top_ + epsilon_) return false;
    return true;
  }

  // Exact convex-polygon / square SAT at one instant. Zero separation is
  // contact, not penetration; epsilon only accommodates floating arithmetic.
  double separation(const Polygon& polygon, const Cell& cell) const {
    double best = -std::numeric_limits<double>::infinity();
    const auto axis = [&](double nx, double ny) {
      double low = std::numeric_limits<double>::infinity(), high = -low;
      for (const auto& p : polygon) {
        const double value = nx * p.x + ny * p.y;
        low = std::min(low, value); high = std::max(high, value);
      }
      const double center = nx * cell.x + ny * cell.y;
      const double reach = resolution_ / 2 * (std::abs(nx) + std::abs(ny));
      best = std::max(best, center - reach - high);
      best = std::max(best, low - center - reach);
    };
    axis(1, 0); axis(0, 1);
    for (std::size_t i = 0; i < polygon.size(); ++i) {
      const auto& a = polygon[i]; const auto& b = polygon[(i + 1) % polygon.size()];
      const double length = std::hypot(b.x - a.x, b.y - a.y);
      axis(-(b.y - a.y) / length, (b.x - a.x) / length);
    }
    return best;
  }

  static Pose position(const Motion& motion, double time) {
    Pose out = motion.from;
    const double turn = motion.omega * time;
    if (motion.world) {
      out.x += motion.vx * time; out.y += motion.vy * time;
    } else {
      const double h = turn / 2;
      const double sinc = std::abs(h) < 1e-8 ? 1 - h * h / 6 : std::sin(h) / h;
      const double c = std::cos(out.yaw + h), s = std::sin(out.yaw + h);
      out.x += time * sinc * (c * motion.vx - s * motion.vy);
      out.y += time * sinc * (s * motion.vx + c * motion.vy);
    }
    out.yaw += turn;
    return out;
  }

  template <class Visit>
  static void angles(const Motion& motion, double start, double end, double root, Visit visit) {
    if (motion.omega == 0) return;
    const double from = motion.from.yaw + motion.omega * start;
    const double to = motion.from.yaw + motion.omega * end;
    const int first = static_cast<int>(std::ceil((std::min(from, to) - root) / (2 * pi())));
    const int last = static_cast<int>(std::floor((std::max(from, to) - root) / (2 * pi())));
    for (int k = first; k <= last; ++k) {
      const double time = (root + 2 * pi() * k - motion.from.yaw) / motion.omega;
      if (time >= start && time <= end) visit(time);
    }
  }

  template <class Visit>
  static void extrema(const geometry_msgs::msg::Point& vertex, const Motion& motion,
                      double nx, double ny, double start, double end, Visit visit) {
    visit(start); visit(end);
    const double dx = (motion.world ? 0 : motion.vx) - motion.omega * vertex.y;
    const double dy = (motion.world ? 0 : motion.vy) + motion.omega * vertex.x;
    const double a = nx * dx + ny * dy, b = -nx * dy + ny * dx;
    const double constant = motion.world ? nx * motion.vx + ny * motion.vy : 0;
    const double amplitude = std::hypot(a, b);
    if (amplitude == 0 || std::abs(constant) > amplitude) return;
    const double phase = std::atan2(b, a);
    const double angle = std::acos(std::max(-1.0, std::min(1.0, -constant / amplitude)));
    angles(motion, start, end, phase + angle, visit);
    angles(motion, start, end, phase - angle, visit);
  }

  // Split the initial-overlap phase at support extrema, active-vertex changes
  // and padding-plane crossings. This makes progress monotone within each
  // interval and restores the full padding before a later motion can return.
  std::vector<double> supportEvents(const Motion& motion) const {
    std::vector<double> events{0, motion.duration};
    if (planes_.empty()) return events;
    const auto add = [&](double time) { events.push_back(time); };
    for (const auto& plane : planes_)
      for (const Polygon* polygon : {&hull_, &padded_})
        for (std::size_t i = 0; i < polygon->size(); ++i) {
          const auto& a = (*polygon)[i]; const auto& b = (*polygon)[(i + 1) % polygon->size()];
          extrema(a, motion, plane.nx, plane.ny, 0, motion.duration, add);
          const double dx = b.x - a.x, dy = b.y - a.y;
          const double phase = std::atan2(-plane.nx * dy + plane.ny * dx,
                                         plane.nx * dx + plane.ny * dy);
          angles(motion, 0, motion.duration, phase + pi() / 2, add);
          angles(motion, 0, motion.duration, phase - pi() / 2, add);
        }
    std::sort(events.begin(), events.end());
    events.erase(std::unique(events.begin(), events.end()), events.end());
    const std::vector<double> monotone = events;
    for (const auto& plane : planes_) {
      const auto support = [&](double time) {
        return bounds(padded_, motion, plane.nx, plane.ny, time, time).low - plane.boundary;
      };
      for (std::size_t i = 1; i < monotone.size(); ++i) {
        double low = monotone[i - 1], high = monotone[i];
        const double initial = support(low), final = support(high);
        if (initial * final >= 0) continue;
        for (unsigned step = 0; step < 56; ++step) {
          const double middle = (low + high) / 2;
          if ((support(middle) > 0) == (initial > 0)) low = middle;
          else high = middle;
        }
        events.push_back((low + high) / 2);
      }
    }
    std::sort(events.begin(), events.end());
    events.erase(std::unique(events.begin(), events.end()), events.end());
    return events;
  }

  // Extrema of each vertex projection occur at the endpoints and at zeros
  // of its analytic derivative. No isotropic sampling inflation is needed.
  Range bounds(const Polygon& polygon, const Motion& motion, double nx, double ny,
               double start, double end) const {
    Range range{std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
    for (const auto& vertex : polygon) {
      const auto evaluate = [&](double time) {
        const Pose pose = position(motion, time);
        const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
        const double value = nx * (pose.x + c * vertex.x - s * vertex.y) +
                            ny * (pose.y + s * vertex.x + c * vertex.y);
        range.low = std::min(range.low, value); range.high = std::max(range.high, value);
      };
      extrema(vertex, motion, nx, ny, start, end, evaluate);
    }
    return range;
  }

  bool intervalClear(const Polygon& polygon, const Motion& motion,
                     const Cell& cell, double start, double end,
                     unsigned depth, bool padding) {
    const auto separated = [&](double nx, double ny) {
      const Range swept = bounds(polygon, motion, nx, ny, start, end);
      const double center = nx * cell.x + ny * cell.y;
      const double reach = resolution_ / 2 * (std::abs(nx) + std::abs(ny));
      return swept.high <= center - reach + epsilon_ ||
             swept.low >= center + reach - epsilon_;
    };
    if (separated(1, 0) || separated(0, 1)) return true;
    const double middle = (start + end) / 2;
    for (double time : {start, middle, end}) {
      const Polygon placed = at(polygon, position(motion, time));
      if (separation(placed, cell) < -epsilon_)
        return fail(cell.unknown ? "swept_footprint_in_unknown_space" :
                    (padding ? "padded_sweep_intersection" :
                               "new_hull_intersection"));
      for (std::size_t i = 0; i < placed.size(); ++i) {
        const auto& a = placed[i]; const auto& b = placed[(i + 1) % placed.size()];
        const double length = std::hypot(b.x - a.x, b.y - a.y);
        if (separated(-(b.y - a.y) / length, (b.x - a.x) / length)) return true;
      }
    }
    // An unresolved interval is never declared clear. This numerical work
    // bound does not relax the geometry or create a new physical tolerance.
    if (depth >= 20) return fail("sweep_separation_unresolved");
    return intervalClear(polygon, motion, cell, start, middle,
                         depth + 1, padding) &&
           intervalClear(polygon, motion, cell, middle, end,
                         depth + 1, padding);
  }
};

}  // namespace teb_local_planner
#endif
