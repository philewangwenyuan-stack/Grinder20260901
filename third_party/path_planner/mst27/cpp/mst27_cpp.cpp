// mst27 v4.7.0：60度柱边折角 + 可调膨胀/躲避距离
// pybind11 仅在构建 Python 扩展时需要。若找不到其头文件（例如脱离
// Python 单独编译内核算法做单元测试），则自动编译掉 pybind 部分。
// 你们的 catkin/cmake 构建环境带 pybind11，行为与原来完全一致。
#if defined(__has_include)
#  if __has_include(<pybind11/pybind11.h>)
#    define MST27_HAVE_PYBIND 1
#  else
#    define MST27_HAVE_PYBIND 0
#  endif
#else
#  define MST27_HAVE_PYBIND 1
#endif

#if MST27_HAVE_PYBIND
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#endif

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#if MST27_HAVE_PYBIND
namespace py = pybind11;
#endif

struct Point {
    int row = 0;
    int col = 0;
};

// 连续坐标点（亚格精度），用于任意角度路径点位输出
struct FPoint {
    double row = 0.0;
    double col = 0.0;
};

// 在线保留线段端点和真实角点。共线点直接延长上一线段，O(1)/点；
// 避免先生成密集路径，再执行全路径 Douglas-Peucker。
static void append_corner(std::vector<FPoint>& dst, const FPoint& p) {
    constexpr double eps = 1e-7;
    if (!dst.empty()) {
        const double dc = p.col - dst.back().col;
        const double dr = p.row - dst.back().row;
        if (dc * dc + dr * dr <= eps * eps) {
            dst.back() = p;
            return;
        }
    }
    if (dst.size() >= 2) {
        const FPoint& a = dst[dst.size() - 2];
        const FPoint& b = dst.back();
        const double v1c = b.col - a.col, v1r = b.row - a.row;
        const double v2c = p.col - b.col, v2r = p.row - b.row;
        const double cross = v1c * v2r - v1r * v2c;
        const double scale = std::max(1.0,
            std::hypot(v1c, v1r) * std::hypot(v2c, v2r));
        if (std::abs(cross) <= eps * scale &&
            v1c * v2c + v1r * v2r >= 0.0) {
            dst.back() = p;
            return;
        }
    }
    dst.push_back(p);
}

template <class It>
static void append_corners(std::vector<FPoint>& dst, It first, It last) {
    for (; first != last; ++first) append_corner(dst, *first);
}

struct PathItem {
    double row = 0.0;
    double col = 0.0;
    std::string path_type;
};

struct Grid {
    int rows = 0;
    int cols = 0;
    std::vector<std::uint8_t> blocked;

    bool in_bounds(int row, int col) const {
        return 0 <= row && row < rows && 0 <= col && col < cols;
    }

    bool is_free(int row, int col) const {
        return in_bounds(row, col) && blocked[index(row, col)] == 0;
    }

    std::size_t index(int row, int col) const {
        return static_cast<std::size_t>(row) * static_cast<std::size_t>(cols) +
               static_cast<std::size_t>(col);
    }
};

struct StageInput {
    int stage_id = 0;
    std::vector<Point> boundary;
    std::optional<Point> start_point;
    Point end_point;
    std::string direction;
    std::optional<double> direction_angle;  // 任意角度规划（度），无则轴向扫描
    double endpoint_margin_pixels = 0.0;     // 沿扫描方向，两端距边界的留边
    double obstacle_corner_angle_deg = 45.0; // 柱边进入/退出坡相对扫描方向的夹角
    double obstacle_avoidance_pixels = 0.0;  // 膨胀柱外的额外路径躲避距离
    std::vector<std::uint8_t> region_mask;
    std::optional<Grid> planning_grid;       // 本阶段角度对齐矩形膨胀图
};

static std::string direction_axis(std::string direction) {
    std::transform(direction.begin(), direction.end(), direction.begin(),
                   [](unsigned char ch) {
                       return static_cast<char>(std::tolower(ch));
                   });
    if (!direction.empty() && (direction.front() == '-' || direction.front() == '+')) {
        direction.erase(direction.begin());
    }
    return direction == "y" ? "y" : "x";
}

static int direction_travel_sign(const std::string& direction) {
    return !direction.empty() && direction.front() == '-' ? -1 : 1;
}

struct HeapNode {
    double priority = 0.0;
    int row = 0;
    int col = 0;
};

struct HeapCompare {
    bool operator()(const HeapNode& lhs, const HeapNode& rhs) const {
        if (lhs.priority != rhs.priority) {
            return lhs.priority > rhs.priority;
        }
        if (lhs.row != rhs.row) {
            return lhs.row > rhs.row;
        }
        return lhs.col > rhs.col;
    }
};

#if MST27_HAVE_PYBIND
static Point tuple_to_point(const py::handle& obj) {
    auto tuple = py::cast<py::tuple>(obj);
    if (tuple.size() != 2) {
        throw std::runtime_error("Point tuples must contain exactly two values");
    }
    return Point{tuple[0].cast<int>(), tuple[1].cast<int>()};
}
#endif

static std::uint64_t visit_key(int row, int col) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(row)) << 32U) |
           static_cast<std::uint32_t>(col);
}

static bool mask_at(const StageInput& stage, const Grid& grid, int row, int col) {
    return grid.in_bounds(row, col) && stage.region_mask[grid.index(row, col)] != 0;
}

static Point find_nearest_free_point(const Grid& grid, Point p) {
    if (grid.is_free(p.row, p.col)) {
        return p;
    }

    static const Point moves[] = {
        {0, 1}, {0, -1}, {1, 0}, {-1, 0},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1},
    };

    std::deque<Point> queue;
    std::unordered_set<std::uint64_t> visited;
    queue.push_back(p);
    visited.insert(visit_key(p.row, p.col));

    const int max_search = 1000;
    int count = 0;

    while (!queue.empty() && count < max_search) {
        Point current = queue.front();
        queue.pop_front();

        if (grid.is_free(current.row, current.col)) {
            return current;
        }

        for (const auto& move : moves) {
            Point next{current.row + move.row, current.col + move.col};
            if (grid.in_bounds(next.row, next.col)) {
                const auto key = visit_key(next.row, next.col);
                if (visited.find(key) == visited.end()) {
                    visited.insert(key);
                    queue.push_back(next);
                }
            }
        }
        ++count;
    }

    return p;
}

static Point find_farthest_corner(const std::vector<Point>& polygon, Point reference_point) {
    if (polygon.empty()) {
        return reference_point;
    }

    double max_dist = -1.0;
    Point farthest = polygon.front();
    for (const auto& point : polygon) {
        const double dr = static_cast<double>(point.row - reference_point.row);
        const double dc = static_cast<double>(point.col - reference_point.col);
        const double dist = std::sqrt(dr * dr + dc * dc);
        if (dist > max_dist) {
            max_dist = dist;
            farthest = point;
        }
    }
    return farthest;
}

static std::vector<Point> astar_search(const Grid& grid, Point start, Point goal) {
    if (start.row == goal.row && start.col == goal.col) {
        return {start};
    }
    if (!grid.in_bounds(start.row, start.col) || !grid.in_bounds(goal.row, goal.col)) {
        return {};
    }

    static const struct {
        int dr;
        int dc;
        double cost;
    } movements[] = {
        {0, 1, 1.0}, {0, -1, 1.0}, {1, 0, 1.0}, {-1, 0, 1.0},
        {1, 1, 1.414}, {1, -1, 1.414}, {-1, 1, 1.414}, {-1, -1, 1.414},
    };

    std::priority_queue<HeapNode, std::vector<HeapNode>, HeapCompare> open_set;
    std::unordered_map<std::uint64_t, double> g_score;
    std::unordered_map<std::uint64_t, Point> came_from;
    open_set.push(HeapNode{0.0, start.row, start.col});
    g_score.emplace(visit_key(start.row, start.col), 0.0);

    const int max_steps = 500000;
    int steps = 0;

    while (!open_set.empty() && steps < max_steps) {
        ++steps;
        const HeapNode heap_current = open_set.top();
        open_set.pop();
        const Point current{heap_current.row, heap_current.col};

        if (current.row == goal.row && current.col == goal.col) {
            // 沿父链直接提取方向变化点，不构造逐格路径。
            std::vector<Point> path{current};
            Point node = current;
            Point previous_dir{};
            bool have_dir = false;
            auto came_it = came_from.find(visit_key(node.row, node.col));
            while (came_it != came_from.end()) {
                const Point parent = came_it->second;
                const Point direction{parent.row - node.row, parent.col - node.col};
                if (have_dir && (direction.row != previous_dir.row ||
                                 direction.col != previous_dir.col)) {
                    path.push_back(node);
                }
                previous_dir = direction;
                have_dir = true;
                node = parent;
                came_it = came_from.find(visit_key(node.row, node.col));
            }
            path.push_back(node);
            std::reverse(path.begin(), path.end());
            return path;
        }

        const auto current_key = visit_key(current.row, current.col);
        const double current_g = g_score[current_key];
        for (const auto& move : movements) {
            const int nr = current.row + move.dr;
            const int nc = current.col + move.dc;

            if (!grid.in_bounds(nr, nc) || !grid.is_free(nr, nc)) {
                continue;
            }

            if (std::abs(move.dr) == 1 && std::abs(move.dc) == 1) {
                if (!grid.is_free(current.row + move.dr, current.col) ||
                    !grid.is_free(current.row, current.col + move.dc)) {
                    continue;
                }
            }

            const double new_g = current_g + move.cost;
            const auto neighbor_key = visit_key(nr, nc);
            const auto old_g = g_score.find(neighbor_key);
            if (old_g == g_score.end() || new_g < old_g->second) {
                g_score[neighbor_key] = new_g;
                const double priority = new_g + std::abs(nr - goal.row) +
                                        std::abs(nc - goal.col);
                open_set.push(HeapNode{priority, nr, nc});
                came_from[neighbor_key] = current;
            }
        }
    }

    return {};
}

// A* 连接：搜索阶段只返回方向变化点；不连通时返回 nullopt。
static std::optional<std::vector<Point>> route_astar(const Grid& grid, Point a, Point b) {
    if (a.row == b.row && a.col == b.col) {
        return std::vector<Point>{};
    }
    const auto route = astar_search(grid, a, b);
    if (route.size() < 2) {
        return std::nullopt;
    }
    if (route.back().row != b.row || route.back().col != b.col) {
        return std::nullopt;
    }
    auto visible = [&](const Point& u, const Point& v) {
        const double dist = std::hypot(static_cast<double>(v.col - u.col),
                                       static_cast<double>(v.row - u.row));
        const int count = std::max(1, static_cast<int>(std::ceil(dist / 0.25)));
        for (int k = 0; k <= count; ++k) {
            const double t = static_cast<double>(k) / count;
            const int r = static_cast<int>(std::lround(u.row + (v.row - u.row) * t));
            const int c = static_cast<int>(std::lround(u.col + (v.col - u.col) * t));
            if (!grid.is_free(r, c)) return false;
        }
        return true;
    };
    std::vector<Point> pulled{route.front()};
    std::size_t anchor = 0;
    while (anchor + 1 < route.size()) {
        std::size_t farthest = anchor + 1;
        std::size_t probe = farthest + 1;
        while (probe < route.size() && visible(route[anchor], route[probe])) {
            farthest = probe++;
        }
        pulled.push_back(route[farthest]);
        anchor = farthest;
    }
    return std::vector<Point>(pulled.begin() + 1, pulled.end() - 1);
}

static std::vector<std::vector<Point>> get_scan_segments(
    const Grid& grid,
    const StageInput& stage,
    int fixed_idx,
    int start_idx,
    int end_idx,
    bool is_row_scan) {
    std::vector<std::vector<Point>> segments;
    std::vector<Point> current_segment;

    for (int moving_idx = start_idx; moving_idx <= end_idx; ++moving_idx+= 6) {
        const int r = is_row_scan ? fixed_idx : moving_idx;
        const int c = is_row_scan ? moving_idx : fixed_idx;

        if (!grid.in_bounds(r, c)) {
            if (!current_segment.empty()) {
                segments.push_back(current_segment);
                current_segment.clear();
            }
            continue;
        }

        const bool is_valid = grid.is_free(r, c) && mask_at(stage, grid, r, c);
        if (is_valid) {
            current_segment.push_back(Point{r, c});
        } else if (!current_segment.empty()) {
            segments.push_back(current_segment);
            current_segment.clear();
        }
    }

    if (!current_segment.empty()) {
        segments.push_back(current_segment);
    }
    return segments;
}

static std::vector<int> make_scan_lines(int start, int end, int path_spacing) {
    std::vector<int> scan_lines;
    if (start <= end) {
        int current = start;
        while (current <= end) {
            scan_lines.push_back(current);
            current += path_spacing;
        }
        if (!scan_lines.empty() && scan_lines.back() != end) {
            if (end - scan_lines.back() > path_spacing / 2.0) {
                scan_lines.push_back(end);
            }
        }
    } else {
        int current = start;
        while (current >= end) {
            scan_lines.push_back(current);
            current -= path_spacing;
        }
        if (!scan_lines.empty() && scan_lines.back() != end) {
            if (scan_lines.back() - end > path_spacing / 2.0) {
                scan_lines.push_back(end);
            }
        }
    }
    return scan_lines;
}

static std::vector<Point> plan_subregion_boustrophedon(
    const Grid& grid,
    const StageInput& stage,
    int path_spacing,
    Point start_point,
    Point end_point) {
    if (stage.boundary.empty()) {
        return {};
    }

    int min_row = stage.boundary.front().row;
    int max_row = stage.boundary.front().row;
    int min_col = stage.boundary.front().col;
    int max_col = stage.boundary.front().col;
    for (const auto& point : stage.boundary) {
        min_row = std::min(min_row, point.row);
        max_row = std::max(max_row, point.row);
        min_col = std::min(min_col, point.col);
        max_col = std::max(max_col, point.col);
    }
    min_row = std::max(0, min_row);
    max_row = std::min(grid.rows - 1, max_row);
    min_col = std::max(0, min_col);
    max_col = std::min(grid.cols - 1, max_col);

    std::vector<std::vector<Point>> all_segments;
    const bool is_x = direction_axis(stage.direction) == "x";

    if (is_x) {
        const auto scan_lines = make_scan_lines(start_point.row, end_point.row, path_spacing);
        for (std::size_t i = 0; i < scan_lines.size(); ++i) {
            int scan_start_col = min_col;
            int scan_end_col = max_col;
            const bool will_reverse = (i % 2 == 1);

            if (i == 0) {
                if (!will_reverse) {
                    scan_start_col = start_point.col;
                } else {
                    scan_end_col = start_point.col;
                }
            }

            if (i == scan_lines.size() - 1) {
                if (!will_reverse) {
                    scan_end_col = end_point.col;
                } else {
                    scan_start_col = end_point.col;
                }
            }

            auto segments = get_scan_segments(
                grid, stage, scan_lines[i], scan_start_col, scan_end_col, true);
            if (i % 2 == 1) {
                std::reverse(segments.begin(), segments.end());
                for (auto& segment : segments) {
                    std::reverse(segment.begin(), segment.end());
                }
            }
            all_segments.insert(all_segments.end(), segments.begin(), segments.end());
        }
    } else {
        const auto scan_lines = make_scan_lines(start_point.col, end_point.col, path_spacing);
        for (std::size_t i = 0; i < scan_lines.size(); ++i) {
            int scan_start_row = min_row;
            int scan_end_row = max_row;
            const bool will_reverse = (i % 2 == 1);

            if (i == 0) {
                if (!will_reverse) {
                    scan_start_row = start_point.row;
                } else {
                    scan_end_row = start_point.row;
                }
            }

            if (i == scan_lines.size() - 1) {
                if (!will_reverse) {
                    scan_end_row = end_point.row;
                } else {
                    scan_start_row = end_point.row;
                }
            }

            auto segments = get_scan_segments(
                grid, stage, scan_lines[i], scan_start_row, scan_end_row, false);
            if (i % 2 == 1) {
                std::reverse(segments.begin(), segments.end());
                for (auto& segment : segments) {
                    std::reverse(segment.begin(), segment.end());
                }
            }
            all_segments.insert(all_segments.end(), segments.begin(), segments.end());
        }
    }

    if (all_segments.empty()) {
        return {};
    }

    std::vector<Point> full_path = all_segments.front();
    for (std::size_t i = 1; i < all_segments.size(); ++i) {
        const auto& prev_seg = all_segments[i - 1];
        const auto& curr_seg = all_segments[i];
        const Point start_node = prev_seg.back();
        const Point end_node = curr_seg.front();
        const int dist = std::abs(start_node.row - end_node.row) +
                         std::abs(start_node.col - end_node.col);

        if (dist > 1.5) {
            const auto bridge_path = astar_search(grid, start_node, end_node);
            if (bridge_path.size() > 1) {
                full_path.insert(full_path.end(), bridge_path.begin() + 1, bridge_path.end() - 1);
            }
            full_path.insert(full_path.end(), curr_seg.begin(), curr_seg.end());
        } else if (!curr_seg.empty()) {
            full_path.insert(full_path.end(), curr_seg.begin(), curr_seg.end());
        }
    }

    std::vector<Point> filtered_path;
    for (const auto& point : full_path) {
        if (mask_at(stage, grid, point.row, point.col)) {
            filtered_path.push_back(point);
        }
    }

    if (!filtered_path.empty()) {
        const Point last_point = filtered_path.back();
        const int dist_to_end = std::abs(last_point.row - end_point.row) +
                                std::abs(last_point.col - end_point.col);
        if (dist_to_end > 1 && grid.is_free(end_point.row, end_point.col) &&
            mask_at(stage, grid, end_point.row, end_point.col)) {
            filtered_path.push_back(end_point);
        }
    }

    return filtered_path;
}

// 任意角度 / �������意方向弓字形规划（不旋转栅格）。
// 栅格仅作障碍查询表，扫描线在连续坐标下沿方向向量 d=(cos,sin) 切分。
// 路径点位以 double 保留完整浮点精度（100% 旋转精度，不向整数格吸附）；
// 仅障碍/边界的自由性判定和段间 A* 桥接在栅格精度上进行（受障碍图分辨率限制）。
static std::vector<FPoint> plan_subregion_oriented(
    const Grid& grid,
    const StageInput& stage,
    double angle_deg,
    int path_spacing,
    double step,
    Point start_point,
    Point end_point,
    double endpoint_margin = 0.0) {
    if (stage.boundary.empty()) {
        return {};
    }

    // RK3588 稀疏角点模式：5cm 栅格按 1 格逐格检测，避免生成半格冗余点。
    step = std::max(step, 1.0);

    const double kPi = std::acos(-1.0);
    const double theta = angle_deg * kPi / 180.0;
    const double dx = std::cos(theta);
    const double dy = std::sin(theta);
    const double nx = -std::sin(theta);
    const double ny = std::cos(theta);

    double d_min = std::numeric_limits<double>::max();
    double d_max = std::numeric_limits<double>::lowest();
    double n_min = std::numeric_limits<double>::max();
    double n_max = std::numeric_limits<double>::lowest();
    for (const auto& p : stage.boundary) {
        const double x = static_cast<double>(p.col);
        const double y = static_cast<double>(p.row);
        const double pd = x * dx + y * dy;
        const double pn = x * nx + y * ny;
        d_min = std::min(d_min, pd);
        d_max = std::max(d_max, pd);
        n_min = std::min(n_min, pn);
        n_max = std::max(n_max, pn);
    }

    auto is_free_xy = [&](double x, double y) -> bool {
        const int c = static_cast<int>(std::lround(x));
        const int r = static_cast<int>(std::lround(y));
        if (!grid.in_bounds(r, c)) return false;
        if (!grid.is_free(r, c)) return false;
        return mask_at(stage, grid, r, c);
    };
    auto inside_region_xy = [&](double x, double y) -> bool {
        const int c = static_cast<int>(std::lround(x));
        const int r = static_cast<int>(std::lround(y));
        return grid.in_bounds(r, c) && mask_at(stage, grid, r, c);
    };
    auto line_free_xy = [&](const FPoint& a, const FPoint& b) -> bool {
        const double dist = std::hypot(b.col - a.col, b.row - a.row);
        const int count = std::max(1, static_cast<int>(std::ceil(dist / 0.5)));
        for (int q = 0; q <= count; ++q) {
            const double u = static_cast<double>(q) / count;
            if (!is_free_xy(a.col + (b.col - a.col) * u,
                            a.row + (b.row - a.row) * u)) return false;
        }
        return true;
    };
    auto to_cell = [](const FPoint& p) -> Point {
        return Point{static_cast<int>(std::lround(p.row)),
                     static_cast<int>(std::lround(p.col))};
    };

    // 连续坐标下的定向扫描：点位保留完整浮点精度（100% 旋转精度）。
    std::vector<std::vector<FPoint>> all_segments;
    std::vector<int> segment_line_ids;
    std::vector<double> segment_base_s;
    auto append_oriented_segment = [&](const std::vector<FPoint>& seg,
                                       int line_id, double base_s) {
        if (seg.empty()) return;
        all_segments.push_back(seg);
        segment_line_ids.push_back(line_id);
        segment_base_s.push_back(base_s);
    };
    const double max_off = path_spacing * 4.5;   // 单侧横移上限(格)
    const double o_step = 1.0;                    // 横移搜索粒度(格)
    // 几何约束分离：掉头保持90°；柱子侧让角与额外躲避距离由调参项控制。
    const double corner_tangent = std::tan(
        stage.obstacle_corner_angle_deg * kPi / 180.0);
    const double slope = corner_tangent * step;

    // 某侧每个采样点所需最小横移量；-1 表示该侧无法清空
    auto req_for = [&](const std::vector<double>& ts, double s, double sgn) {
        std::vector<double> req(ts.size(), 0.0);
        for (std::size_t idx = 0; idx < ts.size(); ++idx) {
            const double bx = ts[idx] * dx + s * nx;
            const double by = ts[idx] * dy + s * ny;
            if (is_free_xy(bx, by)) { req[idx] = 0.0; continue; }
            double o = o_step; double hit = -1.0;
            while (o <= max_off) {
                if (is_free_xy(bx + sgn * o * nx, by + sgn * o * ny)) { hit = o; break; }
                o += o_step;
            }
            req[idx] = hit;
        }
        return req;
    };

    auto clamp_value = [](double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    };

    // 扫描范围必须覆盖区域完整法向宽度 [n_min, n_max]。
    // 起终点投影只选择扫描侧序，不能作为扫描范围端点；否则0°附近且
    // 起终点近似同高时会退化成仅一条扫描线。
    const double start_s_hint = clamp_value(
        start_point.col * nx + start_point.row * ny, n_min, n_max);
    const double end_s_hint = clamp_value(
        end_point.col * nx + end_point.row * ny, n_min, n_max);
    // 现场起点优先决定扫掠侧序，终点不得反向改变设备的起始侧。
    const bool start_near_min =
        std::abs(start_s_hint - n_min) <= std::abs(start_s_hint - n_max);
    const double start_edge = start_near_min ? n_min : n_max;
    const double end_edge = start_near_min ? n_max : n_min;
    const double edge_dir = end_edge >= start_edge ? 1.0 : -1.0;
    const double normal_inset = std::min(0.5 * static_cast<double>(path_spacing),
                                         0.5 * (n_max - n_min));
    // C++阶段始终带有明确起终点：首末作业线严格通过现场选点；
    // 首末间隔允许自适应，中间线仍保持固定间距。
    double start_s = start_s_hint;
    double end_s = end_s_hint;
    const double start_t = start_point.col * dx + start_point.row * dy;
    const double end_t = end_point.col * dx + end_point.row * dy;
    bool first_ascending = direction_travel_sign(stage.direction) > 0;
    // 只有当当前点沿指定方向已无作业空间时才转场；只是更靠近
    // 某一侧时仍从当前点继续，不让就近策略把指定方向反转。
    bool reposition_to_forced_start = false;
    bool wanted_last_ascending = std::abs(end_t - d_max) <= std::abs(end_t - d_min);
    std::vector<double> scan_s;
    const double span = std::abs(end_s - start_s);
    if (span <= 1e-9) {
        wanted_last_ascending = first_ascending;
        scan_s.push_back(start_s);
    } else {
        const bool need_even = first_ascending == wanted_last_ascending;
        const int nominal = std::max(1, static_cast<int>(std::lround(span / path_spacing)));
        int intervals = -1;
        double best_error = std::numeric_limits<double>::infinity();
        for (int n = std::max(1, nominal - 2); n <= nominal + 2; ++n) {
            if ((n % 2 == 0) != need_even) continue;
            const double error = std::abs(span / n - path_spacing);
            if (error < best_error) { best_error = error; intervals = n; }
        }
        if (intervals < 1) intervals = nominal;
        for (int i = 0; i <= intervals; ++i)
            scan_s.push_back(start_s + (end_s - start_s) * i / intervals);
    }
    // 不再为了终点朝向插入/删除扫描线，末线直接使用终点要求方向。

    for (std::size_t k = 0; k < scan_s.size(); ++k) {
        const double s = scan_s[k];
        bool ascending = (k % 2 == 0) ? first_ascending : !first_ascending;
        // 先生成完整边界截线，留边后再根据显式起终点进一步裁剪。
        double t_lo = d_min;
        double t_hi = d_max;

        std::vector<double> ts;
        for (double t = t_lo; t < t_hi - 1e-9; t += step) {
            ts.push_back(t);
        }
        ts.push_back(t_hi);
        // 先按真实区域裁剪。区域外不作为障碍参与侧让，避免边界尖角/长回接。
        ts.erase(std::remove_if(ts.begin(), ts.end(), [&](double t) {
            return !inside_region_xy(t * dx + s * nx, t * dy + s * ny);
        }), ts.end());
        const double margin = std::max(0.0, endpoint_margin);
        if (!ts.empty() && margin > 1e-9) {
            const auto mm = std::minmax_element(ts.begin(), ts.end());
            const double inner_lo = *mm.first + margin;
            const double inner_hi = *mm.second - margin;
            if (inner_lo >= inner_hi - 1e-9) continue;
            std::vector<double> trimmed;
            trimmed.reserve(ts.size() + 2);
            trimmed.push_back(inner_lo);
            for (double t : ts)
                if (t > inner_lo && t < inner_hi) trimmed.push_back(t);
            trimmed.push_back(inner_hi);
            ts.swap(trimmed);
        }
        if (k == 0 && !ts.empty()) {
            const auto mm = std::minmax_element(ts.begin(), ts.end());
            const double forward_span = first_ascending
                ? *mm.second - start_t : start_t - *mm.first;
            reposition_to_forced_start =
                forward_span <= 0.5 * std::max(step, 1.0);
        }
        if (k == 0 && !reposition_to_forced_start) {
            ts.erase(std::remove_if(ts.begin(), ts.end(), [&](double t) {
                return ascending ? t < start_t - 1e-9 : t > start_t + 1e-9;
            }), ts.end());
        }
        if (k + 1 == scan_s.size()) {
            ts.erase(std::remove_if(ts.begin(), ts.end(), [&](double t) {
                return ascending ? t > end_t + 1e-9 : t < end_t - 1e-9;
            }), ts.end());
        }
        // 连续边界内、但取整后落到边界外障碍格的首尾点，沿扫掠方向
        // 向内裁剪；禁止把边界量化误差当成柱子做法向侧移。
        auto base_free_t = [&](double t) {
            return is_free_xy(t * dx + s * nx, t * dy + s * ny);
        };
        while (!ts.empty() && !base_free_t(ts.front())) ts.erase(ts.begin());
        while (!ts.empty() && !base_free_t(ts.back())) ts.pop_back();
        if (ts.empty()) continue;
        if (!ascending) {
            std::reverse(ts.begin(), ts.end());
        }

        // Compute both sides once, then keep one side for the complete lane.
        const std::vector<double> req_p = req_for(ts, s, 1.0);
        const std::vector<double> req_m = req_for(ts, s, -1.0);
        const std::size_t n = ts.size();
        std::vector<double> signed_offset(n, 0.0);
        std::vector<std::uint8_t> impossible(n, 0);

        auto is_obstacle_sample = [&](std::size_t index) {
            return req_p[index] < 0.0 || req_m[index] < 0.0 ||
                   req_p[index] > 1e-9 || req_m[index] > 1e-9;
        };
        auto set_profile = [&](std::size_t index, double value) {
            if (std::abs(value) > std::abs(signed_offset[index])) {
                signed_offset[index] = value;
            }
        };

        // Pick one side for the complete nominal lane. Upper lanes remain
        // upper at every pillar and lower lanes remain lower; a lane never
        // switches side between obstacles.
        int invalid_p = 0;
        int invalid_m = 0;
        double cost_p = 0.0;
        double cost_m = 0.0;
        for (std::size_t x = 0; x < n; ++x) {
            if (!is_obstacle_sample(x)) continue;
            if (req_p[x] < 0.0) ++invalid_p;
            else cost_p += req_p[x];
            if (req_m[x] < 0.0) ++invalid_m;
            else cost_m += req_m[x];
        }
        double lane_side = 1.0;
        if (invalid_m < invalid_p ||
            (invalid_m == invalid_p && cost_m + 0.5 < cost_p)) {
            lane_side = -1.0;
        } else if (invalid_m == invalid_p &&
                   std::abs(cost_p - cost_m) <= 0.5) {
            lane_side = (k % 2 == 0) ? -1.0 : 1.0;
        }

        std::size_t block_index = 0;
        while (block_index < n) {
            if (!is_obstacle_sample(block_index)) {
                ++block_index;
                continue;
            }

            const std::size_t lo = block_index;
            std::size_t last_hit = block_index;
            while (block_index + 1 < n) {
                ++block_index;
                if (is_obstacle_sample(block_index)) {
                    last_hit = block_index;
                } else if (block_index - last_hit > 2) {
                    break;
                }
            }
            const std::size_t hi = last_hit;

            bool block_valid = true;
            double peak = 0.0;
            for (std::size_t x = lo; x <= hi; ++x) {
                const double required =
                    lane_side > 0.0 ? req_p[x] : req_m[x];
                if (required < 0.0) block_valid = false;
                else peak = std::max(peak, required);
            }

            if (!block_valid) {
                for (std::size_t x = lo; x <= hi; ++x) impossible[x] = 1;
                block_index = std::max(block_index, hi + 1);
                continue;
            }

            // Grinding coverage has priority over separating detour lanes.
            // Every blocked nominal lane uses only its minimum safe offset;
            // overlapping detours are allowed and no 0.7 m baseline is removed.
            peak += stage.obstacle_avoidance_pixels;
            const std::size_t ramp_n = static_cast<std::size_t>(std::max(
                1, static_cast<int>(std::ceil(
                    peak / std::max(slope, 1e-9)))));
            // Do not extend the detour plateau before/after a pillar. The old
            // four-sample pad moved adjacent 0.7 m lanes away about 1.2 m too
            // early, leaving an unground strip in front of and behind pillars.
            const std::size_t left_pad = 0;
            const std::size_t right_pad = 0;
            const std::size_t plateau_lo = lo - left_pad;
            const std::size_t plateau_hi = hi + right_pad;
            const std::size_t left =
                plateau_lo > ramp_n ? plateau_lo - ramp_n : 0;
            const std::size_t right =
                std::min(n - 1, plateau_hi + ramp_n);

            for (std::size_t x = left; x < plateau_lo; ++x) {
                const double profile = std::max(
                    0.0, peak - (plateau_lo - x) * slope);
                set_profile(x, lane_side * profile);
            }
            for (std::size_t x = plateau_lo; x <= plateau_hi; ++x) {
                set_profile(x, lane_side * peak);
            }
            for (std::size_t x = plateau_hi + 1; x <= right; ++x) {
                const double profile = std::max(
                    0.0, peak - (x - plateau_hi) * slope);
                set_profile(x, lane_side * profile);
            }
            block_index = std::max(block_index, hi + 1);
        }

        for (std::size_t x = 0; x < n; ++x) {
            const double envelope =
                static_cast<double>(std::min(x, n - 1 - x)) * slope;
            signed_offset[x] = std::copysign(
                std::min(std::abs(signed_offset[x]), envelope),
                signed_offset[x]);
        }

        std::vector<FPoint> cur;
        FPoint raw_last{};
        FPoint raw_dir{};
        bool have_raw = false;
        bool have_dir = false;
        auto emit_raw_corner = [&](double py, double px) {
            const FPoint current{py, px};
            if (!have_raw) {
                cur.push_back(current);
                raw_last = current;
                have_raw = true;
                have_dir = false;
                return;
            }
            const FPoint v{current.row - raw_last.row,
                           current.col - raw_last.col};
            if (!have_dir) {
                raw_dir = v;
                have_dir = true;
            } else {
                const double cross =
                    raw_dir.col * v.row - raw_dir.row * v.col;
                const double scale = std::max(
                    1.0, std::hypot(raw_dir.row, raw_dir.col) *
                             std::hypot(v.row, v.col));
                if (std::abs(cross) > 1e-7 * scale ||
                    raw_dir.row * v.row + raw_dir.col * v.col < 0.0) {
                    cur.push_back(raw_last);
                    raw_dir = v;
                }
            }
            raw_last = current;
        };
        auto flush_raw_segment = [&]() {
            if (have_raw && (cur.empty() ||
                std::abs(cur.back().row - raw_last.row) > 1e-9 ||
                std::abs(cur.back().col - raw_last.col) > 1e-9)) {
                cur.push_back(raw_last);
            }
            if (!cur.empty()) {
                append_oriented_segment(cur, static_cast<int>(k), s);
            }
            cur.clear();
            have_raw = false;
            have_dir = false;
        };

        for (std::size_t xi = 0; xi < n; ++xi) {
            if (impossible[xi]) {
                flush_raw_segment();
                continue;
            }

            const double bx = ts[xi] * dx + s * nx;
            const double by = ts[xi] * dy + s * ny;
            double oo = signed_offset[xi];
            bool ok = is_free_xy(bx + oo * nx, by + oo * ny);
            int guard = 0;
            while (!ok && std::abs(oo) <= max_off && guard < 400) {
                double side = oo < 0.0 ? -1.0 : 1.0;
                if (std::abs(oo) <= 1e-9) {
                    const bool can_p = req_p[xi] >= 0.0;
                    const bool can_m = req_m[xi] >= 0.0;
                    if (!can_p && !can_m) break;
                    if (!can_p || (can_m && req_m[xi] < req_p[xi])) {
                        side = -1.0;
                        oo = -std::max(o_step, req_m[xi]);
                    } else {
                        side = 1.0;
                        oo = std::max(o_step, req_p[xi]);
                    }
                } else {
                    oo += side * o_step;
                }
                ok = is_free_xy(bx + oo * nx, by + oo * ny);
                ++guard;
            }
            if (!ok) {
                flush_raw_segment();
                continue;
            }

            const double px = bx + oo * nx;
            const double py = by + oo * ny;
            if (have_raw &&
                (std::abs(py - raw_last.row) +
                 std::abs(px - raw_last.col)) >
                    1.1 * (1.0 + corner_tangent) *
                    std::max(step, 1.0)) {
                flush_raw_segment();
            }
            emit_raw_corner(py, px);
        }
        flush_raw_segment();
    }

    if (all_segments.empty()) {
        return {};
    }

    // Coverage is mandatory: every configured nominal scan lane must
    // contribute at least one segment. Never return a seemingly valid path
    // after an obstacle caused an intermediate lane to disappear.
    std::vector<std::uint8_t> seen_scan_lines(scan_s.size(), 0);
    for (const int line_id : segment_line_ids) {
        if (line_id < 0 || line_id >= static_cast<int>(scan_s.size())) return {};
        seen_scan_lines[static_cast<std::size_t>(line_id)] = 1;
    }
    if (std::find(seen_scan_lines.begin(), seen_scan_lines.end(), 0) !=
        seen_scan_lines.end()) return {};

    // 现场选点必须是首/末作业线真实端点，不能仅由A*在事后接入。
    // 检查连接开区间，允许选点本身恰落在连续多边形边界上。
    auto strict_endpoint_leg_free = [&](const FPoint& a, const FPoint& b,
                                        bool exclude_b) {
        const double dist = std::hypot(b.col - a.col, b.row - a.row);
        const int count = std::max(1, static_cast<int>(std::ceil(dist / 0.5)));
        const int last_q = exclude_b ? count - 1 : count;
        for (int q = 1; q <= last_q; ++q) {
            const double u = static_cast<double>(q) / count;
            if (!is_free_xy(a.col + (b.col - a.col) * u,
                            a.row + (b.row - a.row) * u)) return false;
        }
        return true;
    };
    if (segment_line_ids.front() != 0) return {};
    const FPoint exact_start{static_cast<double>(start_point.row),
                             static_cast<double>(start_point.col)};
    if (!reposition_to_forced_start) {
        if (!strict_endpoint_leg_free(exact_start, all_segments.front().front(), false))
            return {};
        if (std::hypot(all_segments.front().front().row - exact_start.row,
                       all_segments.front().front().col - exact_start.col) <= 1e-7)
            all_segments.front().front() = exact_start;
        else
            all_segments.front().insert(all_segments.front().begin(), exact_start);
    }

    const int last_line_id = static_cast<int>(scan_s.size()) - 1;
    if (segment_line_ids.back() != last_line_id) return {};
    const FPoint exact_end{static_cast<double>(end_point.row),
                           static_cast<double>(end_point.col)};
    if (!strict_endpoint_leg_free(all_segments.back().back(), exact_end, true))
        return {};
    if (std::hypot(all_segments.back().back().row - exact_end.row,
                   all_segments.back().back().col - exact_end.col) <= 1e-7)
        all_segments.back().back() = exact_end;
    else
        all_segments.back().push_back(exact_end);

    // 首末段已在同一旋转扫描线上严格锚定，不会产生跨线尖刺。

    auto bridge_line_free = [&](const FPoint& a, const FPoint& b) {
        return line_free_xy(a, b);
    };
    auto append_path_corner = [&](std::vector<FPoint>& dst, const FPoint& p) {
        // 保留旋转坐标系中的90°角，禁止直连把工字形拉成斜线。
        append_corner(dst, p);
    };
    auto append_connection_corner = [&](std::vector<FPoint>& dst, const FPoint& p) -> bool {
        // A*/跨段连接只走d/n轴或配置折角，禁止任意斜率。
        if (!dst.empty()) {
            const FPoint a = dst.back();
            const double at = a.col * dx + a.row * dy;
            const double as = a.col * nx + a.row * ny;
            const double bt = p.col * dx + p.row * dy;
            const double bs = p.col * nx + p.row * ny;
            if (std::abs(bt - at) > 1e-6 && std::abs(bs - as) > 1e-6) {
                bool has_safe_corner = false;
                const FPoint candidates[2] = {
                    FPoint{bt * dy + as * ny, bt * dx + as * nx},
                    FPoint{at * dy + bs * ny, at * dx + bs * nx}};
                for (const auto& corner : candidates) {
                    if (bridge_line_free(a, corner) && bridge_line_free(corner, p)) {
                        append_corner(dst, corner);
                        has_safe_corner = true;
                        break;
                    }
                }
                // 直角狗腿受阻时使用配置折角与轴向段组合。
                if (!has_safe_corner) {
                    const double dt = bt - at, ds = bs - as;
                    const double m_t = std::min(
                        std::abs(dt), std::abs(ds) / corner_tangent);
                    const double m_s = corner_tangent * m_t;
                    const double st = dt >= 0.0 ? 1.0 : -1.0;
                    const double ss = ds >= 0.0 ? 1.0 : -1.0;
                    const FPoint hybrid[2] = {
                        FPoint{(at + st*m_t)*dy + (as + ss*m_s)*ny,
                               (at + st*m_t)*dx + (as + ss*m_s)*nx},
                        FPoint{(bt - st*m_t)*dy + (bs - ss*m_s)*ny,
                               (bt - st*m_t)*dx + (bs - ss*m_s)*nx}};
                    for (const auto& corner : hybrid) {
                        if (bridge_line_free(a, corner) && bridge_line_free(corner, p)) {
                            append_corner(dst, corner);
                            has_safe_corner = true;
                            break;
                        }
                    }
                }
                if (!has_safe_corner) return false;
            }
        }
        append_corner(dst, p);
        return true;
    };
    auto append_connection_with_backtrack =
        [&](std::vector<FPoint>& dst, const FPoint& p,
            std::size_t min_index) -> bool {
            if (dst.empty()) return false;
            const std::size_t floor = std::min(min_index, dst.size() - 1);
            for (std::ptrdiff_t tail =
                     static_cast<std::ptrdiff_t>(dst.size()) - 1;
                 tail >= static_cast<std::ptrdiff_t>(floor); --tail) {
                std::vector<FPoint> candidate(
                    dst.begin(), dst.begin() + tail + 1);
                if (append_connection_corner(candidate, p)) {
                    dst.swap(candidate);
                    return true;
                }
            }
            return false;
        };
    auto append_path_corners = [&](std::vector<FPoint>& dst, auto first, auto last) {
        for (; first != last; ++first) append_path_corner(dst, *first);
    };

    std::vector<FPoint> full_path;
    const FPoint first = all_segments.front().front();
    const Point first_cell = to_cell(first);
    append_path_corner(full_path, FPoint{static_cast<double>(start_point.row),
                                         static_cast<double>(start_point.col)});
    if (std::abs(start_point.row - first_cell.row) +
            std::abs(start_point.col - first_cell.col) > 1) {
        const auto mid = route_astar(grid, start_point, first_cell);
        if (!mid.has_value()) return {};
        for (const auto& p : *mid) {
            append_connection_corner(full_path, FPoint{static_cast<double>(p.row),
                                                        static_cast<double>(p.col)});
        }
    }
    append_connection_corner(full_path, all_segments.front().front());
    append_path_corners(full_path, all_segments.front().begin() + 1,
                        all_segments.front().end());

    const double bridge_gap = 1.5 * std::max(step, 1.0);
    std::size_t last_added_idx = 0;
    // full_path 已压缩共线点，禁止用原始段点数反推段首（会发生无符号下溢）。
    std::size_t last_segment_start = 0;

    for (std::size_t i = 1; i < all_segments.size(); ++i) {
        const auto& seg = all_segments[i];
        const FPoint a = full_path.back();
        const FPoint b = seg.front();
        const int prev_line = segment_line_ids[last_added_idx];
        const int curr_line = segment_line_ids[i];
        const double gap = std::abs(a.row - b.row) + std::abs(a.col - b.col);
        if (gap <= bridge_gap) {
            last_segment_start = full_path.size();
            append_connection_corner(full_path, seg.front());
            append_path_corners(full_path, seg.begin() + 1, seg.end());
            last_added_idx = i;
            continue;
        }

        bool connected = false;
        if (curr_line == prev_line + 1 &&
            std::abs(segment_base_s[i] - segment_base_s[last_added_idx]) <=
                1.5 * path_spacing) {
            // Preserve both complete grinding lanes. The turn may retrace a
            // short covered section, but must never resize the previous lane
            // or skip the head of the next lane.
            auto point_t = [&](const FPoint& p) {
                return p.col * dx + p.row * dy;
            };
            auto point_s = [&](const FPoint& p) {
                return p.col * nx + p.row * ny;
            };
            const double a_t = point_t(a);
            const double b_t = point_t(b);
            const double a_s = point_s(a);
            const double b_s = point_s(b);
            const bool side_max = std::abs(a_t - d_max) <= std::abs(a_t - d_min);
            const double turn_cap = side_max ? std::min(a_t, b_t)
                                             : std::max(a_t, b_t);
            const int max_inset = std::max(2, std::min(24,
                static_cast<int>(std::lround(0.8 * path_spacing))));

            for (int inset_i = 2; inset_i <= max_inset; ++inset_i) {
                const double inset = static_cast<double>(inset_i);
                const double turn_t = side_max ? turn_cap - inset : turn_cap + inset;
                const FPoint turn_a{turn_t * dy + a_s * ny,
                                    turn_t * dx + a_s * nx};
                const FPoint turn_b{turn_t * dy + b_s * ny,
                                    turn_t * dx + b_s * nx};

                if (bridge_line_free(a, turn_a) &&
                    bridge_line_free(turn_a, turn_b) &&
                    bridge_line_free(turn_b, b)) {
                    append_path_corner(full_path, turn_a);
                    append_path_corner(full_path, turn_b);
                    last_segment_start = full_path.size();
                    append_path_corners(full_path, seg.begin(), seg.end());
                    last_added_idx = i;
                    connected = true;
                    break;
                }
            }
        }

        if (connected) continue;

        // 同 line_id 为线内断段；非相邻 line_id 也禁止猜测掉头，统一安全 A*。
        const auto mid = route_astar(grid, to_cell(a), to_cell(b));
        if (!mid.has_value()) {
            return {};  // Never backtrack-delete a grinding scan segment.
        }
        for (const auto& p_mid : *mid) {
            append_connection_corner(
                full_path,
                FPoint{static_cast<double>(p_mid.row),
                       static_cast<double>(p_mid.col)});
        }
        last_segment_start = full_path.size();
        if ((std::hypot(full_path.back().row - seg.front().row,
                        full_path.back().col - seg.front().col) > 1e-7) &&
            !append_connection_corner(full_path, seg.front()))
            append_corner(full_path, seg.front());  // 最终几何校验负责重新分解
        append_path_corners(full_path, seg.begin() + 1, seg.end());
        last_added_idx = i;
    }

    if (!full_path.empty()) {
        const FPoint last = full_path.back();
        if (std::abs(last.row - static_cast<double>(end_point.row)) +
                std::abs(last.col - static_cast<double>(end_point.col)) > 1.0 &&
            grid.is_free(end_point.row, end_point.col) &&
            mask_at(stage, grid, end_point.row, end_point.col)) {
            const auto mid = route_astar(grid, to_cell(last), end_point);
            if (mid.has_value()) {  // 连通才走到终点，否则不追加
                for (const auto& p : *mid) {
                    append_connection_corner(full_path, FPoint{static_cast<double>(p.row),
                                                                static_cast<double>(p.col)});
                }
                append_connection_corner(full_path,
                    FPoint{static_cast<double>(end_point.row),
                           static_cast<double>(end_point.col)});
            }
        }
    }

    // 安全局部平滑：直线不变，只消除掉头/A* 的锯齿和避障折角；
    // 候选点与相邻短线段全部重新检查，失败即保留原点。
    const int radius = std::max(4, std::min(24, static_cast<int>(std::lround(
        0.25 * path_spacing / std::max(step, 0.5)))));
    // RK3588 稀疏角点模式：不执行密集全路径平滑，直接安全抽稀。
    for (int pass = 0; pass < 0 && full_path.size() >= 5; ++pass) {
        const std::vector<FPoint> old = full_path;
        std::vector<FPoint> smoothed;
        smoothed.reserve(old.size());
        smoothed.push_back(old.front());
        for (std::size_t idx = 1; idx + 1 < old.size(); ++idx) {
            // 起终点邻域保持原短接入/退出段，避免固定端点与平滑点错位。
            if (idx <= static_cast<std::size_t>(radius) ||
                idx >= old.size() - 1 - static_cast<std::size_t>(radius)) {
                smoothed.push_back(old[idx]);
                continue;
            }
            const std::size_t lo = idx > static_cast<std::size_t>(radius)
                ? idx - static_cast<std::size_t>(radius) : 0;
            const std::size_t hi = std::min(old.size() - 1,
                idx + static_cast<std::size_t>(radius));
            double sw = 0.0, sr = 0.0, sc = 0.0;
            for (std::size_t q = lo; q <= hi; ++q) {
                const double w = radius + 1.0 - std::abs(
                    static_cast<double>(q) - static_cast<double>(idx));
                sw += w; sr += w * old[q].row; sc += w * old[q].col;
            }
            const FPoint cand{sr / sw, sc / sw};
            if (is_free_xy(cand.col, cand.row) &&
                line_free_xy(smoothed.back(), cand) &&
                line_free_xy(cand, old[idx + 1])) {
                smoothed.push_back(cand);
            } else {
                smoothed.push_back(old[idx]);
            }
        }
        smoothed.push_back(old.back());
        full_path.swap(smoothed);
    }

    // 二次安全去毛刺：消除接近轴向角度时 A* 整数格回退形成的
    // 1~3 格短凸点，不拉直正常 U 形掉头。
    const std::size_t endpoint_guard = static_cast<std::size_t>(std::max(
        6, static_cast<int>(std::lround(
            0.25 * path_spacing / std::max(step, 0.5)))));
    for (int pass = 0; pass < 0 && full_path.size() > 2 * endpoint_guard; ++pass) {
        const std::vector<FPoint> old = full_path;
        std::vector<FPoint> cleaned = old;
        for (std::size_t idx = endpoint_guard;
             idx + endpoint_guard < old.size(); ++idx) {
            const FPoint cand{
                0.5 * (old[idx - 1].row + old[idx + 1].row),
                0.5 * (old[idx - 1].col + old[idx + 1].col)};
            const double deviation = std::hypot(cand.row - old[idx].row,
                                                cand.col - old[idx].col);
            if (!(deviation > 0.15 && deviation <= 3.0)) continue;
            if (is_free_xy(cand.col, cand.row) &&
                line_free_xy(old[idx - 1], cand) &&
                line_free_xy(cand, old[idx + 1])) {
                cleaned[idx] = cand;
            }
        }
        full_path.swap(cleaned);
    }

    // 输出级几何校验：仅保留d轴、n轴和配置柱子折角。
    std::vector<FPoint> normalized;
    normalized.reserve(full_path.size());
    for (const auto& point : full_path) {
        if (normalized.empty()) { normalized.push_back(point); continue; }
        const FPoint a = normalized.back();
        const double vx = point.col - a.col, vy = point.row - a.row;
        const double dt = vx * dx + vy * dy;
        const double ds = vx * nx + vy * ny;
        const bool is_obstacle_angle =
            std::abs(std::abs(ds) - corner_tangent * std::abs(dt)) <= 1e-5;
        const double at = a.col * dx + a.row * dy;
        const double bt = point.col * dx + point.row * dy;
        const double boundary_band = std::max(3.0, 0.75 * path_spacing);
        const bool boundary_obstacle_angle = is_obstacle_angle &&
            ((std::abs(at - d_min) <= boundary_band &&
              std::abs(bt - d_min) <= boundary_band) ||
             (std::abs(at - d_max) <= boundary_band &&
              std::abs(bt - d_max) <= boundary_band));
        if (boundary_obstacle_angle) {
            const double as = a.col * nx + a.row * ny;
            const double bs = point.col * nx + point.row * ny;
            const FPoint corners[2] = {
                FPoint{bt * dy + as * ny, bt * dx + as * nx},
                FPoint{at * dy + bs * ny, at * dx + bs * nx}};
            bool folded = false;
            for (const auto& corner : corners) {
                if (bridge_line_free(a, corner) && bridge_line_free(corner, point)) {
                    append_corner(normalized, corner);
                    append_corner(normalized, point);
                    folded = true;
                    break;
                }
            }
            if (!folded) append_corner(normalized, point);
        } else if (std::abs(ds) <= 1e-5 || std::abs(dt) <= 1e-5 ||
                   is_obstacle_angle) {
            append_corner(normalized, point);
        } else {
            // Angle normalization is optional; grinding coverage is not.
            // If a safe generated corner cannot be folded into the preferred
            // axes/angle, retain it instead of silently deleting the point.
            if (!append_connection_corner(normalized, point))
                append_corner(normalized, point);
        }
    }
    // 删除首末现场选点外侧的量化残段，严格禁止向选点之外扩张。
    const double endpoint_tol = 0.20;
    if (!normalized.empty()) {
        if (!reposition_to_forced_start) {
            std::vector<FPoint> clipped;
            clipped.push_back(normalized.front());
            std::size_t idx = 1;
            while (idx < normalized.size()) {
                const double ps = normalized[idx].col * nx + normalized[idx].row * ny;
                if (std::abs(ps - start_s) > endpoint_tol) break;
                const double pt = normalized[idx].col * dx + normalized[idx].row * dy;
                if ((first_ascending && pt >= start_t - 1e-6) ||
                    (!first_ascending && pt <= start_t + 1e-6))
                    append_corner(clipped, normalized[idx]);
                ++idx;
            }
            for (; idx < normalized.size(); ++idx) append_corner(clipped, normalized[idx]);
            normalized.swap(clipped);
        }

        for (std::ptrdiff_t i = static_cast<std::ptrdiff_t>(normalized.size()) - 2;
             i >= 0; --i) {
            const double ps = normalized[static_cast<std::size_t>(i)].col * nx +
                              normalized[static_cast<std::size_t>(i)].row * ny;
            if (std::abs(ps - end_s) > endpoint_tol) break;
            const double pt = normalized[static_cast<std::size_t>(i)].col * dx +
                              normalized[static_cast<std::size_t>(i)].row * dy;
            const bool valid = (wanted_last_ascending && pt <= end_t + 1e-6) ||
                               (!wanted_last_ascending && pt >= end_t - 1e-6);
            if (!valid) normalized.erase(normalized.begin() + i);
        }
        const FPoint last = normalized.back();
        if (std::hypot(last.row - exact_end.row,
                       last.col - exact_end.col) <= 1e-7) {
            normalized.back() = exact_end;
        } else {
            if (!append_connection_corner(normalized, exact_end))
                return {};
        }
    }
    return normalized;
}

static std::vector<PathItem> plan_all_stages(
    const Grid& grid,
    const std::vector<StageInput>& stages,
    int spacing_pixels) {
    std::vector<PathItem> complete_path;
    std::optional<Point> last_stage_end;

    for (const auto& stage : stages) {
        const Grid& active_grid = stage.planning_grid.has_value()
                                      ? *stage.planning_grid : grid;
        Point curr_start;
        if (stage.start_point.has_value()) {
            // 现场选点不可被膨胀栅格的“最近空闲点”静默挪动。
            curr_start = *stage.start_point;
        } else {
            curr_start = find_nearest_free_point(
                active_grid, find_farthest_corner(stage.boundary, stage.end_point));
        }

        if (last_stage_end.has_value()) {
            const int dist = std::abs(last_stage_end->row - curr_start.row) +
                             std::abs(last_stage_end->col - curr_start.col);
            if (dist > 2) {
                const auto conn_pts = route_astar(active_grid, *last_stage_end, curr_start);
                if (conn_pts.has_value()) {
                    for (const auto& p : *conn_pts) {
                        complete_path.push_back(PathItem{
                            static_cast<double>(p.row),
                            static_cast<double>(p.col), "connection"});
                    }
                }
            }
        }

        const std::string path_type = "stage_" + std::to_string(stage.stage_id);
        if (stage.direction_angle.has_value()) {
            const auto region_pts = plan_subregion_oriented(
                active_grid, stage, *stage.direction_angle, spacing_pixels, 0.5,
                curr_start, stage.end_point, stage.endpoint_margin_pixels);
            if (!region_pts.empty()) {
                for (const auto& p : region_pts) {
                    complete_path.push_back(PathItem{p.row, p.col, path_type});
                }
                const auto& back = region_pts.back();
                last_stage_end = Point{static_cast<int>(std::lround(back.row)),
                                       static_cast<int>(std::lround(back.col))};
            } else {
                last_stage_end = curr_start;
            }
        } else {
            // 轴向规划统一使用角点直出内核：x=0°，y=90°。
            const double axis_angle =
                direction_axis(stage.direction) == "x" ? 0.0 : 90.0;
            const auto region_pts = plan_subregion_oriented(
                active_grid, stage, axis_angle, spacing_pixels, 1.0,
                curr_start, stage.end_point, stage.endpoint_margin_pixels);
            if (!region_pts.empty()) {
                for (const auto& point : region_pts) {
                    complete_path.push_back(PathItem{
                        point.row, point.col, path_type});
                }
                const auto& back = region_pts.back();
                last_stage_end = Point{static_cast<int>(std::lround(back.row)),
                                       static_cast<int>(std::lround(back.col))};
            } else {
                last_stage_end = curr_start;
            }
        }
    }

    return complete_path;
}

// 角点规划完成后按固定栅格距离补点；原始角点全部保留。
static std::vector<PathItem> densify_path(
    const std::vector<PathItem>& src, double spacing_pixels) {
    if (src.size() < 2 || spacing_pixels <= 1e-9) return src;
    // Preserve intentional overlap and return-to-baseline obstacle detours.
    // A former A-B-A cleanup removed up to twelve valid grinding corners and
    // silently created unground strips beside pillars. Only densify here.
    const std::vector<PathItem>& corners = src;
    std::vector<PathItem> out;
    out.reserve(corners.size());
    out.push_back(corners.front());
    for (std::size_t i = 1; i < corners.size(); ++i) {
        const PathItem& a = corners[i - 1];
        const PathItem& b = corners[i];
        const double dr = b.row - a.row, dc = b.col - a.col;
        const double dist = std::hypot(dr, dc);
        if (dist <= 1e-9) continue;
        for (double d = spacing_pixels; d < dist - 1e-9; d += spacing_pixels) {
            const double u = d / dist;
            out.push_back(PathItem{a.row + u * dr, a.col + u * dc,
                                   a.path_type.empty() ? b.path_type : a.path_type});
        }
        out.push_back(b);
    }
    return out;
}

#if MST27_HAVE_PYBIND
static Grid parse_grid(const py::array_t<int, py::array::c_style | py::array::forcecast>& grid_array) {
    const auto info = grid_array.request();
    if (info.ndim != 2) {
        throw std::runtime_error("grid_map must be a 2D numpy array");
    }

    Grid grid;
    grid.rows = static_cast<int>(info.shape[0]);
    grid.cols = static_cast<int>(info.shape[1]);
    grid.blocked.resize(static_cast<std::size_t>(grid.rows) * static_cast<std::size_t>(grid.cols));

    const auto* ptr = static_cast<const int*>(info.ptr);
    for (std::size_t i = 0; i < grid.blocked.size(); ++i) {
        grid.blocked[i] = ptr[i] != 0 ? 1 : 0;
    }
    return grid;
}

static std::vector<StageInput> parse_stages(const py::list& stage_specs, const Grid& grid) {
    std::vector<StageInput> stages;
    stages.reserve(stage_specs.size());

    for (const auto& item : stage_specs) {
        auto spec = py::cast<py::dict>(item);
        StageInput stage;
        stage.stage_id = spec["stage_id"].cast<int>();
        stage.direction = spec["direction"].cast<std::string>();
        if (spec.contains("direction_angle")) {
            py::handle da = spec["direction_angle"];
            if (!da.is_none()) {
                stage.direction_angle = da.cast<double>();
            }
        }
        if (spec.contains("endpoint_margin_pixels")) {
            stage.endpoint_margin_pixels = std::max(
                0.0, spec["endpoint_margin_pixels"].cast<double>());
        }
        if (spec.contains("obstacle_corner_angle_deg")) {
            stage.obstacle_corner_angle_deg =
                spec["obstacle_corner_angle_deg"].cast<double>();
        }
        if (spec.contains("obstacle_avoidance_pixels")) {
            stage.obstacle_avoidance_pixels = std::max(
                0.0, spec["obstacle_avoidance_pixels"].cast<double>());
        }
        stage.end_point = tuple_to_point(spec["end_point"]);

        auto boundary = py::cast<py::list>(spec["boundary"]);
        stage.boundary.reserve(boundary.size());
        for (const auto& point_obj : boundary) {
            stage.boundary.push_back(tuple_to_point(point_obj));
        }

        py::handle start_obj = spec["start_point"];
        if (!start_obj.is_none()) {
            stage.start_point = tuple_to_point(start_obj);
        }
        if (spec.contains("planning_grid")) {
            auto planning_array = py::cast<
                py::array_t<int, py::array::c_style | py::array::forcecast>>(
                    spec["planning_grid"]);
            Grid parsed = parse_grid(planning_array);
            if (parsed.rows != grid.rows || parsed.cols != grid.cols) {
                throw std::runtime_error(
                    "planning_grid shape must match grid_map shape");
            }
            stage.planning_grid = std::move(parsed);
        }

        auto mask_array = py::cast<py::array_t<bool, py::array::c_style | py::array::forcecast>>(
            spec["region_mask"]);
        const auto mask_info = mask_array.request();
        if (mask_info.ndim != 2 ||
            static_cast<int>(mask_info.shape[0]) != grid.rows ||
            static_cast<int>(mask_info.shape[1]) != grid.cols) {
            throw std::runtime_error("region_mask shape must match grid_map shape");
        }

        const auto* mask_ptr = static_cast<const bool*>(mask_info.ptr);
        stage.region_mask.resize(static_cast<std::size_t>(grid.rows) *
                                 static_cast<std::size_t>(grid.cols));
        for (std::size_t i = 0; i < stage.region_mask.size(); ++i) {
            stage.region_mask[i] = mask_ptr[i] ? 1 : 0;
        }

        stages.push_back(std::move(stage));
    }

    return stages;
}

static py::list plan_core(
    const py::array_t<int, py::array::c_style | py::array::forcecast>& grid_array,
    const py::list& stage_specs,
    int spacing_pixels,
    double output_spacing_pixels) {
    if (spacing_pixels < 1) {
        throw std::runtime_error("spacing_pixels must be >= 1");
    }

    const Grid grid = parse_grid(grid_array);
    const auto stages = parse_stages(stage_specs, grid);
    const auto corner_path = plan_all_stages(grid, stages, spacing_pixels);
    const auto path = densify_path(corner_path, output_spacing_pixels);

    py::list result;
    for (const auto& point : path) {
        result.append(py::make_tuple(point.row, point.col, point.path_type));
    }
    return result;
}

PYBIND11_MODULE(_mst27_cpp, module) {
    module.doc() = "C++ planning kernel for mst27";
    module.attr("__version__") = "4.7.0";
    module.def("plan_core", &plan_core, py::arg("grid_map"), py::arg("stage_specs"),
               py::arg("spacing_pixels"), py::arg("output_spacing_pixels") = 0.0);
}
#endif  // MST27_HAVE_PYBIND
