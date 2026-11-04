#pragma once
#include <algorithm>
#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <unordered_map>

namespace mapper {
// Only tile payload is charged here; hash nodes and outgoing snapshots are separate.
struct TileBudget {
  explicit TileBudget(std::size_t bytes) : limit(bytes) {}
  const std::size_t limit;
  std::atomic<std::size_t> used{0};
};
class SparseGrid {
 public:
  static constexpr int kSide = 64;
  struct Tile {
    std::array<float, kSide * kSide> odds{};
    std::array<uint8_t, kSide * kSide> observed{};
  };
  explicit SparseGrid(std::shared_ptr<TileBudget> budget) : budget_(std::move(budget)) {}
  ~SparseGrid() { clear(); }
  SparseGrid(const SparseGrid&) = delete;
  SparseGrid& operator=(const SparseGrid&) = delete;
  void clear() {
    budget_->used.fetch_sub(tiles_.size() * sizeof(Tile));
    tiles_.clear(); min_x = min_y = 0; max_x = max_y = -1;
  }
  void swap(SparseGrid& other) {
    if (budget_ != other.budget_) throw std::logic_error("different tile budgets");
    tiles_.swap(other.tiles_);
    std::swap(min_x, other.min_x); std::swap(min_y, other.min_y);
    std::swap(max_x, other.max_x); std::swap(max_y, other.max_y);
  }
  void update(int x, int y, double increment, double low, double high) {
    const auto key = (static_cast<uint64_t>(x / kSide) << 32U) |
                     static_cast<uint32_t>(y / kSide);
    auto found = tiles_.find(key);
    if (found == tiles_.end()) {
      auto used = budget_->used.load();
      do {
        if (used > budget_->limit || sizeof(Tile) > budget_->limit - used)
          throw std::runtime_error("sparse grid memory limit reached; reset or restart scan");
      } while (!budget_->used.compare_exchange_weak(used, used + sizeof(Tile)));
      try { found = tiles_.emplace(key, std::make_unique<Tile>()).first; }
      catch (...) { budget_->used.fetch_sub(sizeof(Tile)); throw; }
    }
    auto& tile = *found->second;
    const int i = (y % kSide) * kSide + x % kSide;
    tile.odds[i] = static_cast<float>(std::max(low, std::min(high, tile.odds[i] + increment)));
    tile.observed[i] = 1;
    if (max_x < 0) { min_x = max_x = x; min_y = max_y = y; }
    else { min_x = std::min(min_x, x); min_y = std::min(min_y, y);
           max_x = std::max(max_x, x); max_y = std::max(max_y, y); }
  }
  template<class Visitor> void visit(Visitor visitor) const {
    for (const auto& entry : tiles_) {
      const int bx = static_cast<int>(entry.first >> 32U) * kSide;
      const int by = static_cast<int>(entry.first & 0xffffffffU) * kSide;
      const auto& tile = *entry.second;
      for (int i = 0; i < kSide * kSide; ++i)
        if (tile.observed[i]) visitor(bx + i % kSide, by + i / kSide, tile.odds[i]);
    }
  }
  std::size_t tileCount() const { return tiles_.size(); }
  int min_x = 0, min_y = 0, max_x = -1, max_y = -1;
 private:
  std::shared_ptr<TileBudget> budget_;
  std::unordered_map<uint64_t, std::unique_ptr<Tile>> tiles_;
};
}  // namespace mapper
