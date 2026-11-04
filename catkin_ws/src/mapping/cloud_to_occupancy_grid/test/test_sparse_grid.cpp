#include "../src/sparse_grid.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>
int main() {
  using mapper::SparseGrid;
  auto budget = std::make_shared<mapper::TileBudget>(32 * 1024 * 1024);
  SparseGrid grid(budget);
  assert(budget->used == 0 && grid.max_x == -1);
  constexpr int width = 641, height = 193;
  std::vector<float> dense(width * height, 0);
  std::vector<bool> seen(width * height, false);
  std::mt19937 rng(19);
  for (int n = 0; n < 100000; ++n) {
    const int x = rng() % width, y = rng() % height;
    const double inc = rng() % 2 ? .8472978603872034 : -.8472978603872034;
    grid.update(x, y, inc, -4., 4.);
    dense[y * width + x] = static_cast<float>(std::max(-4., std::min(4., dense[y * width + x] + inc)));
    seen[y * width + x] = true;
  }
  std::size_t visited = 0;
  grid.visit([&](int x, int y, float odds) {
    assert(seen[y * width + x]); assert(odds == dense[y * width + x]); ++visited;
  });
  assert(visited == static_cast<std::size_t>(std::count(seen.begin(), seen.end(), true)));
  assert(grid.min_x == 0 && grid.max_x == width - 1 && grid.max_y == height - 1);
  const auto bytes = budget->used.load();
  { SparseGrid replay(budget); replay.update(1000, 1000, -1., -4., 4.);
    assert(budget->used == bytes + sizeof(SparseGrid::Tile));
    grid.swap(replay); assert(grid.min_x == 1000 && grid.max_y == 1000);
  }
  assert(budget->used == sizeof(SparseGrid::Tile));
  grid.clear(); assert(budget->used == 0 && grid.max_x == -1);
  auto limited = std::make_shared<mapper::TileBudget>(sizeof(SparseGrid::Tile));
  { SparseGrid live(limited), replay(limited);
    live.update(63, 63, 1., -4., 4.);
    bool rejected = false;
    try { replay.update(64, 64, 1., -4., 4.); } catch (const std::runtime_error&) { rejected = true; }
    assert(rejected && replay.tileCount() == 0);
    live.clear(); replay.update(64, 64, 1., -4., 4.);
  }
  assert(limited->used == 0);
  std::cout << "dense equivalence, bounds, replay swap, shared cap, reset passed\n";
}
