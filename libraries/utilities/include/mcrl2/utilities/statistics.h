// Utility functions to format simple timing/statistics summaries
// Added to centralize formatting so different modules log consistently.
#ifndef MCRL2_UTILITIES_STATISTICS_H
#define MCRL2_UTILITIES_STATISTICS_H

#include <string>
#include <sstream>
#include <vector>
#include <cstdint>

namespace mcrl2::utilities
{
struct alignas(64) lock_stats {
        std::uint64_t calls = 0;
        std::uint64_t lock_nanoseconds = 0;
        std::uint64_t work_nanoseconds = 0;
    };

inline lock_stats sum(const std::vector<lock_stats>& stats) {
  lock_stats total{};
  for (std::size_t thread_index = 0; thread_index < stats.size(); ++thread_index) {
    const lock_stats& statistics = stats[thread_index];
    total.calls += statistics.calls;
    total.lock_nanoseconds += statistics.lock_nanoseconds;
    total.work_nanoseconds += statistics.work_nanoseconds;
  }
  return total;
}

inline std::string format_three_field_stats(const std::string& title, const std::vector<lock_stats>& stats)
{
  lock_stats total = mcrl2::utilities::sum(stats);
  std::ostringstream ss;
  if (total.calls == 0)
  {
    ss << title << " total calls=0\n";
    return ss.str();
  }

  double lock_seconds = static_cast<double>(total.lock_nanoseconds) / 1.0e9;
  double work_seconds = static_cast<double>(total.work_nanoseconds) / 1.0e9;

  ss
    << title
    << " calls=" << total.calls
    << " lock_seconds=" << lock_seconds
    << " work_seconds=" << work_seconds
    << '\n';

  return ss.str();
}

} // namespace mcrl2::utilities

#endif // MCRL2_UTILITIES_STATISTICS_H
