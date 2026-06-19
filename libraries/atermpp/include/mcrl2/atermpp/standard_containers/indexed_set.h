// Author(s): Jan Friso Groote
// Copyright: see the accompanying file COPYING or copy at
// https://github.com/mCRL2org/mCRL2/blob/master/COPYING
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef MCRL2_ATERMPP_INDEXED_SET_H
#define MCRL2_ATERMPP_INDEXED_SET_H

#include "mcrl2/atermpp/detail/thread_aterm_pool.h"
#include "mcrl2/atermpp/standard_containers/deque.h"
#include "mcrl2/utilities/detail/container_utility.h"
#include "mcrl2/utilities/indexed_set.h"
#include "mcrl2/utilities/shared_mutex.h"


namespace atermpp
{

/// \brief A set that assigns each element an unique index, and protects its internal terms en masse.
template<typename Key,
         bool ThreadSafe = false,
         typename Hash = std::hash<Key>,
         typename Equals = std::equal_to<Key>,
         typename Allocator = std::allocator<Key>,
         typename KeyTable = atermpp::deque<Key > >
class indexed_set: public mcrl2::utilities::indexed_set<Key, ThreadSafe, Hash, Equals, Allocator, KeyTable>
{
  using super = mcrl2::utilities::indexed_set<Key, ThreadSafe, Hash, Equals, Allocator, KeyTable>;
struct alignas(64) indexed_set_stats {
        std::uint64_t calls = 0;
        std::uint64_t lock_nanoseconds = 0;
        std::uint64_t unlock_nanoseconds = 0;
        std::uint64_t work_nanoseconds = 0;
    };
    std::vector<indexed_set_stats> m_indexed_set_stats_;
public:
  using size_type = typename super::size_type;

  /// \brief Constructor of an empty indexed set. Starts with a hashtable of size 128.
  indexed_set() = default;

  /// \brief Constructor of an empty indexed set. Starts with a hashtable of size 128.
  indexed_set(std::size_t number_of_threads)
    : super(number_of_threads)
  {
    m_indexed_set_stats_.resize(number_of_threads + 1);
  }

  /// \brief Constructor of an empty index set. Starts with a hashtable of the indicated size. 
  /// \param initial_hashtable_size The initial size of the hashtable.
  /// \param hash The hash function.
  /// \param equals The comparison function for its elements.

  indexed_set(std::size_t number_of_threads,
              std::size_t initial_hashtable_size,
              const typename super::hasher& hash = typename super::hasher(),
              const typename super::key_equal& equals = typename super::key_equal()) 
    : super(number_of_threads, initial_hashtable_size, hash, equals)
  {}
  
  void clear(std::size_t thread_index=0)
  {
    mcrl2::utilities::shared_guard guard = detail::g_thread_term_pool().lock_shared();
    super::clear(thread_index);
  }

  std::pair<size_type, bool> insert(const Key& key, std::size_t thread_index=0)
  {
    std::chrono::steady_clock::time_point start_lock = std::chrono::steady_clock::now();
    mcrl2::utilities::shared_guard guard = detail::g_thread_term_pool().lock_shared();
    std::chrono::steady_clock::time_point start_work = std::chrono::steady_clock::now();
    auto res = super::insert(key, thread_index);
    std::chrono::steady_clock::time_point start_unlock = std::chrono::steady_clock::now();
    guard.unlock_shared();
    std::chrono::steady_clock::time_point end_unlock = std::chrono::steady_clock::now();
    indexed_set_stats& statistics = m_indexed_set_stats_[thread_index];
    statistics.calls += 1;
    statistics.lock_nanoseconds += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(start_work - start_lock).count());
    statistics.work_nanoseconds += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(start_unlock - start_work).count());
    statistics.unlock_nanoseconds += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(end_unlock - start_unlock).count());

    return res;
  }

  void print_stats() const {
    indexed_set_stats total{};
    for (std::size_t thread_index = 0; thread_index < m_indexed_set_stats_.size(); ++thread_index) {
      const indexed_set_stats& statistics = m_indexed_set_stats_[thread_index];
      total.calls += statistics.calls;
      total.lock_nanoseconds += statistics.lock_nanoseconds;
      total.unlock_nanoseconds += statistics.unlock_nanoseconds;
      total.work_nanoseconds += statistics.work_nanoseconds;
    }

    if (total.calls == 0)
    {
      std::cout << "put_in_hashtable total calls=0\n";
      return;
    }

    double lock_seconds = static_cast<double>(total.lock_nanoseconds) / 1.0e9 ;
    double unlock_seconds = static_cast<double>(total.unlock_nanoseconds) / 1.0e9 ;
    double work_seconds = static_cast<double>(total.work_nanoseconds) / 1.0e9 ;

    std::cout
      << "aTerm indexed set total"
      << " calls=" << total.calls
      << " lock_seconds=" << lock_seconds
      << " unlock_seconds=" << unlock_seconds
      << " work_seconds=" << work_seconds
      << '\n';

  }
};
} // end namespace atermppp

namespace mcrl2::utilities::detail
{

// Specialization of a function defined in mcrl2/utilities/detail/container_utility.h.
// In utilities, atermpp is not known. 
template<typename Key,
         bool ThreadSafe = false,
         typename Hash = std::hash<Key>,
         typename Equals = std::equal_to<Key>,
         typename Allocator = std::allocator<Key>,
         typename KeyTable = atermpp::deque<Key > >
bool contains(const atermpp::indexed_set<Key, ThreadSafe, Hash, Equals, Allocator, KeyTable>& c, 
              const typename atermpp::indexed_set<Key, ThreadSafe, Hash, Equals, Allocator, KeyTable>::key_type& v,
              const std::size_t thread_index=0)
{
  return c.find(v, thread_index) != c.end(thread_index);
}

} // namespace mcrl2::utilities::detail

#endif // MCRL2_ATERMPP_INDEXED_SET_H
