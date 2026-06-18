// Author(s): Jan Friso Groote
// Copyright: see the accompanying file COPYING or copy at
// https://github.com/mCRL2org/mCRL2/blob/master/COPYING
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
/// \file utilities/detail/indexed_set.cpp
/// \brief This file contains some constants and functions shared
///        between indexed_sets and tables.

#ifndef MCRL2_UTILITIES_DETAIL_INDEXED_SET_H
#define MCRL2_UTILITIES_DETAIL_INDEXED_SET_H
#pragma once

#include "mcrl2/utilities/indexed_set.h"    // necessary for header test. 

namespace mcrl2::utilities
{
namespace detail
{

static const std::size_t STEP = 1; ///< The position on which the next hash entry is searched.

/// in the hashtable we use the following constant to indicate free positions.
static constexpr std::size_t EMPTY(std::numeric_limits<std::size_t>::max());

static constexpr std::size_t RESERVED(std::numeric_limits<std::size_t>::max()-1);

static constexpr float max_load_factor = 0.6f; ///< The load factor before the hash table is resized.

static constexpr std::size_t PRIME_NUMBER = 999953;

#ifndef NDEBUG  // Numbers are small in debug mode for more intensive checks. 
  static constexpr std::size_t minimal_hashtable_size = 16; 
#else
  static constexpr std::size_t minimal_hashtable_size = 2048;
#endif
  static constexpr std::size_t RESERVATION_FRACTION = 8;        // If the reserved keys entries are exploited, 1/RESERVATION_FRACTION new
                                                                // keys are reserved. This is an expensive operation, as it is executed
                                                                // using a lock_exclusive. 

static_assert(minimal_hashtable_size/RESERVATION_FRACTION != 0);
static_assert(minimal_hashtable_size>=8);       ///< With a max_load of 0.75 the minimal size of the hashtable must be 8.

} // namespace detail

#define INDEXED_SET_TEMPLATE template <class Key, bool ThreadSafe, typename Hash, typename Equals, typename Allocator, typename KeyTable>
#define INDEXED_SET indexed_set<Key, ThreadSafe, Hash, Equals, Allocator, KeyTable>

INDEXED_SET_TEMPLATE
inline void INDEXED_SET::reserve_indices(const std::size_t thread_index)
{
  lock_guard guard = m_shared_mutexes[thread_index].lock();

  if (m_next_index + m_shared_mutexes.size() >= m_keys.size())   // otherwise another process already reserved entries, and nothing needs to be done. 
  {
    assert(m_next_index <= m_keys.size());
    m_keys.resize(m_keys.size() + std::max(m_keys.size() / detail::RESERVATION_FRACTION, m_shared_mutexes.size()));  // Increase with at least the number of threads. 

    while ((detail::max_load_factor * m_hashtable.size()) < m_keys.size())
    {
       resize_hashtable(thread_index);
    }
  }
}

INDEXED_SET_TEMPLATE
inline typename INDEXED_SET::size_type INDEXED_SET::put_in_hashtable(
                  const key_type& key, 
                  std::size_t value, 
                  std::size_t& new_position,
                  const std::size_t thread_index)
{
  assert(thread_index < m_put_statistics.size());

  put_in_hashtable_statistics& statistics =
      m_put_statistics[thread_index];

  std::uint64_t iterations = 0;
  std::uint64_t occupied_probes = 0;
  std::uint64_t reserved_spins = 0;
  std::uint64_t cas_failures = 0;
  std::uint64_t key_comparisons = 0;

  std::uint64_t reserved_streak = 0;
  std::uint64_t maximum_reserved_streak = 0;

  auto finish = [&](const size_type result) -> size_type
  {
    statistics.loop_iterations += iterations;
    statistics.occupied_probes += occupied_probes;
    statistics.reserved_spins += reserved_spins;
    statistics.cas_failures += cas_failures;
    statistics.key_comparisons += key_comparisons;

    if (reserved_spins != 0)
    {
      ++statistics.calls_with_reserved;
    }

    statistics.max_iterations =
        std::max(statistics.max_iterations, iterations);

    statistics.max_reserved_streak =
        std::max(
          statistics.max_reserved_streak,
          maximum_reserved_streak
        );
    return result;
  };

  // Find a place to insert key and find whether key already exists.
  assert(m_hashtable.size() > 0);

  new_position = ((m_hasher(key) * detail::PRIME_NUMBER) >> 2) % m_hashtable.size();
  [[maybe_unused]] // Not used in release mode
  std::size_t start = new_position;

  while (true)
  {
    ++iterations;

    std::size_t index = m_hashtable[new_position];
    assert(index == detail::EMPTY || index == detail::RESERVED || index < m_keys.size());

    if (index == detail::EMPTY)
    {
      // Found an empty spot, insert a new index belonging to key,
      std::size_t pos=detail::EMPTY;
      if (reinterpret_cast<std::atomic<std::size_t>*>(&m_hashtable[new_position])->compare_exchange_strong(pos,value))
      {
        return finish(value);
      }
      // Another worker changed this slot before the CAS completed.
      ++cas_failures;
      index=pos;             // Insertion failed, but another process put an alternative value "pos"
                             // at this position. 
    }

    // If the index is RESERVED, we go into a busy loop, as another process 
    // will shortly change the RESERVED value into a sensible index. 
    if (index != detail::RESERVED) 
    {
      reserved_streak = 0;
      assert(index!=detail::EMPTY);
      assert(index < m_keys.size());

      ++occupied_probes;
      ++key_comparisons;
      if (m_equals(m_keys[index], key))
      {
        // key is already in the set, return position of key.
        assert(index<m_next_index && m_next_index<=m_keys.size());
        return finish(index);
      }
      assert(m_hashtable.size()>0);
      new_position = (new_position + detail::STEP) % m_hashtable.size();
      assert(new_position != start); // In this case the hashtable is full, which should never happen.
    } else {
      ++reserved_spins;
      ++reserved_streak;

      maximum_reserved_streak =
          std::max(
            maximum_reserved_streak,
            reserved_streak
          );
    }
  }

  // not reached. 
  std::abort();
  return detail::EMPTY;
}

INDEXED_SET_TEMPLATE
inline void INDEXED_SET::resize_hashtable(const std::size_t thread_index)
{
  assert(thread_index < m_put_statistics.size());
  m_hashtable.assign(m_hashtable.size() * 2, detail::EMPTY);
  size_t index = 0;
  for (const Key& k: m_keys)
  {
    if (index<m_next_index)
    {
      std::size_t new_position;  // The resulting new_position is not used here. 
      put_in_hashtable(k, index, new_position, thread_index);
    }
    else 
    {
      break;
    }
    ++index;
  }
}

INDEXED_SET_TEMPLATE
inline INDEXED_SET::indexed_set()
  : indexed_set(1, detail::minimal_hashtable_size)   // Run with one main thread. 
{} 

INDEXED_SET_TEMPLATE
inline INDEXED_SET::indexed_set(std::size_t number_of_threads)
  : indexed_set(number_of_threads, detail::minimal_hashtable_size)
{
  assert(number_of_threads != 0);
}

INDEXED_SET_TEMPLATE
inline INDEXED_SET::indexed_set(
           std::size_t number_of_threads,
           std::size_t initial_size,
           const hasher& hasher,
           const key_equal& equals)
  : m_hashtable(std::max(initial_size, detail::minimal_hashtable_size), detail::EMPTY), 
    m_mutex(new std::mutex()),
    m_hasher(hasher),
    m_equals(equals)
{
  assert(number_of_threads != 0);

  // Insert the main mutex.
  m_shared_mutexes.emplace_back();

  for (std::size_t i = 1; i < ((number_of_threads == 1) ? 1 : number_of_threads + 1); ++i)
  {
    // Copy the mutex n times for all the other threads.
    m_shared_mutexes.emplace_back(m_shared_mutexes[0]);
  }

  // Use the same indexing convention as m_shared_mutexes.
  //
  // Single-threaded:
  //     size == 1; valid index is 0
  //
  // Multithreaded with N workers:
  //     size == N + 1; valid worker indices are 1..N
  //     entry 0 remains unused
  m_put_statistics.resize(m_shared_mutexes.size());
}

INDEXED_SET_TEMPLATE
inline typename INDEXED_SET::size_type INDEXED_SET::index(const key_type& key, const std::size_t thread_index) const
{
  shared_guard guard = m_shared_mutexes[thread_index].lock_shared();
  assert(m_hashtable.size() > 0);

  std::size_t start = ((m_hasher(key) * detail::PRIME_NUMBER) >> 2) % m_hashtable.size();
  std::size_t position = start;
  do
  {
    std::size_t index = m_hashtable[position];
    if (index == detail::EMPTY)
    {
      return npos; // Not found.
    }
    // If the index is RESERVED, go into a busy loop. Another thread will 
    // change this RESERVED index shortly into a sensible index. 
    if (index != detail::RESERVED)
    {
      assert(index < m_keys.size());
      if (m_equals(key, m_keys[index]))
      {
        assert(index<m_next_index && m_next_index <= m_keys.size());
        return index;
      }

      assert(m_hashtable.size() > 0);
      position = (position + detail::STEP) % m_hashtable.size();
      assert(position != start); // The hashtable is full. This should never happen.
    }
  }
  while (true);

  std::abort();
  return npos; // Dummy return.
}

INDEXED_SET_TEMPLATE
inline typename INDEXED_SET::const_iterator INDEXED_SET::find(const key_type& key, const std::size_t thread_index) const
{
  const std::size_t idx = index(key, thread_index);
  if (idx < m_keys.size())
  {
    return begin(thread_index) + idx;
  }

  return end(thread_index);
}


INDEXED_SET_TEMPLATE
inline const Key& INDEXED_SET::at(std::size_t index) const
{
  if (index >= m_next_index)
  {
    throw std::out_of_range("indexed_set: index too large: " + std::to_string(index) + " > " + std::to_string(m_next_index) + ".");
  }

  return m_keys[index];
}

INDEXED_SET_TEMPLATE
inline const Key& INDEXED_SET::operator[](std::size_t index) const
{
  assert(index<m_keys.size());
  const Key& key = m_keys[index];
  return key;
}

INDEXED_SET_TEMPLATE
inline void INDEXED_SET::clear(const std::size_t thread_index)
{
  lock_guard guard = m_shared_mutexes[thread_index].lock();
  m_hashtable.assign(m_hashtable.size(), detail::EMPTY);

  m_keys.clear();
  m_next_index.store(0);
}


INDEXED_SET_TEMPLATE
inline std::pair<typename INDEXED_SET::size_type, bool> INDEXED_SET::insert(const Key& key, const std::size_t thread_index)
{
  std::chrono::steady_clock::time_point func_start = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point lock_start;
  std::chrono::steady_clock::time_point lock_end;
  std::chrono::steady_clock::time_point reserve_end;
  std::chrono::steady_clock::time_point hashtable_end;
  std::chrono::steady_clock::time_point finalize_end;
  std::chrono::steady_clock::time_point func_end;

  assert(thread_index < m_shared_mutexes.size());
  assert(thread_index < m_put_statistics.size());

  put_in_hashtable_statistics& statistics =
      m_put_statistics[thread_index];

  statistics.calls++;

  lock_start = std::chrono::steady_clock::now();
  shared_guard guard = m_shared_mutexes[thread_index].lock_shared();
  lock_end = std::chrono::steady_clock::now();

  assert(m_next_index <= m_keys.size());
  if (m_next_index + m_shared_mutexes.size() >= m_keys.size())
  {
    guard.unlock_shared();
    reserve_indices(thread_index);
    guard.lock_shared();
  }
  reserve_end = std::chrono::steady_clock::now();
  std::size_t new_position;
  const std::size_t index = put_in_hashtable(key, detail::RESERVED, new_position, thread_index);
  
  hashtable_end = std::chrono::steady_clock::now();

  statistics.lock_nanoseconds += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(lock_end - lock_start).count());
  statistics.reserve_nanoseconds += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(reserve_end - lock_end).count());
  statistics.hashtable_nanoseconds += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(hashtable_end - reserve_end).count());
  
  if (index != detail::RESERVED) // Key already exists.
  {
    assert(index < m_next_index && m_next_index <= m_keys.size());

    finalize_end = std::chrono::steady_clock::now();
    statistics.early_exit_nanoseconds += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(finalize_end - hashtable_end).count());
    guard.unlock_shared();
    func_end = std::chrono::steady_clock::now();
    statistics.unlock_nanoseconds += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(func_end - finalize_end).count());
    statistics.function_inner_nanoseconds += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(func_end - func_start).count());
    return std::make_pair(index, false);
  }

  const std::size_t new_index = m_next_index.fetch_add(1);
  assert(new_index < m_keys.size());
  m_keys[new_index] = key; 

  std::atomic_thread_fence(std::memory_order_seq_cst);   // Necessary for ARM. std::memory_order_acquire and 
                                                            // std::memory_order_release appear to work, too.
  m_hashtable[new_position] = new_index;


  assert(new_index < m_next_index && m_next_index <= m_keys.size());
  
  finalize_end = std::chrono::steady_clock::now();
  statistics.finalize_nanoseconds += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(finalize_end - hashtable_end).count());
  guard.unlock_shared();
  func_end = std::chrono::steady_clock::now();
  statistics.unlock_nanoseconds += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(func_end - finalize_end).count());
  statistics.function_inner_nanoseconds += static_cast<std::uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(func_end - func_start).count());
  return std::make_pair(new_index, true);
}

INDEXED_SET_TEMPLATE
inline void INDEXED_SET::reset_put_in_hashtable_statistics()
{
  // This function is deliberately not synchronized. It must only be called
  // before worker threads start or after they have all joined.
  for (put_in_hashtable_statistics& statistics:
       m_put_statistics)
  {
    statistics = put_in_hashtable_statistics{};
  }
}
INDEXED_SET_TEMPLATE
inline void INDEXED_SET::print_put_in_hashtable_statistics() const
{
  put_in_hashtable_statistics total{};

  for (std::size_t thread_index = 0;
       thread_index < m_put_statistics.size();
       ++thread_index)
  {
    const put_in_hashtable_statistics& statistics =
        m_put_statistics[thread_index];

    if (statistics.calls == 0)
    {
      continue;
    }

    const double average_iterations =
        static_cast<double>(statistics.loop_iterations) /
        static_cast<double>(statistics.calls);

    const double average_reserved_spins =
        static_cast<double>(statistics.reserved_spins) /
        static_cast<double>(statistics.calls);

    const double reserved_call_percentage =
        100.0 *
        static_cast<double>(statistics.calls_with_reserved) /
        static_cast<double>(statistics.calls);

    double function_inner_seconds = 0.0;
    double lock_seconds = 0.0;
    double reserve_seconds = 0.0;
    double hashtable_seconds = 0.0;
    double early_exit_seconds = 0.0;
    double finalize_seconds = 0.0;
    double unlock_seconds = 0.0;

    function_inner_seconds =
        static_cast<double>(statistics.function_inner_nanoseconds) / 1.0e9 ;
    lock_seconds =
        static_cast<double>(statistics.lock_nanoseconds) / 1.0e9 ;
    reserve_seconds =
        static_cast<double>(statistics.reserve_nanoseconds) / 1.0e9;
    hashtable_seconds =
        static_cast<double>(statistics.hashtable_nanoseconds) / 1.0e9;
    early_exit_seconds =
        static_cast<double>(statistics.early_exit_nanoseconds) / 1.0e9;
    finalize_seconds =
        static_cast<double>(statistics.finalize_nanoseconds) / 1.0e9;
    unlock_seconds =
        static_cast<double>(statistics.unlock_nanoseconds) / 1.0e9;

    mCRL2log(log::verbose)
      << "put_in_hashtable"
      << " thread=" << thread_index
      << " calls=" << statistics.calls
      << " function_inner_seconds=" << function_inner_seconds
      << " lock_seconds=" << lock_seconds
      << " reserve_seconds=" << reserve_seconds
      << " hashtable_seconds=" << hashtable_seconds
      << " early_exit_seconds=" << early_exit_seconds
      << " finalize_seconds=" << finalize_seconds
      << " unlock_seconds=" << unlock_seconds
      << " avg_iterations=" << average_iterations
      << " avg_reserved_spins=" << average_reserved_spins
      << " reserved_calls_percent=" << reserved_call_percentage
      << " occupied_probes=" << statistics.occupied_probes
      << " key_comparisons=" << statistics.key_comparisons
      << " cas_failures=" << statistics.cas_failures
      << " max_iterations=" << statistics.max_iterations
      << " max_reserved_streak="
      << statistics.max_reserved_streak
      << '\n';

    total.calls += statistics.calls;
    total.loop_iterations += statistics.loop_iterations;
    total.occupied_probes += statistics.occupied_probes;
    total.reserved_spins += statistics.reserved_spins;
    total.cas_failures += statistics.cas_failures;
    total.key_comparisons += statistics.key_comparisons;
    total.calls_with_reserved += statistics.calls_with_reserved;
    total.function_inner_nanoseconds += statistics.function_inner_nanoseconds;
    total.lock_nanoseconds += statistics.lock_nanoseconds;
    total.reserve_nanoseconds += statistics.reserve_nanoseconds;
    total.hashtable_nanoseconds += statistics.hashtable_nanoseconds;
    total.early_exit_nanoseconds += statistics.early_exit_nanoseconds;
    total.finalize_nanoseconds += statistics.finalize_nanoseconds;
    total.unlock_nanoseconds += statistics.unlock_nanoseconds;

    total.max_iterations =
        std::max(
          total.max_iterations,
          statistics.max_iterations
        );

    total.max_reserved_streak =
        std::max(
          total.max_reserved_streak,
          statistics.max_reserved_streak
        );
  }

  if (total.calls == 0)
  {
    mCRL2log(log::verbose) << "put_in_hashtable total calls=0\n";
    return;
  }

  const double average_iterations =
      static_cast<double>(total.loop_iterations) /
      static_cast<double>(total.calls);

  const double average_reserved_spins =
      static_cast<double>(total.reserved_spins) /
      static_cast<double>(total.calls);

  const double reserved_call_percentage =
      100.0 *
      static_cast<double>(total.calls_with_reserved) /
      static_cast<double>(total.calls);

  double function_inner_seconds = 0.0;
  double lock_seconds = 0.0;
  double reserve_seconds = 0.0;
  double hashtable_seconds = 0.0;
  double early_exit_seconds = 0.0;
  double finalize_seconds = 0.0;
  double unlock_seconds = 0.0;

    function_inner_seconds =
        static_cast<double>(total.function_inner_nanoseconds) / 1.0e9 ;
    lock_seconds =
        static_cast<double>(total.lock_nanoseconds) / 1.0e9;
    reserve_seconds =
        static_cast<double>(total.reserve_nanoseconds) / 1.0e9;
    hashtable_seconds =
        static_cast<double>(total.hashtable_nanoseconds) / 1.0e9;
    early_exit_seconds =
        static_cast<double>(total.early_exit_nanoseconds) / 1.0e9;
    finalize_seconds =
        static_cast<double>(total.finalize_nanoseconds) / 1.0e9;
    unlock_seconds =
        static_cast<double>(total.unlock_nanoseconds) / 1.0e9;


  mCRL2log(log::verbose)
    << "put_in_hashtable total"
    << " calls=" << total.calls
    << " function_inner_seconds=" << function_inner_seconds
    << " lock_seconds=" << lock_seconds
    << " reserve_seconds=" << reserve_seconds
    << " hashtable_seconds=" << hashtable_seconds
    << " early_exit_seconds=" << early_exit_seconds
    << " finalize_seconds=" << finalize_seconds
    << " unlock_seconds=" << unlock_seconds
    << " avg_iterations=" << average_iterations
    << " avg_reserved_spins=" << average_reserved_spins
    << " reserved_calls_percent=" << reserved_call_percentage
    << " occupied_probes=" << total.occupied_probes
    << " key_comparisons=" << total.key_comparisons
    << " cas_failures=" << total.cas_failures
    << " max_iterations=" << total.max_iterations
    << " max_reserved_streak=" << total.max_reserved_streak
    << '\n';
}
#undef INDEXED_SET_TEMPLATE 
#undef INDEXED_SET 


} // namespace mcrl2::utilities



#endif // MCRL2_UTILITIES_DETAIL_INDEXED_SET_H
