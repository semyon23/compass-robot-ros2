#include "compass_core/control_lease.hpp"

#include <cstdio>
#include <random>

namespace compass_core {

namespace {

// 128 случайных бит в hex: lease_id нельзя угадать по client_id другой вкладки.
std::string make_lease_id()
{
  static std::mt19937_64 rng{std::random_device{}()};
  char buf[33];
  std::snprintf(buf, sizeof(buf), "%016llx%016llx",
                static_cast<unsigned long long>(rng()), static_cast<unsigned long long>(rng()));
  return buf;
}

}  // namespace

ControlLease::Acquired ControlLease::acquire(const std::string & client_id,
                                             const std::string & client_name, bool force,
                                             Clock::time_point now)
{
  Acquired result;
  const bool other = active() && holder_id_ != client_id;
  if (other && !force) {
    return result;
  }
  if (other) {
    result.previous_holder_id = holder_id_;
  }
  holder_id_ = client_id;
  holder_name_ = client_name;
  lease_id_ = make_lease_id();
  last_seen_ = now;
  result.granted = true;
  result.lease_id = lease_id_;
  return result;
}

bool ControlLease::release(const std::string & lease_id)
{
  if (!active() || lease_id != lease_id_) {
    return false;
  }
  clear();
  return true;
}

bool ControlLease::touch(const std::string & lease_id, Clock::time_point now)
{
  if (!active() || lease_id != lease_id_) {
    return false;
  }
  last_seen_ = now;
  return true;
}

bool ControlLease::expire(Clock::time_point now, Clock::duration timeout)
{
  if (!active() || now - last_seen_ <= timeout) {
    return false;
  }
  clear();
  return true;
}

void ControlLease::clear()
{
  holder_id_.clear();
  holder_name_.clear();
  lease_id_.clear();
}

}  // namespace compass_core
