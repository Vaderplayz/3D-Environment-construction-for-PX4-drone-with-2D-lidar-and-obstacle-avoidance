#pragma once

#include <stdexcept>
#include <utility>
#include <variant>

namespace pc_env_viewer {

template <typename T, typename E>
class Result {
public:
  static Result Ok(T value)
  {
    return Result(true, std::move(value));
  }

  static Result Err(E error)
  {
    return Result(false, std::move(error));
  }

  bool ok() const
  {
    return ok_;
  }

  explicit operator bool() const
  {
    return ok_;
  }

  const T & value() const
  {
    if (!ok_) {
      throw std::logic_error("Result does not contain a value");
    }
    return std::get<T>(storage_);
  }

  T & value()
  {
    if (!ok_) {
      throw std::logic_error("Result does not contain a value");
    }
    return std::get<T>(storage_);
  }

  T take_value()
  {
    if (!ok_) {
      throw std::logic_error("Result does not contain a value");
    }
    return std::move(std::get<T>(storage_));
  }

  const E & error() const
  {
    if (ok_) {
      throw std::logic_error("Result does not contain an error");
    }
    return std::get<E>(storage_);
  }

  E & error()
  {
    if (ok_) {
      throw std::logic_error("Result does not contain an error");
    }
    return std::get<E>(storage_);
  }

  E take_error()
  {
    if (ok_) {
      throw std::logic_error("Result does not contain an error");
    }
    return std::move(std::get<E>(storage_));
  }

private:
  explicit Result(bool ok, T value)
  : ok_(ok), storage_(std::move(value))
  {
  }

  explicit Result(bool ok, E error)
  : ok_(ok), storage_(std::move(error))
  {
  }

  bool ok_;
  std::variant<T, E> storage_;
};

}  // namespace pc_env_viewer
