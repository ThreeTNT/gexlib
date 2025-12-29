#pragma once

#include "pros/rtos.hpp"
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>

namespace gexlib {
namespace detail {
template <typename T>
struct FutureState {
	std::optional<T> value;
	bool ready = false;
	bool has_error = false;
	std::string error_message;
	bool cancelled = false;
	std::string cancel_reason;
};

template <typename T>
struct SharedFutureState {
	SharedFutureState() = default;
	SharedFutureState(const SharedFutureState&) = delete;
	SharedFutureState& operator=(const SharedFutureState&) = delete;

	pros::MutexVar<FutureState<T>> storage;
};
} // namespace detail

template <typename T>
class Promise;

template <typename T>
class Future {
	static_assert(!std::is_void_v<T>, "gexlib::Future requires a non-void type");

public:
	Future() = default;
	Future(Future&&) noexcept = default;
	Future& operator=(Future&&) noexcept = default;

	Future(const Future&) = delete;
	Future& operator=(const Future&) = delete;

	bool valid() const { return static_cast<bool>(state); }

	bool ready() const {
		if (!state) {
			return false;
		}
		auto lock = state->storage.lock();
		return lock->ready;
	}

	bool has_error() const {
		if (!state) {
			return false;
		}
		auto lock = state->storage.lock();
		return lock->has_error;
	}

	bool cancelled() const {
		if (!state) {
			return false;
		}
		auto lock = state->storage.lock();
		return lock->cancelled;
	}

	std::string cancellation_reason() const {
		if (!state) {
			return {};
		}
		auto lock = state->storage.lock();
		return lock->cancel_reason;
	}

	bool cancel(std::string reason = "Cancelled") const {
		if (!state) {
			return false;
		}
		auto lock = state->storage.lock();
		if (lock->ready) {
			return false;
		}
		lock->cancelled = true;
		lock->cancel_reason = std::move(reason);
		lock->value.reset();
		lock->has_error = false;
		lock->error_message.clear();
		return true;
	}

	std::string error_message() const {
		if (!state) {
			return {};
		}
		auto lock = state->storage.lock();
		return lock->error_message;
	}

	void wait(std::uint32_t poll_ms = 5) const {
		while (!ready()) {
			pros::delay(poll_ms);
		}
	}

	template <typename Rep, typename Period>
	bool wait_for(const std::chrono::duration<Rep, Period>& duration, std::uint32_t poll_ms = 5) const {
		const auto target_ms = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
		if (target_ms <= 0) {
			return ready();
		}

		std::int64_t elapsed = 0;
		while (elapsed < target_ms) {
			if (ready()) {
				return true;
			}
			pros::delay(poll_ms);
			elapsed += poll_ms;
		}

		return ready();
	}

	T get(std::uint32_t poll_ms = 5) const {
		if (!state) {
			throw std::logic_error("Future has no shared state");
		}

		wait(poll_ms);
		auto lock = state->storage.lock();
		if (lock->has_error) {
			throw std::runtime_error(lock->error_message);
		}
		if (lock->cancelled) {
			throw std::runtime_error(lock->cancel_reason.empty() ? "Future cancelled" : ("Future cancelled: " + lock->cancel_reason));
		}
		if (!lock->value.has_value()) {
			throw std::logic_error("Future satisfied without value");
		}
		return *lock->value;
	}

	std::optional<T> try_get() const {
		if (!state) {
			return std::nullopt;
		}
		auto lock = state->storage.lock();
			if (!lock->ready || lock->has_error || lock->cancelled || !lock->value.has_value()) {
			return std::nullopt;
		}
		return lock->value;
	}

private:
	explicit Future(std::shared_ptr<detail::SharedFutureState<T>> shared) : state(std::move(shared)) {}

	std::shared_ptr<detail::SharedFutureState<T>> state;

	friend class Promise<T>;
};

template <typename T>
class Promise {
	static_assert(!std::is_void_v<T>, "gexlib::Promise requires a non-void type");

public:
	Promise() : state(std::make_shared<detail::SharedFutureState<T>>()) {}

	Promise(Promise&&) noexcept = default;
	Promise& operator=(Promise&&) noexcept = default;

	Promise(const Promise&) = delete;
	Promise& operator=(const Promise&) = delete;

	Future<T> get_future() {
		ensure_state();
		return Future<T>(state);
	}

	bool ready() const {
		if (!state) {
			return false;
		}
		auto lock = state->storage.lock();
		return lock->ready;
	}

	void reset() { state = std::make_shared<detail::SharedFutureState<T>>(); }

	void set_value(const T& value) {
		emplace(value);
	}

	void set_value(T&& value) {
		emplace(std::move(value));
	}

	template <typename... Args>
	void emplace(Args&&... args) {
		ensure_state();
		auto lock = state->storage.lock();
		if (lock->ready && !lock->cancelled) {
			throw std::logic_error("Future already satisfied");
		}
		lock->value.emplace(std::forward<Args>(args)...);
		lock->ready = true;
	}

	void set_exception(std::string message) {
		ensure_state();
		auto lock = state->storage.lock();
		if (lock->ready && !lock->cancelled) {
			throw std::logic_error("Future already satisfied");
		}
		lock->has_error = true;
		lock->ready = true;
		lock->error_message = std::move(message);
	}

	bool cancel(std::string reason = "Cancelled") {
		ensure_state();
		auto lock = state->storage.lock();
		if (lock->ready) {
			return false;
		}
		lock->cancelled = true;
		lock->ready = true;
		lock->cancel_reason = std::move(reason);
		lock->value.reset();
		lock->has_error = false;
		lock->error_message.clear();
		return true;
	}

private:
	void ensure_state() {
		if (!state) {
			state = std::make_shared<detail::SharedFutureState<T>>();
		}
	}

	std::shared_ptr<detail::SharedFutureState<T>> state;
};
}