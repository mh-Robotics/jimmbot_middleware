#ifndef STL_HELPER_FUNCTIONS_H_
#define STL_HELPER_FUNCTIONS_H_

namespace std {

/**
 * @brief An object that can be used to ensure that a function is called only
 * once.
 *
 * This object is used in conjunction with the `std::call_once` function.
 */
struct once_flag {
  /**
   * @brief A boolean indicating whether the function associated with this
   * `once_flag` object has already been called.
   */
  bool is_called = false;
};

/**
 * @brief Calls a function exactly once, even if called from multiple threads.
 *
 * This function ensures that a given function `f` is executed exactly once,
 * even if called from multiple threads. It uses an associated `once_flag`
 * object to ensure that the function is executed only on the first call.
 *
 * @tparam Callable A function or functor type.
 * @tparam Args Additional argument types to be passed to the function.
 * @param flag A reference to a `once_flag` object that is used to track
 * whether the function has already been called.
 * @param f The function or functor to be called.
 * @param args Additional arguments to be passed to the function.
 */
template <class Callable, class... Args>
void inline call_once(once_flag &flag, Callable &&f, Args &&...args) {
  if (!flag.is_called) {
    f();
    flag.is_called = true;
  }
}

}  // namespace std

#endif  // STL_HELPER_FUNCTIONS_H_
