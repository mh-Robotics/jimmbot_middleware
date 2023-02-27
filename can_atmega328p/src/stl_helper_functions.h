#ifndef STL_HELPER_FUNCTIONS_H_
#define STL_HELPER_FUNCTIONS_H_

namespace std {
/**
 * @brief @todo Add doxy doc
 *
 */
struct once_flag {
  /**
   * @brief @todo Add doxy doc
   *
   */
  bool is_called = false;
};

/**
 * @brief @todo Add doxy doc
 *
 * @tparam Callable
 * @tparam Args
 * @param flag
 * @param f
 * @param args
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
