#include <trajectory/state.h>

template <class T>
State<T>::State(const State<T>::Vec3t &pos, const State<T>::Vec3t &vel,
                const T yaw)
    : pos_(pos), vel_(vel), yaw_(yaw) {}

template <class T> State<T>::~State() {}

template <class T> typename State<T>::Vec3t State<T>::Pos() const {
  return pos_;
}

template <class T> typename State<T>::Vec3t State<T>::Vel() const {
  return vel_;
}

template <class T> T State<T>::Yaw() const { return yaw_; }

template <class T> void State<T>::print() const {
  printf("Position: [ %.2f, %.2f, %.2f ]\n", pos_(0), pos_(1), pos_(2));
  printf("Velocity: [ %.2f, %.2f, %.2f ]\n", vel_(0), vel_(1), vel_(2));
  printf("Yaw : %.2f\n", yaw_);
}
