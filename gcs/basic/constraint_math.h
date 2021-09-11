#ifndef GCS_BASIC_CONSTRAINT_MATH
#define GCS_BASIC_CONSTRAINT_MATH

//! @brief Set two values to equal
template <typename T1, typename T2>
T1 equate(const T1& x1, const T2& x2) {
    return x1 - x2;
}

//! @brief Set the difference between 2 values
template <typename T>
T difference(const T& x1, const T& x2, const T& d) {
    return ceres::abs(x1 - x2) - d;
}

#endif  // GCS_BASIC_CONSTRAINT_MATH
