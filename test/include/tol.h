#ifndef _TOL_
#define _TOL_

// Define a small TOLERANCE for floating-point comparisons
constexpr double HARD_TOLERANCE = 1e-9;
// A looser TOLERANCE for complex transformations involving approximations
constexpr double SOFT_TOLERANCE = 1e-6;
// A looser unit-aware TOLERANCE for cm-level computations
constexpr double CM_LEVEL_TOLERANCE = 1e-2;
// A looser unit-aware TOLERANCE for mm-level computations
constexpr double MM_LEVEL_TOLERANCE = 1e-3;

#endif /* _TOL_ */
