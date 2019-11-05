# Try to find CppAD
# Once done this will define
#
#  CPPAD_FOUND - system has eigen lib with correct version
#  CPPAD_INCLUDE_DIRS - the eigen include directory

find_path(CPPAD_INCLUDE_DIRS NAMES cppad/cppad.hpp)
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CppAD DEFAULT_MSG CPPAD_INCLUDE_DIRS)
mark_as_advanced(CPPAD_INCLUDE_DIRS)
