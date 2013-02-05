AC_INIT([KSP Library], [0.1], [leo.osvald@gmail.com],
	     [ksp], [https://github.com/losvald/ksp])
AC_PREREQ([2.59])
AC_CONFIG_AUX_DIR([build-aux])
AM_INIT_AUTOMAKE([1.10 foreign -Wall no-define])
AC_PROG_CXX
AC_PROG_LIBTOOL
AC_CONFIG_HEADERS([config.h])
AC_CONFIG_FILES([Makefile src/Makefile test/Makefile])

AC_LANG(C++)
AC_LINK_IFELSE(
  [AC_LANG_PROGRAM([#include <gtest/gtest.h>],
    [])],
  [TEST_LIBS="$TEST_LIBS -lgtest"] [HAVE_GTEST=1],
  [AC_MSG_ERROR([libgtest is not installed.])])
AC_OUTPUT
