/**
 * sysio.hpp
 * This file declares and implements platform agnostic input output streams.
 */

#pragma once

void clear_screen();

#ifdef WIN32
// windows: to be implemented
void clear_screen() {}
#else
#ifdef LINUX
// linux: using curses
void clear_screen() {}
#else
// macos: using curses
#include <curses.h>
void clear_screen() { clear(); }
#endif
#endif