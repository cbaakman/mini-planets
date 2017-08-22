/* Copyright (C) 2017 Coos Baakman
  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.
  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:
  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#ifndef LOG_H
#define LOG_H

    // Meant for debugging:
    #ifdef DEBUG
        #include <stdio.h>

        #define LOG(msg, ...) { printf("log: "); printf(msg, ##__VA_ARGS__); printf("\n"); }
        #define LOG_ERROR(msg, ...) { fprintf(stderr, "error: "); fprintf(stderr, msg, ##__VA_ARGS__); fprintf(stderr, "\n"); }
    #else
        #define LOG(msg, ...) {}
        #define LOG_ERROR(msg, ...) {}
    #endif  // DEBUG

#endif  // LOG_H
