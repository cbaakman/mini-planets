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

#ifndef STR_H
#define STR_H

#include <string>
#include <list>
#include <cstddef>

/*
    Here, the PATH_SEPARATOR macro is used
    to join paths. Could also use boost for this:
    http://www.boost.org/doc/libs/1_53_0/libs/filesystem/doc/reference.html
 */
#ifdef _WIN32
    #define PATH_SEPARATOR '\\'

    #include <windows.h>

    std::string WindowsErrorString (const DWORD errorCode);
#else
    #define PATH_SEPARATOR '/'
#endif

typedef char32_t unicode_char; // these codes are backbards compatible with ascii chars

/**
 * Picks one unicode character from the utf-8 byte array and returns
 * a pointer to the utf-8 data after it.
 *
 * See also: https://nl.wikipedia.org/wiki/UTF-8
 */
const char *next_from_utf8 (const char *pBytes, unicode_char *out);

/**
 * Picks one unicode character from the utf-8 byte array and returns
 * a pointer to the utf-8 data before it.
 *
 * See also: https://nl.wikipedia.org/wiki/UTF-8
 */
const char *prev_from_utf8 (const char *pBytes, unicode_char *out);

/**
 * Tells the number of utf-8 characters in the given byte array.
 * Set 'end' to break out before a terminating null.
 */
std::size_t strlen_utf8 (const char *pBytes, const char *end = 0);

/**
 * returns a pointer in the byte array where the n'th utf-8
 * character is located. (starting from 0)
 */
const char *pos_utf8 (const char *pBytes, const std::size_t n);

bool isnewline (const int c);
bool emptyline (const char *line);

/**
 * Places a NULL character so that trailing whitespaces
 * are removed.
 */
void stripr (char *s);

/**
 * atof and scanf are locale dependent. ParseFloat isn't,
 * decimals always assumed to have dots here, never commas.
 *
 * :returns: the string after the parsed number on success,
 *           NULL on failure.
 */
const char *ParseFloat(const char *in, float *out);

/**
 * Like strcmp, but ignores case.
 */
int StrCaseCompare(const char *s1, const char *s2);

bool StartsWith(const char *s, const char *prefix);

/**
 * splits a string by given character.
 */
void split (const char *s, const char by, std::list<std::string> &out);

/**
 * Gives the start and end position of the word at pos in the string.
 */
void WordAt (const char *s, const int pos, int &start, int &end);

/**
 * Shows the input value's bits as a string of
 * '1's and '0's.
 * Can be used in print statements for debugging.
 * Output depends on endianness.
 */
template <typename Type>
std::string bitstr (const Type &value)
{
    const char *p = (const char *)&value;
    int i, j, n = sizeof (Type);
    bool bit;
    char byte;

    std::string s;
    for (i = 0; i < n; i++)
    {
        byte = p [i];
        for (j = 0; j < 8; j++)
        {
            bit = byte & (0b10000000 >> j);
            s += bit? '1' : '0';
        }
    }
    return s;
}
/**
 * Does not generate output for the terminating null!
 */
std::string bitstr (const std::string &);
std::string bitstr (const char *);

/**
 * Like bitstr, but doesn't work for non-numerical data.
 * The output is independent of endianness.
 */
template <typename Number>
std::string bitstr_noendian (const Number &value)
{
    int i, n = 8 * sizeof (Number);
    bool bit;

    std::string s;
    for (i = n - 1; i >= 0; i--)
    {
        bit = value & (0b00000001 << i);
        s += bit? '1' : '0';
    }
    return s;
}

#endif  // STR_H
