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

#include <cctype>
#include <cstdlib>
#include <cstring>
#include <math.h>

#include "log.h"
#include "str.h"

int count_successive_left_1bits (const char byte)
{
    int n = 0;
    while (n < 8 && (byte & (0b0000000010000000 >> n)))

        n ++;

    return n;
}
const char *next_from_utf8 (const char *pBytes, unicode_char *out)
{
    int n_bytes, i;

    /*
        The first bits of the first byte determine the length
        of the utf-8 character. The number of bytes equals the
        number of uninterrupted 1 bits at the beginning.
        The remaining first byte bits are considered coding.
        ???????? : 0 bytes
        1??????? : 1 bytes
        11?????? : 2 bytes
        110????? : stop, take 6 coding bits and assume 2 byte code!
          ^^^^^^
     */

    n_bytes = count_successive_left_1bits (pBytes [0]);

    if (n_bytes > 0 && n_bytes < 8)
    {
         // get the remaining first byte bits:

        *out = pBytes [0] & (0b0000000011111111 >> n_bytes);
    }
    else // assume ascii
    {
        *out = pBytes [0];
        return pBytes + 1;
    }

    /*
        Complement the unicode identifier from the remaining encoding bits.
        All but the first UTF-8 byte must start in 10.., so six coding
        bits per byte:
                        .. 10?????? 10?????? 10?????? ..
     */
    for (i = 1; i < n_bytes; i++)
    {
        if ((pBytes [i] & 0b11000000) != 0b10000000)
        {
            LOG_ERROR("utf-8 byte %d not starting in 10.. !\n", i + 1);
        }

        *out = (*out << 6) | (pBytes [i] & 0b00111111);
    }

    return pBytes + n_bytes; // move to the next utf-8 character pointer
}
const char *prev_from_utf8 (const char *pBytes, unicode_char *out)
{
    int n_bytes = 0;
    bool begin_found = false;
    char mask, byte;

    *out = 0;
    while (!begin_found)
    {
        // Take one byte:
        n_bytes ++;
        byte = *(pBytes - n_bytes);

        // is it 10??????
        begin_found = (byte & 0b11000000) == 0b10000000;

        if (begin_found)
            mask = 0b00111111; // takes rightmost 6 bits
        else
            mask = 0b0000000011111111 >> n_bytes; // takes coding bits from 1st byte

        *out |= unicode_char (byte & mask) << (6 * n_bytes);
    }

    if (n_bytes <= 1 ||
        count_successive_left_1bits (byte) != n_bytes)
        // assume ascii
    {
        *out = *(pBytes - 1);

        return pBytes - 1;
    }

    return pBytes - n_bytes;
}
const char *pos_utf8 (const char *pBytes, const std::size_t n)
{
    std::size_t i = 0;
    unicode_char ch;
    while (i < n)
    {
        pBytes = next_from_utf8 (pBytes, &ch);
        i ++;
    }
    return pBytes;
}
std::size_t strlen_utf8 (const char *pBytes, const char *end)
{
    std::size_t n = 0;
    unicode_char ch;
    while (*pBytes)
    {
        pBytes = next_from_utf8 (pBytes, &ch);
        n ++;
        if (end and pBytes >= end)
            return n;
    }
    return n;
}
const char *ParseFloat(const char *in, float *out)
{
    float f = 10.0f;
    int digit, ndigit = 0;

    // start from zero
    *out = 0;

    const char *p = in;
    while (*p)
    {
        if (isdigit (*p))
        {
            digit = (*p - '0');

            if (f > 1.0f) // left from period
            {
                *out *= f;
                *out += digit;
            }
            else // right from period, decimal
            {
                *out += f * digit;
                f *= 0.1f;
            }
            ndigit++;
        }
        else if (tolower (*p) == 'e')
        {
            // exponent

            // if no digits precede the exponent, assume 1
            if (ndigit <= 0)
                *out = 1.0f;

            p++;
            if (*p == '+') // '+' just means positive power, default
                p++; // skip it, don't give it to atoi

            int e = atoi (p);

            *out = *out * pow(10, e);

            // read past exponent number
            if (*p == '-')
                p++;

            while (isdigit(*p))
                p++;

            return p;
        }
        else if (*p == '.')
        {
            // expect decimal digits after this

            f = 0.1f;
        }
        else if (*p == '-')
        {
            // negative number

            float v;
            p = ParseFloat(p + 1, &v);

            *out = -v;

            return p;
        }
        else
        {
            // To assume success, must have read at least one digit

            if (ndigit > 0)
                return p;
            else
                return NULL;
        }
        p++;
    }

    return p;
}
bool isnewline (const int c)
{
    return (c=='\n' || c=='\r');
}
bool emptyline (const char *line)
{
    while (*line)
    {
        if (!isspace (*line))
            return false;

        line ++;
    }
    return true;
}
void stripr(char *s)
{
    // Start on the back and move left:

    int n = strlen (s);
    while (true)
    {
        n --;
        if (isspace (s[n]))
        {
            s[n] = NULL;
        }
        else break;
    }
}
bool StartsWith(const char *s, const char *prefix)
{
    int i;
    for (i = 0; prefix[i] != NULL; i++)
        if (s[i] != prefix[i])
            return false;

    return true;
}
int StrCaseCompare(const char *s1, const char *s2)
{
    int i = 0, n = 0;

    while (s1[i] && s2[i])
    {

        n = toupper(s1[i]) - toupper(s2[i]);

        if(n != 0)
            return n;

        i++;
    }

    if(s1[i] && !s2[i])
        return 1;
    else if(!s1[i] && s2[i])
        return -1;
    else
        return 0;
}
void split(const char *s, const char b, std::list<std::string> &out)
{
    std::string str = "";
    while (*s)
    {
        if (*s == b)
        {
            if (str.size() > 0)
                out.push_back(str);

            str = "";
        }
        else
            str += *s;
        s++;
    }
    if (str.size() > 0)
        out.push_back(str);
}

void WordAt (const char *s, const int pos, int &start, int &end)
{
    bool bSpace = isspace (s[pos]);

    start = pos;
    while (start > 0)
    {
        if (isspace (s [start - 1]) != bSpace)
            break;
        start--;
    }

    end = pos;
    while (s[end] && isspace (s [end]) == bSpace)
        end++;
}
std::string bitstr (const std::string &str)
{
    return bitstr (str.c_str());
}
std::string bitstr (const char *str)
{
    int i, j;
    bool bit;

    std::string s;
    for (i = 0; str [i]; i++)
    {
        for (j = 0; j < 8; j++)
        {
            bit = str [i] & (0b10000000 >> j);
            s += bit? '1' : '0';
        }
    }
    return s;
}
