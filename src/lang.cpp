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

#include <errno.h>
#include <stdio.h>
#include <cstring>

#include "log.h"
#include "lang.h"
#include "exception.h"


void ParseLang(SDL_RWops *io, LanguageMap &langMap)
{
    char id[256],
         value[1024],
         c;
    int i, n;

    while (true)
    {
        while ((n = io->read(io, &c, 1, 1)) == 1 && isspace(c));

        if (n <= 0)
            break;

        i = 0;
        id[i] = c;

        while (io->read(io, &c, 1, 1) == 1 && c != '=')
        {
            i ++;
            id[i] = c;
        }
        id[i + 1] = NULL;

        if (c != '=')
            throw FormatableException("missing \'=\' on a line in the lang file");

        while (isspace(id[i]))
        {
            id[i] = NULL;
            i --;
        }

        if (strlen(id) <= 0)
            throw FormatableException("an id has length 0");

        while (io->read(io, &c, 1, 1) == 1 && isspace(c));

        i = 0;
        value[i] = c;

        while (io->read(io, &c, 1, 1) == 1 && c != '\n')
        {
            i ++;
            value[i] = c;
        }
        value[i + 1] = NULL;

        while (isspace(value[i]))
        {
            value[i] = NULL;
            i --;
        }

        if (strlen(value) <= 0)
            throw FormatableException("value of %s has length 0", id);

        langMap[id] = value;

        if (c != '\n')
            break;
    }
}

