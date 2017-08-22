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

#include <cstring>

#include "event.h"
#include "log.h"


bool EventProcessor::OnKeyBoundEvent(const KeyBinding keyb,
                                     const SDL_Event &event)
{
    switch (event.type)
    {
    case SDL_KEYDOWN:
        if (OnBoundKeyDown(keyb))
            return true;
    break;
    case SDL_MOUSEBUTTONDOWN:
        if (OnBoundKeyDown(keyb))
            return true;
    break;
    }

    return false;
}
bool EventProcessor::OnEvent(const SDL_Event &event)
{
    switch (event.type)
    {
    case SDL_QUIT:
        if (OnQuit(event.quit))
            return true;
    break;
    case SDL_KEYDOWN:
        if (OnKeyDown(event.key))
            return true;
    break;
    case SDL_MOUSEBUTTONDOWN:
        if (OnMouseButtonDown(event.button))
            return true;
    break;
    case SDL_MOUSEMOTION:
        if (OnMouseMotion(event.motion))
            return true;
    break;
    case SDL_MOUSEWHEEL:
        if (OnMouseWheel(event.wheel))
            return true;
    break;
    }
    return false;
}
bool ServerInteractor::OnServerData(const DataPackage &pkg)
{
    switch (pkg.type)
    {
    }
    return false;
}
