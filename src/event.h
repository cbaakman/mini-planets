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

#ifndef EVENT_H
#define EVENT_H

#include <SDL2/SDL.h>

#include "settings.h"
#include "server.h"


/**
 * Its functions return true if the event must be cancelled,
 * otherwise, the events are passed on to other listeneners.
 */
class EventProcessor
{
public:
    virtual bool OnEvent(const SDL_Event &);
    virtual bool OnKeyBoundEvent(const KeyBinding, const SDL_Event &);
protected:
    virtual bool OnMouseMotion(const SDL_MouseMotionEvent &) { return false; }
    virtual bool OnMouseWheel(const SDL_MouseWheelEvent &) { return false; }
    virtual bool OnQuit(const SDL_QuitEvent &) { return false; }
    virtual bool OnMouseButtonDown(const SDL_MouseButtonEvent &) { return false; }
    virtual bool OnKeyDown(const SDL_KeyboardEvent &) { return false; }

    virtual bool OnBoundKeyDown(const KeyBinding) { return false; }
};

/**
 * Its functions return true if the data must be cancelled,
 * otherwise, the data is passed on to other interactors.
 */
class ServerInteractor
{
public:
    virtual bool OnServerData(const DataPackage &);
protected:
};

#endif  // EVENT_H
