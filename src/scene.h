#ifndef SCENE_H
#define SCENE_H

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

#include <functional>
#include <list>

#include <SDL2/SDL.h>

#include "event.h"


class Client;

typedef std::function<void ()> InitJobFunc;

class Scene : public EventProcessor, public ServerInteractor {
protected:
    Client *pClient;
    std::list <InitJobFunc> mInitJobs;
public:
    Scene(Client *);
    virtual ~Scene(void) {};

    virtual void Render(void) = 0;
    virtual void Update(const float dt) = 0;

    virtual void Reset(void) {}

    const std::list <InitJobFunc> &GetInitJobs(void);
};

/**
 * thread safe
 */
class Initializer
{
private:
    SDL_mutex *mJobsMutex;
    std::list <InitJobFunc> mJobs;
    int nJobsDone;
public:
    Initializer(const std::list <InitJobFunc> &);
    ~Initializer(void);

    void InitAll(void);

    /**
     * 1.0f when completely done
     */
    float GetFractionDone(void);
};

#endif  // SCENE_H
