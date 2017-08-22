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

#include "scene.h"
#include "exception.h"


Scene::Scene(Client *pCl):
    pClient(pCl)
{
}
const std::list<InitJobFunc> &Scene::GetInitJobs(void)
{
    return mInitJobs;
}
Initializer::Initializer(const std::list <InitJobFunc> &jobs):
    mJobs(jobs.begin(), jobs.end()),
    nJobsDone(0)
{
    mJobsMutex = SDL_CreateMutex();
    if (!mJobsMutex)
        throw FormatableException("Cannot make initializer mutex: %s", SDL_GetError());
}
Initializer::~Initializer(void)
{
    SDL_DestroyMutex(mJobsMutex);
}
void Initializer::InitAll(void)
{
    for (InitJobFunc init : mJobs)
    {
        init();

        if (SDL_LockMutex(mJobsMutex) == 0)
        {
            nJobsDone ++;

            SDL_UnlockMutex(mJobsMutex);
        }
        else
            throw FormatableException("Cannot lock initializer mutex: %s", SDL_GetError());
    }
}
float Initializer::GetFractionDone(void)
{
    if (SDL_LockMutex(mJobsMutex) == 0)
    {
        return float(nJobsDone) / mJobs.size();

        SDL_UnlockMutex(mJobsMutex);
    }
    else
        throw FormatableException("Cannot lock initializer mutex: %s", SDL_GetError());
}
