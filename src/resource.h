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

#ifndef RESOURCE_H
#define RESOURCE_H

#include <string>
#include <map>

#include <SDL2/SDL.h>
#include <libxml/tree.h>

#include "exception.h"
#include "font.h"
#include "texture.h"

void ListResourceFiles(const std::string dirPath,
                       std::list <std::string> &);

SDL_RWops *GetResourceIO(const std::string &path);

typedef std::map<std::string, std::string> ResourceParamsMap;

template <class R>
class Resource
{
private:
    R *pReferenced;
    int mRefCount;
    ResourceParamsMap mParams;
    std::string mName;
    SDL_threadID mThreadID;
public:
    Resource(const std::string &name,
             const ResourceParamsMap &params):
        mName(name),
        mParams(params),
        pReferenced(NULL),
        mRefCount(0),
        mThreadID(0)
    {
    }
    Resource(void): Resource("", ResourceParamsMap()) {}
    ~Resource(void)
    {
        Free(pReferenced);
    }

    void IncRef(void)
    {
        mRefCount++;
    }
    void DecRef(void)
    {
        mRefCount--;
        if (mRefCount <= 0)
        {
            Free(pReferenced);
            pReferenced = NULL;
        }
    }
    int GetRefCount(void) const { return mRefCount; }
    const std::string &GetName(void) const { return mName; }
    const ResourceParamsMap &GetParams(void) const { return mParams; }

    R *GetReferenced(void)
    {
        if (mThreadID != SDL_GetThreadID(NULL))
            throw FormatableException("resource referenced in wrong thread");

        return pReferenced;
    }
    void SetReferenced(R *p)
    {
        mThreadID = SDL_GetThreadID(NULL);

        pReferenced = p;
    }
};

/**
 * Works like std::shared_ptr, but also releases the resource.
 */
template <class R>
class ResourceReference
{
private:
    Resource<R> *pResource;
public:
    ResourceReference(void):
        pResource(NULL)
    {
    }
    ResourceReference(Resource<R> *p)
    {
        pResource = p;
        if (pResource)
            pResource->IncRef();
    }
    ResourceReference(const ResourceReference<R> &other)
    {
        pResource = other.pResource;
        if (pResource)
            pResource->IncRef();
    }
    ~ResourceReference(void)
    {
        if (pResource)
            pResource->DecRef();
    }
    ResourceReference<R> &operator=(const ResourceReference<R> other)
    {
        if (this != &other)
        {
            if (pResource)
                pResource->DecRef();

            pResource = other.pResource;
            if (pResource)
                pResource->IncRef();
        }
        return *this;
    }

    bool HasResource(void)
    {
        return pResource != NULL &&
               pResource->GetRefCount() > 0 &&
               pResource->GetReferenced() != NULL;
    }

    R *operator->(void) const
    {
        return pResource->GetReferenced();
    }
    R &operator*(void) const
    {
        return *(pResource->GetReferenced());
    }
    operator R *(void) const
    {
        return pResource->GetReferenced();
    }
};

/**
 * The loaded resources are only valid for the thread they were loaded in.
 */
class ResourceManager
{
private:
    std::map <std::string, Resource<xmlDoc>> mXmls;
    std::map <std::string, Resource<Font>> mFonts;
    std::map <std::string, Resource<Texture>> mTextures;
    std::map <std::string, Resource<CubeMap>> mCubeMaps;

    void AddResourceEntry(const std::string &name,
                          const std::string &type,
                          const ResourceParamsMap &);
public:
    void Init(void);

    ResourceReference<xmlDoc> GetXML(const std::string &name);
    ResourceReference<Font> GetFont(const std::string &name);
    ResourceReference<Texture> GetTexture(const std::string &name);
    ResourceReference<CubeMap> GetCubeMap(const std::string &name);
};

template <class R>
R *Load(ResourceManager *, const std::string &name,
        const ResourceParamsMap &);

template <class R>
void Free(R *);

#endif  // RESOURCE_H
