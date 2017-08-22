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

#ifndef CLIENT_H
#define CLIENT_H

#include <string>
#include <functional>
#include <list>
#include <vector>
#include <memory>

#include <SDL2/SDL.h>
#include <boost/thread.hpp>
#include "vec.h"
#include "settings.h"
#include "event.h"
#include "lang.h"
#include "server.h"
#include "resource.h"
#include "scene.h"


class ClientServerInterface {
public:
    virtual void SendData(const DataPackage &) = 0;
    virtual bool PollData(DataPackage &) = 0;
};


class ClientInternalServerInterface : public ClientServerInterface {
private:
    InternalServer *pServer;
public:
    ClientInternalServerInterface(InternalServer *);

    void SendData(const DataPackage &);
    bool PollData(DataPackage &);
};


class ClientRemoteServerInterface : public ClientServerInterface {
};

typedef std::function<bool (void)> EndConditionFunc;

class RenderLoop
{
private:
    Client *pClient;

    Scene *pCurrentScene;

    SDL_GLContext mGLContext;
    SDL_Window *pWindow;

    ClientServerInterface *pServerInterface;
private:
    void HandleEvent(const SDL_Event &);
    void HandleServerData(const DataPackage &);

    void InitGL(void);
    void FreeGL(void);
public:
    RenderLoop(Client *, SDL_Window *);
    ~RenderLoop(void);

    void SwitchScene(Scene *);
    void SetServerInterface(ClientServerInterface *);
    void SetWindow(SDL_Window *);

    void Run(EndConditionFunc);
};

class Client
{
private:
    bool bDone;

    SDL_Window *mMainWindow;
    ClientServerInterface *mServerInterface;
    RenderLoop *mMainLoop;

    // Only used when running an internal server:
    InternalServer *mInternalServer;
    boost::thread mInternalServerThread;

    boost::thread_specific_ptr<ResourceManager> mResourceManagerStorage;

    ControlSettings mControls;

    RenderSettings mRenderSettings;

    std::string mLanguageID;
    LanguageMap mLanguage;

    Scene *mPlayScene;
private:
    void LoadLanguage(void);

    void StartInternalServer(void);
    void StopInternalServer(void);

    void OpenWindow(const DisplaySettings &);
    void CloseWindow(void);

    void Init(void);
    void CleanUp(void);
    void RunMainLoop(void);

    std::string GetSettingsPath(void);
public:
    Client(void);
    ~Client(void);

    bool Run(void);
    void ShutDown(void);

    void StartSinglePlayer(void);
    //void ReturnToMain(void);

    /**
     * Careful not to spam this!
     * Don't use this in functions that get called every frame.
     * Use LOG_ERROR in log.h for that.
     */
    void MessageBoxError(const char *format, ...);

    void GetSettings(ClientSettings &) const;
    void ChangeDisplaySettings(const DisplaySettings &);
    void GetResolutionOptions(std::vector <ScreenResolution> &);

    void SetRenderSettings(const RenderSettings &);
    void GetRenderSettings(RenderSettings &);

    void SetKeyBinding(const KeyBinding, const KeyBindingValue);
    bool HasKeyBinding(const SDL_Event &, KeyBinding &);
    KeyBindingValue GetKeyBinding(const KeyBinding) const;
    bool KeyIsDown(const KeyBinding) const;

    ResourceManager *GetResourceManager(void);

    bool HasMouseFocus(void);

    RenderLoop *GetMainRenderLoop(void);

    void SetLanguage(const std::string &languageID);
    std::string GetLanguageString(const std::string &id);
    void GetLanguageOptions(std::list<std::string> &);

    /**
     * Returns NULL if not logged in.
     */
    ClientServerInterface *GetServerInterface(void);
};

#endif  // CLIENT_H
