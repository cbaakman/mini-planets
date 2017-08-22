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

#include <stdio.h>
#include <string>
#include <sstream>
#include <algorithm>

#include <GL/glew.h>
#include <GL/gl.h>
#include <boost/thread/thread.hpp>

#include "client.h"
#include "log.h"
#include "matrix.h"
#include "play.h"
#include "resource.h"
#include "exception.h"
#include "lang.h"


void CleanupResourceManager(void *p)
{
    ResourceManager *pResourceManager = (ResourceManager *)p;
    delete pResourceManager;
}
/**
 * Resource manager should only be called in the beginning
 * while initializing data structures. Not during rendering.
 * It might throw an exception!
 */
ResourceManager *Client::GetResourceManager(void)
{
    ResourceManager *p = mResourceManagerStorage.get();
    if (p == NULL)
    {
        mResourceManagerStorage.reset(new ResourceManager());
        mResourceManagerStorage.get()->Init();
        return mResourceManagerStorage.get();
    }
    else
        return (ResourceManager *)p;
}
Client::Client(void):
    mMainWindow(NULL),
    bDone(false),
    mRenderSettings({0}),
    mInternalServer(NULL),
    mServerInterface(NULL),
    mMainLoop(NULL),
    mPlayScene(NULL)
{
}
Client::~Client(void)
{
    CleanUp();
}
void Client::CleanUp(void)
{
    delete mPlayScene;

    if (mInternalServer)
        StopInternalServer();

    delete mMainLoop;

    CloseWindow();

    SDL_Quit();
}
std::string Client::GetSettingsPath(void)
{
    char *pBasePath = SDL_GetBasePath();
    std::string basePath(pBasePath);
    SDL_free(pBasePath);

    return basePath + "mini-planets.ini";
}
void Client::Init(void)
{
    int error = SDL_Init(SDL_INIT_EVERYTHING);
    if (error != 0)
    {
        throw FormatableException("Unable to initialize SDL: %s", SDL_GetError());
    }

    LOG("load settings");
    ClientSettings settings;
    GetDefaultClientSettings(settings);
    try
    {
        LoadClientSettings(GetSettingsPath(), settings);
    }
    catch (std::exception &e)
    {
        /*
            If something is wrong with the settings file,
            then the client should recover and write a new one.
         */

        LOG_ERROR("Cannot load settings: %s", e.what());

        SaveClientSettings(GetSettingsPath(), settings);
    }

    mRenderSettings = settings.render;

    mControls = settings.controls;

    mLanguageID = settings.language;
    LoadLanguage();

    OpenWindow(settings.display);

    mMainLoop = new RenderLoop(this, mMainWindow);

    StartSinglePlayer();
}
void Client::GetLanguageOptions(std::list<std::string> &lOut)
{
    int l;

    std::list<std::string> files;
    ListResourceFiles("lang", files);
    for (const std::string &name : files)
    {
        l = name.size();
        if (name.substr(l - 5) == ".lang")
            lOut.push_back(name.substr(0, l - 5));
    }
}
void Client::LoadLanguage(void)
{
    std::string filename = "lang/" + mLanguageID + ".lang";

    mLanguage.clear();

    SDL_RWops *io = GetResourceIO(filename);
    try
    {
        ParseLang(io, mLanguage);
    }
    catch (std::exception &e)
    {
        SDL_RWclose(io);
        throw ParsingException(filename, e.what());
    }
    SDL_RWclose(io);
}
void Client::SetLanguage(const std::string &languageID)
{
    mLanguageID = languageID;
    LoadLanguage();

    ClientSettings newSettings;
    GetSettings(newSettings);
    try
    {
        SaveClientSettings(GetSettingsPath(), newSettings);
    }
    catch(std::exception &e)
    {
        LOG_ERROR("Cannot save settings: %s", e.what());
    }
}
void Client::SetRenderSettings(const RenderSettings &settings)
{
    mRenderSettings = settings;

    ClientSettings newSettings;
    GetSettings(newSettings);
    try
    {
        SaveClientSettings(GetSettingsPath(), newSettings);
    }
    catch(std::exception &e)
    {
        LOG_ERROR("Cannot save settings: %s", e.what());
    }
}
void Client::GetRenderSettings(RenderSettings &settings)
{
    settings = mRenderSettings;
}
std::string Client::GetLanguageString(const std::string &id)
{
    if (mLanguage.find(id) != mLanguage.end())
        return mLanguage.at(id);
    else
        return id;
}
RenderLoop::RenderLoop(Client *pCl, SDL_Window *pWin):
    pClient(pCl),
    pWindow(pWin),
    pCurrentScene(NULL),
    pServerInterface(NULL)
{
    InitGL();
}
RenderLoop::~RenderLoop(void)
{
    FreeGL();
}
void RenderLoop::SwitchScene(Scene *pScene)
{
    pCurrentScene = pScene;
}
void RenderLoop::InitGL(void)
{
    mGLContext = SDL_GL_CreateContext(pWindow);
    if (!mGLContext)
    {
        throw FormatableException("Failed to create GL context: %s",
                                  SDL_GetError());
    }

    GLenum err = glewInit();
    if (GLEW_OK != err)
    {
        throw FormatableException("glewInit failed: %s",
                                  glewGetErrorString(err));
    }

    if (!GLEW_VERSION_3_2)
    {
        throw FormatableException("OpenGL version 3.2 is not enabled.");
    }
}
void RenderLoop::FreeGL(void)
{
    if (mGLContext)
        SDL_GL_DeleteContext(mGLContext);
    mGLContext = NULL;
}
void RenderLoop::Run(EndConditionFunc endConditionFunc)
{
    GLenum glError;
    DataPackage pkg;
    SDL_Event event;
    Uint32 lastTicks, ticks;
    float dt;

    lastTicks = SDL_GetTicks();

    while (!endConditionFunc())
    {
        /* It is dangerous to delete the scene pointer in the
           middle of the loop. So do it here.
         */

        while (SDL_PollEvent(&event))
        {
            HandleEvent(event);
        }

        if (pServerInterface)
        {
            while (pServerInterface->PollData(pkg))
            {
                HandleServerData(pkg);
                free(pkg.data);
            }
        }

        ticks = SDL_GetTicks();
        dt = float(ticks - lastTicks) / 1000;
        lastTicks = ticks;

        if (pCurrentScene)
            pCurrentScene->Update(dt);

        // Do this before rendering:
        if (SDL_GL_MakeCurrent(pWindow, mGLContext) < 0)
        {
            LOG_ERROR("SDL_GL_MakeCurrent: %s", SDL_GetError());
        }

        if (pCurrentScene)
        {
            pCurrentScene->Render();

            while ((glError = glGetError()) != GL_NO_ERROR)
            {
                LOG_ERROR("%s while rendering the current scene",
                          GLErrorString(glError).c_str());
            }
        }
        else
        {
            // At least clear the buffer
            glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
            glClear(GL_COLOR_BUFFER_BIT);
        }

        SDL_GL_SwapWindow(pWindow);
    }
}
void RenderLoop::HandleServerData(const DataPackage &pkg)
{
    switch (pkg.type)
    {
    case DATA_PING:
    {
        PingRequest *req = (PingRequest *)pkg.data;

        PingRequest res;
        res.bounces = req->bounces - 1;

        if (pServerInterface && req->bounces > 0)
        {
            DataPackage resPkg;
            resPkg.type = DATA_PING;
            resPkg.data = (void *)&res;
            resPkg.size = sizeof(res);
            pServerInterface->SendData(resPkg);
        }
        break;
    }
    default:
        pCurrentScene->OnServerData(pkg);
        break;
    }
}
void RenderLoop::HandleEvent(const SDL_Event &event)
{
    const Uint32 windowID = SDL_GetWindowID(pWindow);

    if (event.type == SDL_QUIT && !pCurrentScene)
    {
        pClient->ShutDown();
        return;
    }

    if (event.type == SDL_WINDOWEVENT && event.window.windowID != windowID ||
        (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP) && event.key.windowID != windowID ||
        event.type == SDL_MOUSEWHEEL && event.wheel.windowID != windowID ||
        (event.type == SDL_MOUSEBUTTONDOWN || event.type == SDL_MOUSEBUTTONUP) && event.wheel.windowID != windowID ||
        event.type == SDL_MOUSEMOTION && event.motion.windowID != windowID)
    {
        return;
    }

    if (pCurrentScene)
    {
        if (!pCurrentScene->OnEvent(event))
        {
            KeyBinding keyb;
            if (pClient->HasKeyBinding(event, keyb))
                pCurrentScene->OnKeyBoundEvent(keyb, event);
        }
    }
}
void RenderLoop::SetWindow(SDL_Window *pWin)
{
    pWindow = pWin;
}
void RenderLoop::SetServerInterface(ClientServerInterface *pIFace)
{
    pServerInterface = pIFace;
}
void Client::ShutDown(void)
{
    bDone = true;
}
void Client::OpenWindow(const DisplaySettings &settings)
{
    // Set the openGL parameters we want:
    SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, 8);

    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, 1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, 4);

    Uint32 flags = SDL_WINDOW_SHOWN | SDL_WINDOW_OPENGL;
    if (settings.fullscreen)
        flags |= SDL_WINDOW_FULLSCREEN | SDL_WINDOW_INPUT_GRABBED;

    mMainWindow = SDL_CreateWindow("Mini Planets",
                                   SDL_WINDOWPOS_UNDEFINED,
                                   SDL_WINDOWPOS_UNDEFINED,
                                   settings.resolution.width,
                                   settings.resolution.height,
                                   flags);
    if (!mMainWindow)
    {
        throw FormatableException("SDL_CreateWindow failed: %s",
                                  SDL_GetError());
    }
}
void Client::CloseWindow(void)
{
    if (mMainWindow)
        SDL_DestroyWindow(mMainWindow);
    mMainWindow = NULL;
}
void Client::GetSettings(ClientSettings &settings) const
{
    if (mMainWindow == NULL)
    {
        LOG_ERROR("Cannot get display settings, because there is no window");
    }

    SDL_GL_GetDrawableSize(mMainWindow,
                           &settings.display.resolution.width,
                           &settings.display.resolution.height);

    Uint32 flags = SDL_GetWindowFlags(mMainWindow);
    settings.display.fullscreen = flags & (SDL_WINDOW_FULLSCREEN | SDL_WINDOW_FULLSCREEN_DESKTOP);

    settings.controls = mControls;
    settings.render = mRenderSettings;

    settings.language = mLanguageID;
}
void Client::ChangeDisplaySettings(const DisplaySettings &settings)
{
    SDL_GL_MakeCurrent(mMainWindow, NULL);

    CloseWindow();
    OpenWindow(settings);

    mMainLoop->SetWindow(mMainWindow);

    ClientSettings newSettings;
    GetSettings(newSettings);
    try
    {
        SaveClientSettings(GetSettingsPath(), newSettings);
    }
    catch(std::exception &e)
    {
        LOG_ERROR("Cannot save settings: %s", e.what());
    }
}
void Client::GetResolutionOptions(std::vector <ScreenResolution> &v)
{
    ScreenResolution resolution;
    SDL_DisplayMode mode;
    int i,
        n = SDL_GetNumDisplayModes(0);
    if (n < 1)
    {
        throw FormatableException("SDL_GetNumDisplayModes: %s", SDL_GetError());
    }

    for (i = 0; i < n; ++i)
    {
        if (SDL_GetDisplayMode(0, i, &mode) != 0)
        {
            throw FormatableException("SDL_GetDisplayMode: %s", SDL_GetError());
        }
        resolution = {mode.w, mode.h};
        // Only collect modes of minimal size and don't allow doubles:
        if (resolution.width >= MIN_SCREEN_WIDTH &&
            resolution.height >= MIN_SCREEN_HEIGHT &&
            std::find(v.begin(), v.end(), resolution) == v.end())
            v.push_back(resolution);
    }
    // Better to have this in order of small to large.
    std::reverse(v.begin(), v.end());
}
bool Client::HasKeyBinding(const SDL_Event &event, KeyBinding &keyb)
{
    KeyBindingValue value;

    if (event.type == SDL_MOUSEBUTTONDOWN || event.type == SDL_MOUSEBUTTONUP)
    {
        for (auto pair : mControls)
        {
            keyb = pair.first;
            value = pair.second;

            if (value.type == KEYTYPE_MOUSEBUTTON &&
                value.mouse.button == event.button.button)
                return true;
        }
    }
    else if (event.type == SDL_KEYDOWN || event.type == SDL_KEYUP)
    {
        for (auto pair : mControls)
        {
            keyb = pair.first;
            value = pair.second;

            if (value.type == KEYTYPE_KEYBOARD &&
                value.key.code == event.key.keysym.sym)
                return true;
        }
    }

    return false;
}
void Client::SetKeyBinding(const KeyBinding keyb, const KeyBindingValue value)
{
    mControls[keyb] = value;

    ClientSettings newSettings;
    GetSettings(newSettings);
    try
    {
        SaveClientSettings(GetSettingsPath(), newSettings);
    }
    catch(std::exception &e)
    {
        LOG_ERROR("Cannot save settings: %s", e.what());
    }
}
KeyBindingValue Client::GetKeyBinding(const KeyBinding keyb) const
{
    return mControls.at(keyb);
}
bool Client::KeyIsDown(const KeyBinding keyb) const
{
    KeyBindingValue value = GetKeyBinding(keyb);

    if (value.type == KEYTYPE_KEYBOARD)
    {
        const Uint8 *keys = SDL_GetKeyboardState(NULL);
        return keys[SDL_GetScancodeFromKey(value.key.code)];
    }
    else if (value.type == KEYTYPE_MOUSEBUTTON)
        return SDL_GetMouseState(NULL, NULL) & SDL_BUTTON(value.mouse.button);

    return false;
}
void Client::StartInternalServer(void)
{
    mInternalServer = new InternalServer;

    mInternalServerThread = boost::thread(
        [this]()
        {
            mInternalServer->Run();
        }
    );

    mServerInterface = new ClientInternalServerInterface(mInternalServer);

    mMainLoop->SetServerInterface(mServerInterface);
}
ClientServerInterface *Client::GetServerInterface(void)
{
    return mServerInterface;
}
void Client::StopInternalServer(void)
{
    mMainLoop->SetServerInterface(NULL);

    delete mServerInterface;
    mServerInterface = NULL;

    mInternalServer->ShutDown();

    mInternalServerThread.join();

    delete mInternalServer;
    mInternalServer = NULL;
}
void Client::StartSinglePlayer(void)
{
    LOG("starting internal server");

    try
    {
        StartInternalServer();
    }
    catch (std::exception &e)
    {
        LOG_ERROR("Failed to start internal server: %s", e.what());
    }

    LOG("initializing play scene");

    try
    {
        mPlayScene = new PlayScene(this);

        Initializer initializer(mPlayScene->GetInitJobs());
        initializer.InitAll();

        mMainLoop->SwitchScene(mPlayScene);
    }
    catch (std::exception &e)
    {
        LOG_ERROR("Failed to init scene: %s", e.what());

        delete mPlayScene;
        mPlayScene = NULL;

        StopInternalServer();

        return;
    }
}
/* void Client::ReturnToMain(void)
{
    LOG("initializing main menu");

    mMainLoop->SwitchScene(mMenuScene,
        [this]()
        {
            delete mPlayScene;
            mPlayScene = NULL;

            if (mInternalServer)
            {
                LOG("stopping internal server");
                StopInternalServer();
            }
        }
    );
    mMenuScene->Reset();
} */
ClientInternalServerInterface::ClientInternalServerInterface(InternalServer *pSrv):
    pServer(pSrv)
{
}
void ClientInternalServerInterface::SendData(const DataPackage &pkg)
{
    pServer->PushDataFromClient(pkg);
}
bool ClientInternalServerInterface::PollData(DataPackage &pkg)
{
    return pServer->PopDataFromServer(pkg);
}
void Client::MessageBoxError(const char *format, ...)
{
    va_list args;
    char buf[2048];

    va_start(args, format);

    int res = vsnprintf(buf, sizeof(buf), format, args);

    va_end(args);

    if (res > 0)
    {
        if (SDL_ShowSimpleMessageBox(SDL_MESSAGEBOX_ERROR,
                                     "Error",
                                     buf,
                                     mMainWindow) < 0)
        {
            LOG_ERROR("Cannot show messagebox: %s", SDL_GetError());
            LOG_ERROR("message was: %s", buf);
        }
    }
}
bool Client::Run(void)
{
    LOG("initilizing client");

    try
    {
        Init();
    }
    catch (std::exception &e)
    {
        MessageBoxError("Init failed, %s", e.what());
        return false;
    }

    LOG("running client main loop");

    mMainLoop->Run(
        [this]()
        {
            return bDone;
        }
    );

    LOG("client ended successfully");

    return true;
};
bool Client::HasMouseFocus(void)
{
    return mMainWindow == SDL_GetMouseFocus();
}
RenderLoop *Client::GetMainRenderLoop(void)
{
    return mMainLoop;
}
