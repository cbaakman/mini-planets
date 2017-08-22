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
#include <cstring>

#include <SDL2/SDL.h>

#include "server.h"
#include "exception.h"
#include "log.h"


Server::Server(void):
    done(false)
{
}
Server::~Server(void)
{
}
bool Server::Run()
{
    ClientID clientID;
    DataPackage pkg;
    const char *error;
    Uint32 lastTicks, ticks;
    float dt;

    done = false;
    while (!done)
    {
        try
        {
            while (PollFromClient(clientID, pkg))
            {
                HandleClientData(clientID, pkg);
                free(pkg.data);
            }

            ticks = SDL_GetTicks();
            dt = float(ticks - lastTicks) / 1000;
            lastTicks = ticks;

            Update(dt);
        }
        catch (std::exception &e)
        {
            DisplayError(e.what());
        }
    }

    return true;
}
void Server::Update(const float dt)
{
}
void Server::HandleClientData(const ClientID &clientID, const DataPackage &pkg)
{
    switch (pkg.type)
    {
    case DATA_PING:
    {
        PingRequest *req = (PingRequest *)pkg.data;

        PingRequest res;
        res.bounces = req->bounces - 1;

        if (req->bounces > 0)
        {
            DataPackage pkgRes;
            pkgRes.type = DATA_PING;
            pkgRes.data = (void *)&res;
            pkgRes.size = sizeof(res);
            SendToClient(clientID, pkgRes);
        }
        break;
    }
    }
}
void Server::DisplayError(const std::string &error)
{
    fprintf(stderr, "server error: %s\n", error.c_str());
}
void Server::ShutDown(void)
{
    done = true;
}
InternalServer::InternalServer(void)
{
}
InternalServer::~InternalServer(void)
{
}
bool InternalServer::PollFromClient(ClientID &clientID, DataPackage &pkg)
{
    bool result;

    mFromClientMutex.lock();

    if (mDataFromClient.empty())
        result = false;
    else
    {
        // There's only one client in an internal server:
        clientID = 0;
        memcpy(&pkg, &mDataFromClient.front(), sizeof(pkg));
        mDataFromClient.pop();
        result = true;
    }

    mFromClientMutex.unlock();

    return result;
}
void InternalServer::SendToClient(const ClientID &id, const DataPackage &pkg)
{
    mToClientMutex.lock();

    DataPackage sendPkg = pkg;
    sendPkg.data = malloc(pkg.size);
    memcpy(sendPkg.data, pkg.data, pkg.size);

    // There's only one client in an internal server:
    mDataToClient.push(sendPkg);

    mToClientMutex.unlock();
}
void InternalServer::PushDataFromClient(const DataPackage &pkg)
{
    mFromClientMutex.lock();

    DataPackage storePkg = pkg;
    storePkg.data = malloc(pkg.size);
    memcpy(storePkg.data, pkg.data, pkg.size);

    // There's only one client in an internal server:
    mDataFromClient.push(storePkg);

    mFromClientMutex.unlock();
}
bool InternalServer::PopDataFromServer(DataPackage &pkg)
{
    // Not asking the client's identity, there is only one ;)

    bool result;

    mToClientMutex.lock();

    if (mDataToClient.empty())
    {
        result = false;
    }
    else
    {
        memcpy(&pkg, &mDataToClient.front(), sizeof(pkg));
        mDataToClient.pop();
        result = true;
    }

    mToClientMutex.unlock();

    return result;
}
