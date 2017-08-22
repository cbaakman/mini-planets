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

#ifndef SERVER_H
#define SERVER_H

#include <map>
#include <set>
#include <cstdint>
#include <queue>
#include <string>

#include <SDL2/SDL.h>
#include <boost/thread/mutex.hpp>

#include "vec.h"

typedef int64_t ClientID;


enum DataType
{
    DATA_PING,
    DATA_POSITION_UPDATE,
    DATA_CHUNK_UPDATE,
    DATA_CHUNK_REQUEST
};

struct DataPackage
{
    uint8_t type;

    void *data;
    size_t size;
};

struct PingRequest
{
    uint8_t bounces;
};

#define SERVER_RUNERR_ID "server_runtime_error"

class Server
{
private:
    bool done;
private:
    void HandleClientData(const ClientID &, const DataPackage &);

    void Update(const float dt);
protected:
    /**
     * Must free the result with 'free'!
     */
    virtual bool PollFromClient(ClientID &, DataPackage &) = 0;

    /**
     * After this, the data can safely be deleted.
     */
    virtual void SendToClient(const ClientID &, const DataPackage &) = 0;
public:
    Server(void);
    virtual ~Server(void);

    bool Run(void);
    void ShutDown(void);

    void DisplayError(const std::string &);
};


class InternalServer : public Server
{
private:
    std::queue<DataPackage> mDataFromClient;
    std::queue<DataPackage> mDataToClient;

    boost::mutex mFromClientMutex,
                 mToClientMutex;
protected:
    bool PollFromClient(ClientID &, DataPackage &);
    void SendToClient(const ClientID &, const DataPackage &);
public:
    InternalServer(void);
    ~InternalServer(void);

    /*
     * After this, the data can safely be deleted.
     */
    void PushDataFromClient(const DataPackage &);

    /**
     * Must free the result with 'free'!
     */
    bool PopDataFromServer(DataPackage &);
};


class RemoteServer : public Server
{
};

#endif  // SERVER_H
