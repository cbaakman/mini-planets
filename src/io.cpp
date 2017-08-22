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

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "io.h"
#include "exception.h"


void ListZipArchive(const char *archive,
                    const char *dirPathStr, std::list<std::string> &lOut)
{
    char fileName[1024];
    fs::path relativeDirPath(dirPathStr),
             filePath;

    // Let unzip open the archive:
    unzFile zip = unzOpen(archive);
    if (!zip)
        throw FileOpenException(archive, "don\'t exist or not valid");

    if (unzGoToFirstFile(zip) == UNZ_OK)
    {
        do
        {
            if (unzGetCurrentFileInfo(zip, NULL, fileName, sizeof(fileName), NULL, 0, NULL, 0) == UNZ_OK)
            {
                filePath = fs::path(fileName);
                if (filePath.parent_path().compare(relativeDirPath) == 0)
                    lOut.push_back(filePath.string());
            }
            else
                throw FormatableException("cannot get the filename for a file in %s", archive);
        }
        while (unzGoToNextFile(zip) == UNZ_OK);
    }

    unzClose(zip);
}

size_t NoWriteCallBack(SDL_RWops *context, const void *ptr, size_t size, size_t num)
{
    // Tell that we cannot write any bytes:
    return 0;
}

Sint64 NoSeekCallBack(SDL_RWops *context, Sint64 offset, int whence)
{
    // Tell that we cannot seek:
    return -1;
}
Sint64 ZipArchiveSizeCallBack(SDL_RWops *context)
{
    // Get the unzip pointer from the object:
    unzFile zip = (unzFile)context->hidden.unknown.data1;

    // Get the file info from unzip:
    unz_file_info info;

    int res = unzGetCurrentFileInfo(zip, &info, NULL, 0, NULL, 0, NULL, 0);
    if (res != UNZ_OK)
    {
        SDL_SetError("unzip: error getting current file info");
        return -1;
    }

    // Get the size part from info:
    return info.uncompressed_size;
}

size_t ZipArchiveReadCallBack(SDL_RWops *context, void *data, size_t size, size_t maxnum)
{
    // Get the unzip pointer from the object:
    unzFile zip = (unzFile)context->hidden.unknown.data1;

    // Read from the currently pointed file in the archive:
    int res = unzReadCurrentFile(zip, data, size*maxnum);

    // Return the number of objects read:
    return res / size;
}
int ZipArchiveCloseCallBack(SDL_RWops *context)
{
    // Get the unzip pointer from the object:
    unzFile zip = (unzFile)context->hidden.unknown.data1;

    // Call unzip's close procedures:
    unzCloseCurrentFile(zip);
    unzClose(zip);

    // Delete the SDL_RWops:
    SDL_FreeRW(context);

    return 0;
}
SDL_RWops *SDL_RWFromZipArchive(const char *_archive, const char *_entry)
{
    // Create an SDL_RWops:
    SDL_RWops *c=SDL_AllocRW();
    if (!c)
        return NULL;

    // Let unzip open the archive:
    unzFile zip = unzOpen(_archive);
    if (!zip)
    {
        SDL_SetError("failed to open %s", _archive);
        return NULL;
    }

    // Let unzip find the desired file in the archive:
    int result = unzLocateFile(zip, _entry, 1);

    if (result != UNZ_OK)
    {
        SDL_SetError("not found in %s: %s", _archive, _entry);
        unzClose(zip);

        return NULL;
    }

    // Open the desired file in the archive:
    unzOpenCurrentFile(zip);

    // Set the callbacks, for size, read and close.
    // unzip cannot seek nor write.
    c->size = ZipArchiveSizeCallBack;
    c->seek = NoSeekCallBack;
    c->read = ZipArchiveReadCallBack;
    c->write = NoWriteCallBack;
    c->close = ZipArchiveCloseCallBack;
    c->type = SDL_RWOPS_UNKNOWN;
    c->hidden.unknown.data1 = (voidp) zip;

    return c;
}
