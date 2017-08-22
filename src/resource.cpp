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

#include <SDL2/SDL.h>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "resource.h"
#include "str.h"
#include "exception.h"
#include "io.h"
#include "xml.h"
#include "log.h"


#define RESOURCE_DIR_PATH "resources"
#define RESOURCE_ZIP_PATH "tropix.zip"
SDL_RWops *GetResourceIO(const std::string &path)
{
    SDL_RWops *io;

    #ifdef DEBUG
        fs::path filePath =
            fs::current_path() / RESOURCE_DIR_PATH / path;
        io = SDL_RWFromFile(filePath.string().c_str(), "rb");
    #else
        fs::path zipPath =
            fs::initial_path() / RESOURCE_ZIP_PATH;
        io = SDL_RWFromZipArchive(zipPath.string().c_str(), path.c_str());
    #endif

    if (io == NULL)
        throw FormatableException(SDL_GetError());

    return io;
}
void ListResourceFiles(const std::string dirName,
                       std::list <std::string> &lOut)
{
    #ifdef DEBUG
        fs::path dirPath =
             fs::current_path() / RESOURCE_DIR_PATH / dirName,
                                filePath,
                                diffPath,
                                tmpPath;
        fs::directory_iterator end_itr;
        for (fs::directory_iterator it(dirPath);
             it != end_itr; it++)
        {
            filePath = it->path();
            if (fs::is_regular_file(filePath))
            {
                // Make the path relative to resource dir:
                diffPath = "";
                tmpPath = filePath;
                while (tmpPath != dirPath)
                {
                    diffPath = tmpPath.filename() / diffPath;
                    tmpPath = tmpPath.parent_path();
                }

                lOut.push_back(diffPath.string());
            }
        }
    #else
        fs::path zipPath =
            fs::initial_path() / RESOURCE_ZIP_PATH;

        ListZipArchive(zipPath.string().c_str(), dirName.c_str(), lOut);
    #endif
}

void ResourceManager::AddResourceEntry(const std::string &name,
                                       const std::string &type,
                                       const ResourceParamsMap& params)
{
    if (StrCaseCompare(type.c_str(), "xml") == 0)
    {
        if (mXmls.find(name) != mXmls.end())
        {
            throw FormatableException(
                       "duplicate name: %s", name.c_str());
        }
        mXmls.emplace(name, Resource<xmlDoc>(name, params));
    }
    else if(StrCaseCompare(type.c_str(), "font") == 0)
    {
        if (mFonts.find(name) != mFonts.end())
        {
            throw FormatableException(
                       "duplicate name: %s", name.c_str());
        }

        mFonts.emplace(name, Resource<Font>(name, params));
    }
    else if(StrCaseCompare(type.c_str(), "texture") == 0)
    {
        if (mTextures.find(name) != mTextures.end())
        {
            throw FormatableException(
                       "duplicate name: %s", name.c_str());
        }
        mTextures.emplace(name, Resource<Texture>(name, params));
    }
    else if(StrCaseCompare(type.c_str(), "cubemap") == 0)
    {
        if (mCubeMaps.find(name) != mCubeMaps.end())
        {
            throw FormatableException(
                       "duplicate name: %s", name.c_str());
        }
        mCubeMaps.emplace(name, Resource<CubeMap>(name, params));
    }
    else
    {
        throw FormatableException(
                   "unknown resource type: %s", type.c_str());
    }
}
#define RESOURCES_XML_PATH "resources.xml"
void ResourceManager::Init()
{
    xmlChar *pAttrib;
    xmlDocPtr pDoc = NULL;
    xmlNodePtr pRoot;
    SDL_RWops *io = NULL;
    std::string name,
                type,
                id,
                value;
    try
    {
        io = GetResourceIO(RESOURCES_XML_PATH);

        pDoc = ParseXML(io);

        pRoot = xmlDocGetRootElement(pDoc);

        if (!pRoot)
        {
            throw FormatableException("no root element in resources doc");
        }


        if (StrCaseCompare((const char *)pRoot->name, "resources") != 0)
        {
            throw FormatableException(
                       "document of the wrong type, found \'%s\' instead of \'resources\'",
                       pRoot->name);
        }

        xmlNode *pChild = pRoot->children,
                *pParam;

        while (pChild)
        {
            if (StrCaseCompare((const char *)pChild->name, "resource") == 0) {

                pAttrib = xmlGetProp(pChild, (const xmlChar *)"name");
                if (!pAttrib)
                {
                    throw FormatableException(
                               "\'name\' is not set on resource tag, it\'s required");
                }
                name = std::string((const char *)pAttrib);
                xmlFree(pAttrib);

                pAttrib = xmlGetProp(pChild, (const xmlChar *)"type");
                if (!pAttrib)
                {
                    throw FormatableException(
                               "\'type\' is not set on resource tag, it\'s required");
                }
                type = std::string((const char *)pAttrib);
                xmlFree(pAttrib);

                ResourceParamsMap params;
                for (pParam = pChild->children; pParam; pParam = pParam->next)
                {
                    if (StrCaseCompare((const char *)pParam->name, "param") == 0)
                    {
                        pAttrib = xmlGetProp(pParam, (const xmlChar *)"id");
                        if (!pAttrib)
                        {
                            throw FormatableException(
                                       "in resource %s, \'id\' is not set on param tag, it\'s required",
                                   name.c_str());
                        }
                        id = std::string((const char *)pAttrib);
                        xmlFree(pAttrib);

                        pAttrib = xmlGetProp(pParam, (const xmlChar *)"value");
                        if (!pAttrib)
                        {
                            throw FormatableException(
                                       "in resource %s, \'value\' is not set on param tag, it\'s required",
                                        name.c_str());
                        }
                        value = std::string((const char *)pAttrib);
                        xmlFree(pAttrib);

                        if (params.find(id) != params.end())
                        {
                            throw FormatableException(
                                        "duplicate param id in resource %s: %s",
                                        name.c_str(), id.c_str());
                        }
                        params[id] = value;
                    }
                }

                AddResourceEntry(name, type, params);
            }
            pChild = pChild->next;
        }
    }
    catch (std::exception &e)
    {
        if (io)
            SDL_RWclose(io);
        xmlFreeDoc(pDoc);
        throw ParsingException(RESOURCES_XML_PATH, e.what());
    }

    SDL_RWclose(io);
    xmlFreeDoc(pDoc);
}
template <class R>
Resource<R> *FindAndLoadIn(ResourceManager *pResourceManager,
                           std::map <std::string, Resource<R>> &resources,
                           const std::string &name)
{
    if (resources.find(name) != resources.end())
    {
        Resource<R> *pResource = &(resources.at(name));

        if (pResource->GetRefCount() <= 0)
        {
            R *pReferenced = Load<R>(pResourceManager, pResource->GetName(),
                                     pResource->GetParams());

            pResource->SetReferenced(pReferenced);
        }

        return pResource;
    }
    else  // not found in 'resources'
    {
        throw FormatableException(
                   "no such resource defined: %s", name.c_str());
    }
}
template <>
xmlDoc *Load(ResourceManager *pResourceManager, const std::string &name,
             const ResourceParamsMap &params)
{
    std::string path;
    if (params.find("path") == params.end())
    {
        throw FormatableException(
                   "missing path for xml resource %s",
                   name.c_str());
    }
    path = params.at("path");

    SDL_RWops *io = GetResourceIO(path);

    xmlDoc *pDoc;
    try
    {
        pDoc = ParseXML(io);
    }
    catch (std::exception &e)
    {
        SDL_RWclose(io);

        throw ParsingException(path.c_str(), e.what());
    }

    SDL_RWclose(io);

    return pDoc;
}
template <>
void Free(xmlDoc *pDoc)
{
    if (pDoc)
        xmlFreeDoc(pDoc);
}
ResourceReference<xmlDoc> ResourceManager::GetXML(const std::string &name)
{
    Resource<xmlDoc> *res = FindAndLoadIn(this, mXmls, name);
    ResourceReference<xmlDoc> ref(res);
    return ref;
}
bool ParseColorRGBA(const char *p, float *rgba)
{
    int i;

    while (*p != NULL && isspace(*p)) p++;

    for (i = 0; i < 4; i++)
    {
        p = ParseFloat(p, &rgba[i]);
        while (*p != NULL && isspace(*p)) p++;
        if (*p == NULL)
            break;

        if (*p != ',')
            break;
        p++;

        while (*p != NULL && isspace(*p)) p++;
    }
    return i >= 4;
}
template <>
Font *Load(ResourceManager *pResourceManager, const std::string &name,
           const ResourceParamsMap &params)
{
    std::string svgName;
    int fontSize = 12;
    float lineWidth = 0.0f,
          stroke[4] = {0.0f, 0.0f, 0.0f, 0.0f},
          fill[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    if (params.find("input-svg") == params.end())
    {
        throw FormatableException(
                   "missing input-svg for font resource %s",
                   name.c_str());
    }
    svgName = params.at("input-svg");

    ResourceReference<xmlDoc> refDoc = pResourceManager->GetXML(svgName);

    if (params.find("size") != params.end())
    {
        fontSize = atoi(params.at("size").c_str());
    }
    if (params.find("line-width") != params.end())
    {
        ParseFloat(params.at("line-width").c_str(), &lineWidth);
    }
    if (params.find("stroke") != params.end())
    {
        ParseColorRGBA(params.at("stroke").c_str(), stroke);
    }
    if (params.find("fill") != params.end())
    {
        ParseColorRGBA(params.at("fill").c_str(), fill);
    }

    Font *pFont = new Font;
    try
    {
        MakeSVGFont(refDoc, fontSize,
                     [&](cairo_t *cr)
                     {
                         cairo_set_source_rgba(cr, (double)fill[0],
                                                   (double)fill[1],
                                                   (double)fill[2],
                                                   (double)fill[3]);
                         cairo_fill_preserve(cr);

                         cairo_set_line_width(cr, lineWidth);
                         cairo_set_source_rgba(cr, (double)stroke[0],
                                                   (double)stroke[1],
                                                   (double)stroke[2],
                                                   (double)stroke[3]);
                         cairo_stroke(cr);

                         return true;
                     },
                     pFont);
    }
    catch (std::exception &e)
    {
        delete pFont;

        throw ParsingException(svgName, e.what());
    }

    return pFont;
}
template <>
void Free(Font *pFont)
{
    if (pFont)
    {
        ClearFont(pFont);
        delete pFont;
    }
}
ResourceReference<Font> ResourceManager::GetFont(const std::string &name)
{
    return ResourceReference<Font>(FindAndLoadIn(this, mFonts, name));
}
template <>
Texture *Load(ResourceManager *pResourceManager, const std::string &name,
              const ResourceParamsMap &params)
{
    std::string path;
    if (params.find("path") == params.end())
    {
        throw FormatableException(
                   "missing path for texture resource %s",
                   name.c_str());
    }
    path = params.at("path");

    SDL_RWops *io = GetResourceIO(path);

    Texture *pTexture = new Texture;
    LoadPNG(io, pTexture);

    SDL_RWclose(io);

    return pTexture;
}
template <>
void Free(Texture *pTexture)
{
    if (pTexture)
    {
        glDeleteTextures(1, &(pTexture->tex));
        delete pTexture;
    }
}
ResourceReference<Texture> ResourceManager::GetTexture(const std::string &name)
{
    ResourceReference<Texture> ref(FindAndLoadIn(this, mTextures, name));
    return ref;
}
template <>
CubeMap *Load(ResourceManager *pResourceManager, const std::string &name,
              const ResourceParamsMap &params)
{
    std::string xneg_path,
                xpos_path,
                yneg_path,
                ypos_path,
                zneg_path,
                zpos_path;

    if (params.find("xneg_path") == params.end())
    {
        throw FormatableException(
                   "missing xneg_path for cubemap resource %s",
                   name.c_str());
    }
    xneg_path = params.at("xneg_path");

    if (params.find("xpos_path") == params.end())
    {
        throw FormatableException(
                   "missing xpos_path for cubemap resource %s",
                   name.c_str());
    }
    xpos_path = params.at("xpos_path");

    if (params.find("yneg_path") == params.end())
    {
        throw FormatableException(
                   "missing yneg_path for cubemap resource %s",
                   name.c_str());
    }
    yneg_path = params.at("yneg_path");

    if (params.find("ypos_path") == params.end())
    {
        throw FormatableException(
                   "missing ypos_path for cubemap resource %s",
                   name.c_str());
    }
    ypos_path = params.at("ypos_path");

    if (params.find("zneg_path") == params.end())
    {
        throw FormatableException(
                   "missing zneg_path for cubemap resource %s",
                   name.c_str());
    }
    zneg_path = params.at("zneg_path");

    if (params.find("zpos_path") == params.end())
    {
        throw FormatableException(
                   "missing zpos_path for cubemap resource %s",
                   name.c_str());
    }
    zpos_path = params.at("zpos_path");

    SDL_RWops *pXneg = GetResourceIO(xneg_path);
    SDL_RWops *pXpos = GetResourceIO(xpos_path);
    SDL_RWops *pYneg = GetResourceIO(yneg_path);
    SDL_RWops *pYpos = GetResourceIO(ypos_path);
    SDL_RWops *pZneg = GetResourceIO(zneg_path);
    SDL_RWops *pZpos = GetResourceIO(zpos_path);

    CubeMap *pCubeMap = new CubeMap;
    LoadPNGs(pXneg, pXpos, pYneg, pYpos, pZneg, pZpos, pCubeMap);

    SDL_RWclose(pXneg);
    SDL_RWclose(pXpos);
    SDL_RWclose(pYneg);
    SDL_RWclose(pYpos);
    SDL_RWclose(pZneg);
    SDL_RWclose(pZpos);

    return pCubeMap;
}
template <>
void Free(CubeMap *pCubeMap)
{
    if (pCubeMap)
    {
        glDeleteTextures(1, &(pCubeMap->tex));
        delete pCubeMap;
    }
}
ResourceReference<CubeMap> ResourceManager::GetCubeMap(const std::string &name)
{
    ResourceReference<CubeMap> ref(FindAndLoadIn(this, mCubeMaps, name));
    return ref;
}
