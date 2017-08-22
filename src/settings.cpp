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
#include <errno.h>
#include <cstring>
#include <sstream>
#include <ctype.h>

#include "exception.h"
#include "settings.h"
#include "str.h"
#include "log.h"

KeyBindingValue MakeKeyBoardValue(SDL_Keycode code)
{
    KeyBindingValue value;
    value.type = KEYTYPE_KEYBOARD;
    value.key.code = code;

    return value;
}

void GetDefaultClientSettings(ClientSettings &settings)
{
    settings.render.renderDistance = 200;
    settings.display.fullscreen = false;
    settings.display.resolution.width = 800;
    settings.display.resolution.height = 600;

    settings.controls[KEYB_JUMP] = MakeKeyBoardValue(SDLK_SPACE);
    settings.controls[KEYB_DUCK] = MakeKeyBoardValue(SDLK_LSHIFT);
    settings.controls[KEYB_GOFORWARD] = MakeKeyBoardValue(SDLK_w);
    settings.controls[KEYB_GOBACK] = MakeKeyBoardValue(SDLK_s);
    settings.controls[KEYB_GOLEFT] = MakeKeyBoardValue(SDLK_a);
    settings.controls[KEYB_GORIGHT] = MakeKeyBoardValue(SDLK_d);
    settings.controls[KEYB_INVENTORY] = MakeKeyBoardValue(SDLK_e);
    settings.controls[KEYB_MENU] = MakeKeyBoardValue(SDLK_ESCAPE);
    settings.controls[KEYB_CAMERAMODE] = MakeKeyBoardValue(SDLK_m);

    settings.language = "english";
}

bool operator==(const ScreenResolution &res1, const ScreenResolution &res2)
{
    return res1.width == res2.width && res1.height == res2.height;
}
bool operator==(const DisplaySettings &s1, const DisplaySettings &s2)
{
    return s1.resolution == s2.resolution && s1.fullscreen == s2.fullscreen;
}
bool operator!=(const DisplaySettings &s1, const DisplaySettings &s2)
{
    return !(s1 == s2);
}
std::string GetKeyBindingValueName(const KeyBindingValue &value)
{
    if (value.type == KEYTYPE_KEYBOARD)
    {
        const char *sdlName = SDL_GetKeyName(value.key.code);

        std::ostringstream os;
        os << "key.";

        if (strlen(sdlName) <= 0)
            os << "unnamed";
        else
            for (const char *p = sdlName; *p != NULL; p++)
            {
                if (*p == ' ')
                    os << "-";
                else if (*p == '=')  // the parser's separator
                    os << "equals";
                else if (*p != '(' && *p != ')')
                    os << std::string(1, tolower(*p));
            }

        os << ".name";

        return os.str();
    }
    else if (value.type == KEYTYPE_MOUSEBUTTON)
    {
        switch (value.mouse.button)
        {
        case SDL_BUTTON_LEFT:
            return "mousebutton.left.name";
        case SDL_BUTTON_MIDDLE:
            return "mousebutton.middle.name";
        case SDL_BUTTON_RIGHT:
            return "mousebutton.right.name";
        case SDL_BUTTON_X1:
            return "mousebutton.x1.name";
        case SDL_BUTTON_X2:
            return "mousebutton.x2.name";
        default:
            throw FormatableException("Unknown mouse button 0x%x", value.mouse.button);
        }
    }
    else
        throw FormatableException("Unknown key binding value type");
}
std::string GetKeyBindingName(const KeyBinding keyBinding)
{
    switch(keyBinding)
    {
    case KEYB_JUMP:
        return "jump";
    case KEYB_DUCK:
        return "duck";
    case KEYB_GOFORWARD:
        return "forward";
    case KEYB_GOBACK:
        return "back";
    case KEYB_GOLEFT:
        return "left";
    case KEYB_GORIGHT:
        return "right";
    case KEYB_INVENTORY:
        return "inventory";
    case KEYB_MENU:
        return "menu";
    case KEYB_CAMERAMODE:
        return "camera_mode";
    default:
        throw FormatableException("Unnamed key binding: %d", keyBinding);
    }
}
std::string GetKeyBindingDescription(const KeyBinding keyBinding)
{
    return "menu.option.keybind." + GetKeyBindingName(keyBinding);
}
KeyBinding ParseKeyBinding(const std::string name)
{
    if (name == GetKeyBindingName(KEYB_JUMP))
        return KEYB_JUMP;
    else if (name == GetKeyBindingName(KEYB_DUCK))
        return KEYB_DUCK;
    else if (name == GetKeyBindingName(KEYB_GOFORWARD))
        return KEYB_GOFORWARD;
    else if (name == GetKeyBindingName(KEYB_GOBACK))
        return KEYB_GOBACK;
    else if (name == GetKeyBindingName(KEYB_GOLEFT))
        return KEYB_GOLEFT;
    else if (name == GetKeyBindingName(KEYB_GORIGHT))
        return KEYB_GORIGHT;
    else if (name == GetKeyBindingName(KEYB_INVENTORY))
        return KEYB_INVENTORY;
    else if (name == GetKeyBindingName(KEYB_MENU))
        return KEYB_MENU;
    else if (name == GetKeyBindingName(KEYB_CAMERAMODE))
        return KEYB_CAMERAMODE;
    else
        throw FormatableException("Unknown key binding: %s", name.c_str());
}
void ParseControlsSetting(const char *id, ControlSettings &controls, const char *value)
{
    KeyBindingValue kbValue;

    KeyBinding keyBinding = ParseKeyBinding(id);

    if (StartsWith(value, "key."))
    {
        kbValue.type = KEYTYPE_KEYBOARD;
        kbValue.key.code = strtol(value + 4, NULL, 16);
    }
    else if (StartsWith(value, "mouse."))
    {
        kbValue.type = KEYTYPE_MOUSEBUTTON;
        kbValue.mouse.button = strtol(value + 6, NULL, 16);
    }
    else
        throw FormatableException("unknown key binding value type: %s", value);

    controls[keyBinding] = kbValue;
}
void WriteControlsSettings(FILE *pFile, const char *prefix, const ControlSettings &controls)
{
    for (auto pair : controls)
    {
        KeyBinding keyb = pair.first;
        KeyBindingValue kbValue = pair.second;

        if (kbValue.type == KEYTYPE_KEYBOARD)
            fprintf(pFile, "%s%s: key.%x\n", prefix,
                    GetKeyBindingName(keyb).c_str(),
                    kbValue.key.code);
        else if (kbValue.type == KEYTYPE_MOUSEBUTTON)
            fprintf(pFile, "%s%s: mouse.%x\n", prefix,
                    GetKeyBindingName(keyb).c_str(),
                    kbValue.mouse.button);
        else
            throw FormatableException("unwritable key value");
    }
}
#define SCREENWIDTH_ID "width"
#define SCREENHEIGHT_ID "height"
void ParseResolutionSetting(const char *id, ScreenResolution &resolution, const char *value)
{
    if (strcmp(id, SCREENWIDTH_ID) == 0)
    {
        resolution.width = atoi(value);
    }
    else if (strcmp(id, SCREENHEIGHT_ID) == 0)
    {
        resolution.height = atoi(value);
    }
    else
        throw FormatableException("unknown resolution setting: %s", id);
}
void WriteResolutionSettings(FILE *pFile, const char *prefix, const ScreenResolution &resolution)
{
    fprintf(pFile, "%s%s: %d\n", prefix, SCREENWIDTH_ID, resolution.width);
    fprintf(pFile, "%s%s: %d\n", prefix, SCREENHEIGHT_ID, resolution.height);
}
#define RESOLUTION_PREFIX "resolution."
#define FULLSCREEN_ID "fullscreen"
void ParseDisplaySetting(const char *id, DisplaySettings &settings, const char *value)
{
    if (StartsWith(id, RESOLUTION_PREFIX))
    {
        ParseResolutionSetting(id + strlen(RESOLUTION_PREFIX), settings.resolution, value);
    }
    else if (strcmp(id, FULLSCREEN_ID) == 0)
    {
        settings.fullscreen = atoi(value);
    }
    else
        throw FormatableException("unknown display setting: %s", id);
}
void WriteDisplaySettings(FILE *pFile, const char *prefix, const DisplaySettings &settings)
{
    char id[256];

    strcpy(id, prefix);
    strcat(id, RESOLUTION_PREFIX);
    WriteResolutionSettings(pFile, id, settings.resolution);

    fprintf(pFile, "%s%s: %d\n", prefix, FULLSCREEN_ID, settings.fullscreen);
}
#define RENDERDIST_ID "renderdistance"
void ParseRenderSetting(const char *id, RenderSettings &settings, const char *value)
{
    if (strcmp(id, RENDERDIST_ID) == 0)
    {
        settings.renderDistance = atoi(value);
    }
    else
        throw FormatableException("unknown render setting: %s", id);
}
void WriteRenderSettings(FILE *pFile, const char *prefix, const RenderSettings &settings)
{
    fprintf(pFile, "%s%s: %d\n", prefix, RENDERDIST_ID, settings.renderDistance);
}
#define RENDER_PREFIX "render."
#define DISPLAY_PREFIX "display."
#define CONTROLS_PREFIX "controls."
#define LANGUAGE_SETTING "language"
void ParseLanguageSetting(std::string &language, const std::string &value)
{
    language = value;
}
void WriteLanguageSetting(FILE *pFile, const std::string &language)
{
    fprintf(pFile, "%s: %s\n", LANGUAGE_SETTING, language.c_str());
}
void LoadClientSettings(const std::string &path, ClientSettings &settings)
{
    FILE *pFile;
    char id[256],
         value[256],
         c;
    int i;

    if ((pFile = fopen(path.c_str(), "r")) == NULL)
        throw FileOpenException(path, strerror(errno));

    GetDefaultClientSettings(settings);

    try
    {
        while (true)
        {
            while (isspace((c = fgetc(pFile))));

            if (feof(pFile))
                break;

            i = 0;
            id[i] = c;
            i ++;

            while ((c = fgetc(pFile)) != ':')
            {
                id[i] = c;
                i ++;
                if (feof(pFile))
                     throw FormatableException("end of file while reading id");
            }

            while (isspace(id[i]))
                i --;
            id[i] = NULL;

            while (isspace((c = fgetc(pFile))));

            if (feof(pFile))
                throw FormatableException("end of file while looking for value of %s", id);

            i = 0;
            value[i] = c;
            i ++;

            while ((c = fgetc(pFile)) != '\n' && !feof(pFile))
            {
                value[i] = c;
                i ++;
            }

            while (isspace(value[i]))
                i --;
            value[i] = NULL;

            if (StartsWith(id, DISPLAY_PREFIX))
            {
                ParseDisplaySetting(id + strlen(DISPLAY_PREFIX), settings.display, value);
            }
            else if (StartsWith(id, RENDER_PREFIX))
            {
                ParseRenderSetting(id + strlen(RENDER_PREFIX), settings.render, value);
            }
            else if (StartsWith(id, CONTROLS_PREFIX))
            {
                ParseControlsSetting(id + strlen(CONTROLS_PREFIX), settings.controls, value);
            }
            else if (strcmp(id, LANGUAGE_SETTING) == 0)
            {
                ParseLanguageSetting(settings.language, value);
            }
            else
                throw FormatableException("unknown setting: %s", id);
        }
    }
    catch (std::exception &e)
    {
        fclose(pFile);
        throw ParsingException(path, e.what());
    }

    fclose(pFile);
}
void SaveClientSettings(const std::string &path, const ClientSettings &settings)
{
    FILE *pFile;

    if ((pFile = fopen(path.c_str(), "w")) == NULL)
        throw FileOpenException(path, strerror(errno));

    WriteControlsSettings(pFile, CONTROLS_PREFIX, settings.controls);
    WriteDisplaySettings(pFile, DISPLAY_PREFIX, settings.display);
    WriteLanguageSetting(pFile, settings.language);
    WriteRenderSettings(pFile, RENDER_PREFIX, settings.render);

    fclose(pFile);
}
