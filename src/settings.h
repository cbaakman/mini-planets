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

#ifndef SETTINGS_H
#define SETTINGS_H

#include <string>
#include <map>

#include <SDL2/SDL.h>

#define MIN_SCREEN_WIDTH 800
#define MIN_SCREEN_HEIGHT 600

struct ScreenResolution
{
    int width,
        height;
};

bool operator==(const ScreenResolution &, const ScreenResolution &);

struct DisplaySettings
{
    ScreenResolution resolution;
    bool fullscreen;
};

bool operator==(const DisplaySettings &, const DisplaySettings &);
bool operator!=(const DisplaySettings &, const DisplaySettings &);

struct RenderSettings
{
    int renderDistance;
};

enum KeyBinding
{
    KEYB_NONE = 0,
    KEYB_JUMP,
    KEYB_DUCK,
    KEYB_GOFORWARD,
    KEYB_GOBACK,
    KEYB_GOLEFT,
    KEYB_GORIGHT,
    KEYB_INVENTORY,
    KEYB_MENU,
    KEYB_CAMERAMODE
};

enum KeyBindingValueType
{
    KEYTYPE_KEYBOARD,
    KEYTYPE_MOUSEBUTTON
};

struct KeyBindingValueKeyboard
{
    KeyBindingValueType type;
    SDL_Keycode code;
};

struct KeyBindingValueMouseButton
{
    KeyBindingValueType type;
    Uint8 button;
};

union KeyBindingValue
{
    KeyBindingValueType type;
    KeyBindingValueKeyboard key;
    KeyBindingValueMouseButton mouse;
};

std::string GetKeyBindingValueName(const KeyBindingValue &);

std::string GetKeyBindingDescription(const KeyBinding);

typedef std::map <KeyBinding, KeyBindingValue> ControlSettings;

struct ClientSettings
{
    ControlSettings controls;
    DisplaySettings display;
    RenderSettings render;
    std::string language;
    int renderDistance;
};

void GetDefaultClientSettings(ClientSettings &);

void LoadClientSettings(const std::string &path, ClientSettings &);
void SaveClientSettings(const std::string &path, const ClientSettings &);

#endif  // SETTINGS_H
