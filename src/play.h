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

#ifndef PLAY_H
#define PLAY_H

#include <time.h>
#include <vector>
#include <list>

#include "vec.h"
#include "client.h"
#include "collision.h"
#include "sphere.h"

struct TerrainVertex
{
    vec3 p, n;
    vec2 t;
};

struct TerrainTriangle
{
    VertexIndex i[3];
};

enum CameraMode
{
    CAMERA_TOP,
    CAMERA_FIRSTPERSON
};

class PlayScene : public Scene
{
private:
    CameraMode mPrevCameraMode,
               mCameraMode;
    float inCameraMode;

    GLfloat mCameraTopDist;

    vec3 mPlayerPos,
         mPlayerForwardDir,
         mPlayerVelocity;
    GLfloat mPlayerPitch;
    bool bPlayerOnGround;

    std::vector<TerrainVertex> mTerrainVertices;
    std::vector<TerrainTriangle> mTerrainTriangles;
    vec3 mPlanetMassCenter;

    ResourceReference<Texture> rGrassTexture;
private:
    void GetPlanetTriangles(std::list<Triangle> &);

    void ToggleCameraMode(void);

    std::tuple<vec3, vec3, vec3> GetCameraParams(const CameraMode mode);
public:
    PlayScene(Client *);
    ~PlayScene(void);

    void Update(const float dt);
    void Render(void);
protected:
    bool OnEvent(const SDL_Event &);

    bool OnMouseMotion(const SDL_MouseMotionEvent &);
    bool OnMouseWheel(const SDL_MouseWheelEvent &);
    bool OnBoundKeyDown(const KeyBinding);
    bool OnQuit(const SDL_QuitEvent &);
};

#endif  // PLAY_H
