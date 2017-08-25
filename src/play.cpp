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

#include <time.h>
#include <math.h>
#include <algorithm>
#include <map>

#include "play.h"
#include "matrix.h"
#include "exception.h"
#include "log.h"
#include "noise.h"
#include "shader.h"


const char terrainVertexShader[] = R"shader(
    #version 330 core
    layout (location = 0) in vec3 pos;
    layout (location = 13) in vec3 n;

    out vec3 Normal;
    out vec3 FaceDirection;

    uniform mat4 projection;
    uniform mat4 view;
    uniform vec3 center;

    void main()
    {
        Normal = n;
        FaceDirection = pos - center;

        gl_Position = projection * view * vec4(pos, 1.0);
    }
)shader",
           terrainFragmentShader[] = R"shader(
    #version 330 core
    out vec4 FragColor;

    in vec3 Normal;
    in vec3 FaceDirection;

    uniform sampler2D tex;

    const vec3 lightDir = normalize(vec3(1.0f, 1.0f, 1.0f)),

               cubefaceDirections[6] = vec3[6]
               (
                    vec3(-1.0, 0.0, 0.0),
                    vec3(1.0, 0.0, 0.0),
                    vec3(0.0, -1.0, 0.0),
                    vec3(0.0, 1.0, 0.0),
                    vec3(0.0, 0.0, -1.0),
                    vec3(0.0, 0.0, 1.0)
               ),

               cubefaceUpDirs[6] = vec3[6]
               (
                    vec3(0.0, -1.0, 0.0),
                    vec3(0.0, 1.0, 0.0),
                    vec3(0.0, 0.0, 1.0),
                    vec3(0.0, 0.0, -1.0),
                    vec3(-1.0, 0.0, 0.0),
                    vec3(1.0, 0.0, 0.0)
               );

    #define PI 3.14159265358979323846264338327

    /*
       Calculate the texture coordinates based on their angles
       with the x, y, z axes.

       Weight the texture coordinates. The bigger the angle, the
       less weight.
     */

    vec2 GetTexCoords(vec3 dir, vec3 faceDir, vec3 upDir)
    {
        float phi, theta, x, y, z;

        y = dot(upDir, dir);
        z = dot(faceDir, dir);
        x = dot(cross(upDir, faceDir), dir);

        phi = atan(x / z);
        theta = atan(y / z);

        return vec2(
            0.5 + phi / PI,
            0.5 + theta / PI
        );
    }

    float GetTexWeight(vec3 dir, vec3 faceDir, vec3 upDir)
    {
        float angle = acos(dot(dir, faceDir));

        return 1.0 - 2 * clamp(angle, 0.0, PI / 2) / PI;
    }

    void main()
    {
        vec3 dir = normalize(FaceDirection);

        int i;
        float sum = 0.0,
              w;
        vec4 color = vec4(0.0, 0.0, 0.0, 0.0);
        for (i = 0; i < 6; i++)
        {
            w = GetTexWeight(dir, cubefaceDirections[i], cubefaceUpDirs[i]);
            sum += w;
            color += w * texture(tex, GetTexCoords(dir, cubefaceDirections[i], cubefaceUpDirs[i]));
        }
        color /= sum;

        float ambient = 0.3;

        float lit = ambient + (1.0 - ambient) * max(0.0, dot(normalize(Normal), lightDir));
        vec4 shadecolor = vec4(0.0, 0.0, 0.5, 1.0);

        FragColor = lit * color + (1.0 - lit) * shadecolor;
    }
 )shader";

template <typename T>
struct AverageBuilder
{
    int n;
    T sum;

    void Add(const T &t)
    {
        sum += t;
        n ++;
    }
    T Result(void) const
    {
        return sum / n;
    }
};

#define PLAYER_EYE_Y 0.7f
#define PLAYER_FEET_Y -1.0f

PlayScene::PlayScene(Client *pCl):
    Scene(pCl),
    mPrevCameraMode(CAMERA_FIRSTPERSON), mCameraMode(CAMERA_FIRSTPERSON), inCameraMode(1.0f),
    mCameraTopDist(10.0f),
    mPlayerPos(0.0f, 0.0f, 30.0f),
    mPlayerForwardDir(0.0f, 1.0f, 0.0f),
    mPlayerPitch(-1.0f)
{
    PerlinNoiseGenerator<3> perlin(time(NULL));

    std::vector<AverageBuilder<vec3>> averageNormals;
    IterIcoSphere(4,
        [this, &averageNormals, &perlin](const VertexIndex i, const vec3 &p)
        {
            int j;
            TerrainVertex v;

            GLfloat r = 10.0f + 0.5f * perlin.Noise(p * 2);
            v.p = p * r;

            mTerrainVertices.push_back(v);
            averageNormals.push_back({0, {0.0f, 0.0f, 0.0f}});
        },
        [this, &averageNormals](const VertexIndex i0, const VertexIndex i1, const VertexIndex i2)
        {
            mTerrainTriangles.push_back({i0, i1, i2});

            vec3 n0 = Cross(mTerrainVertices[i0].p - mTerrainVertices[i1].p,
                            mTerrainVertices[i0].p - mTerrainVertices[i2].p).Unit(),
                 n1 = Cross(mTerrainVertices[i1].p - mTerrainVertices[i2].p,
                            mTerrainVertices[i1].p - mTerrainVertices[i0].p).Unit(),
                 n2 = Cross(mTerrainVertices[i2].p - mTerrainVertices[i0].p,
                            mTerrainVertices[i2].p - mTerrainVertices[i1].p).Unit();
            averageNormals[i0].Add(n0);
            averageNormals[i1].Add(n1);
            averageNormals[i2].Add(n2);
        }
    );
    int i;
    for (i = 0; i < mTerrainVertices.size(); i++)
        mTerrainVertices[i].n = averageNormals[i].Result();

    mPlanetMassCenter = vec3(0.0f, 0.0f, 0.0f);

    mInitJobs.push_back(
        [this]()
        {
            rGrassTexture = pClient->GetResourceManager()->GetTexture("grass");
        }
    );

    mInitJobs.push_back(
        [this]()
        {
            mTerrainShaderProgram = CreateShaderProgram(GL_VERTEX_SHADER, terrainVertexShader,
                                                        GL_FRAGMENT_SHADER, terrainFragmentShader);
        }
    );
}
PlayScene::~PlayScene(void)
{
    glDeleteProgram(mTerrainShaderProgram);
}
void PlayScene::GetPlanetTriangles(std::list<Triangle> &triangles)
{
    int i, j;
    for (i = 0; i < mTerrainTriangles.size(); i++)
    {
        Triangle t;

        for (j = 0; j < 3; j++)
        {
            t.p[j] = mTerrainVertices[mTerrainTriangles[i].i[j]].p;
        }

        triangles.push_back(t);
    }
}
vec3 RotateToPlane(const vec3 &horizontalMovement, const vec3 &n)
{
    float length = horizontalMovement.Length();

    vec3 dir = PlaneProjection(horizontalMovement, {n, 0.0}).Unit();

    return length * dir;
}
#define GRAVITY 10.0f
#define FEET_ACC 40.0f
#define STATIC_FRICTION_COEFF 1.0f
#define KINETIC_FRICTION_COEFF 0.8f
void PlayScene::Update(const float dt)
{
    bool hit;
    Triangle tr;
    vec3 groundUnderPlayer,
         playerFeetPos;
    std::list<ColliderP> playerColliders;

    std::list<Triangle> collTriangles;
    GetPlanetTriangles(collTriangles);

    vec3 down = (mPlanetMassCenter - mPlayerPos).Unit(),
         up = -down,
         forward = PlaneProjection(mPlayerForwardDir, {up, 0.0f}),
         back = -forward,
         right = Cross(up, back),
         left = -right;

    playerColliders.push_back(new SphereCollider(0.5f * up, 0.5f));
    playerColliders.push_back(new SphereCollider((PLAYER_FEET_Y + 0.1f) * up, 0.1f));

    vec3 accGravity = GRAVITY * down,
         accNormal = vec3(0.0f, 0.0f, 0.0f),
         accFriction = vec3(0.0f, 0.0f, 0.0f),
         accDrag = vec3(0.0f, 0.0f, 0.0f),
         accFeet = vec3(0.0f, 0.0f, 0.0f);

    // OnGround test:
    playerFeetPos = mPlayerPos + up * PLAYER_FEET_Y;
    std::tie(hit, tr, groundUnderPlayer) =
            CollisionTraceBeam(playerFeetPos,
                               playerFeetPos + down, collTriangles);

    vec3 canMovePlayerDown = CollisionClosestBump(mPlayerPos,
                                                  mPlayerPos + down,
                                                  playerColliders,
                                                  collTriangles);

    // Hitting the ground and not moving away from it?
    bPlayerOnGround = (mPlayerPos - canMovePlayerDown).Length() < 0.005f &&
                      Dot(up, mPlayerVelocity) <= 0.0f;


    accDrag = -0.05f * mPlayerVelocity.Length2() * mPlayerVelocity.Unit();

    if (bPlayerOnGround)
    {
        if (pClient->KeyIsDown(KEYB_GOFORWARD))
            accFeet += FEET_ACC * forward;
        else if (pClient->KeyIsDown(KEYB_GOBACK))
            accFeet += FEET_ACC * back;

        if (pClient->KeyIsDown(KEYB_GOLEFT))
            accFeet += FEET_ACC * left;
        else if (pClient->KeyIsDown(KEYB_GORIGHT))
            accFeet += FEET_ACC * right;

        accFeet = RotateToPlane(accFeet, tr.GetPlane().n);

        if (pClient->KeyIsDown(KEYB_JUMP))
        {
            bPlayerOnGround = false;
            mPlayerVelocity += 5.0f * tr.GetPlane().n; // independent of dt !
        }
        else
        {
            accNormal = GRAVITY * tr.GetPlane ().n;

            // sum of forces that friction must counteract:
            vec3 a_applied = accGravity + accNormal + accFeet;

            if (a_applied.Length () > STATIC_FRICTION_COEFF * GRAVITY)

                accFriction = -KINETIC_FRICTION_COEFF * GRAVITY * a_applied.Unit();
            else
                accFriction = -a_applied;

            accDrag = -10.0f * mPlayerVelocity.Length () * mPlayerVelocity.Unit();
        }
    }

    mPlayerVelocity += (accGravity + accNormal + accFeet + accDrag + accFriction) * dt;

    vec3 targetPlayerPos = mPlayerPos + mPlayerVelocity * dt;

    mPlayerPos = CollisionMove(mPlayerPos, targetPlayerPos,
                              playerColliders,
                              collTriangles);

    // Try to keep it on the ground after one step.
    if (bPlayerOnGround)
        mPlayerPos = CollisionClosestBump(mPlayerPos + 0.1f * up,
                                          mPlayerPos + 0.1f * down,
                                          playerColliders,
                                          collTriangles);

    // Update forward dir to new angle:
    mPlayerForwardDir = PlaneProjection(
                            mPlayerForwardDir,
                            {(mPlayerPos - mPlanetMassCenter).Unit(), 0.0f}
                        ).Unit();

    inCameraMode = std::min(1.0f, inCameraMode + 2.0f * dt);

    for (ColliderP pCollider: playerColliders)
        delete pCollider;
}
std::tuple<vec3, vec3, vec3> PlayScene::GetCameraParams(const CameraMode mode)
{
    switch (mode)
    {
    case CAMERA_FIRSTPERSON:
    {
        vec3 down = (mPlanetMassCenter - mPlayerPos).Unit(),
             up = -down,
             viewForward = mPlayerForwardDir,
             right = Cross(up, -viewForward);

        // Apply pitch to view direction:
        viewForward = MatRotAxis(right, mPlayerPitch) * viewForward;

        return std::make_tuple(mPlayerPos, mPlayerPos + viewForward, up);
    }
    case CAMERA_TOP:
    {
        vec3 up = (mPlayerPos - mPlanetMassCenter).Unit();

        return std::make_tuple(mPlayerPos + up * mCameraTopDist, mPlayerPos, mPlayerForwardDir);
    }
    }
}
#define VIEW_ANGLE 45.0f
#define NEAR_VIEW 0.1f
#define FAR_VIEW 1000.0f
const GLfloat lightPos[] = {1.0f, 1.0f, 1.0f, 0.0f};
const GLfloat diffuse[] = {1.0f, 1.0f, 1.0f, 1.0f};
const GLfloat ambient[] = {0.0f, 0.0f, 1.0f, 1.0f};
void PlayScene::Render(void)
{
    GLint loc;
    int i, j, k;
    TerrainVertex v;
    vec3 cameraPos, cameraLookat, cameraUp,
         cameraPosPrev, cameraLookatPrev, cameraUpPrev;
    float outCameraMode = 1.0f - inCameraMode;

    ClientSettings settings;
    pClient->GetSettings(settings);

    glUseProgram(mTerrainShaderProgram);

    //glMatrixMode(GL_PROJECTION);
    matrix4 matPerspec = MatPerspec(VIEW_ANGLE,
                                  (GLfloat)settings.display.resolution.width / (GLfloat)settings.display.resolution.height,
                                  NEAR_VIEW, FAR_VIEW);
    //glLoadMatrixf(&matPerspec);
    glViewport(0, 0, settings.display.resolution.width,
                     settings.display.resolution.height);
    loc = glGetUniformLocation(mTerrainShaderProgram, "projection");
    if (loc == -1)
        throw GLException("getting projection matrix location", glGetError());
    glUniformMatrix4fv(loc, 1, GL_FALSE, &matPerspec);

    //glMatrixMode(GL_MODELVIEW);
    std::tie(cameraPosPrev, cameraLookatPrev, cameraUpPrev) = GetCameraParams(mPrevCameraMode);
    std::tie(cameraPos, cameraLookat, cameraUp) = GetCameraParams(mCameraMode);
    matrix4 matLook = MatLookAt(cameraPos * inCameraMode + cameraPosPrev * outCameraMode,
                                cameraLookat * inCameraMode + cameraLookatPrev * outCameraMode,
                                cameraUp  * inCameraMode + cameraUpPrev * outCameraMode);
    //glLoadMatrixf(&matLook);
    loc = glGetUniformLocation(mTerrainShaderProgram, "view");
    if (loc == -1)
        throw GLException("getting modelview matrix location", glGetError());
    glUniformMatrix4fv(loc, 1, GL_FALSE, &matLook);

    loc = glGetUniformLocation(mTerrainShaderProgram, "center");
    if (loc == -1)
        throw GLException("getting center location", glGetError());
    glUniform3fv(loc, 1, &mPlanetMassCenter);

    glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
    glClearDepth(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glLightfv(GL_LIGHT0, GL_POSITION, lightPos);

    glEnable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, rGrassTexture->tex);

    glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse);
    glMaterialfv(GL_FRONT, GL_AMBIENT, ambient);
    glBegin(GL_TRIANGLES);
    for (i = 0; i < mTerrainTriangles.size(); i++)
    {
        for (j = 0; j < 3; j++)
        {
            v = mTerrainVertices[mTerrainTriangles[i].i[j]];

            glVertexAttrib3f(13, v.n.x, v.n.y, v.n.z);

            glVertexAttrib3f(0, v.p.x, v.p.y, v.p.z);
        }
    }
    glEnd();

    glBindTexture(GL_TEXTURE_2D, NULL);
    glDisable(GL_TEXTURE_2D);

    glUseProgram(NULL);
}
void PlayScene::ToggleCameraMode(void)
{
    mPrevCameraMode = mCameraMode;

    if (mCameraMode == CAMERA_FIRSTPERSON)
        mCameraMode = CAMERA_TOP;
    else if (mCameraMode == CAMERA_TOP)
        mCameraMode = CAMERA_FIRSTPERSON;

    inCameraMode = 0.0f;
}
bool PlayScene::OnEvent(const SDL_Event &event)
{
    if (EventProcessor::OnEvent(event))
        return true;

    return false;
}
#define MOUSEMOVE_SENSITIVITY 0.007f
bool PlayScene::OnMouseMotion(const SDL_MouseMotionEvent &event)
{
    if (mCameraMode == CAMERA_FIRSTPERSON)
    {
        vec3 up = mPlayerPos - mPlanetMassCenter;

        mPlayerForwardDir = MatRotAxis(up, -MOUSEMOVE_SENSITIVITY * event.xrel) * mPlayerForwardDir;

        mPlayerPitch -= MOUSEMOVE_SENSITIVITY * event.yrel;

        // clamp this angle:
        if (mPlayerPitch < -1.45f)
            mPlayerPitch = -1.45f;
        if (mPlayerPitch > 1.45f)
            mPlayerPitch = 1.45f;

        if (!SDL_GetRelativeMouseMode())
        {
            if (SDL_SetRelativeMouseMode(SDL_TRUE) < 0)
            {
                LOG_ERROR("cannot set relative mouse mode: %s",
                          SDL_GetError());
            }
        }
    }

    return false;
}
bool PlayScene::OnMouseWheel(const SDL_MouseWheelEvent &event)
{
    mCameraTopDist = std::max(3.0f, mCameraTopDist - event.y);

    return false;
}
bool PlayScene::OnBoundKeyDown(const KeyBinding keyb)
{
    // As a temporary way to stop the application without a mouse:
    if (keyb == KEYB_MENU)
        pClient->ShutDown();
    else if (keyb == KEYB_CAMERAMODE)
        ToggleCameraMode();

    return false;
}
bool PlayScene::OnQuit(const SDL_QuitEvent &)
{
    pClient->ShutDown();

    return true;
}
