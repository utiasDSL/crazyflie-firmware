/*
The MIT License (MIT)

Copyright (c) 2019 Mario Vukosavljev

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
This controller is based on the following publication:

TO DO
*/
#include <string.h>
#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"
#include "position_controller.h"
#include "FreeRTOS.h"
#include "task.h"
#include "system.h"

#define PRIM_OFF  0
#define PRIM_HOLD 1
#define PRIM_FORW 2
#define PRIM_BACK 3
#define LANDTHRES 0.08f  // height [m] to shut off motors when descending
#define MAXCOUNT 10  // factor slower that primitives are updated
#define PLANLEN 7 //22

static int counter = 0;

// Harded-coded motion primitive plan
// The second last primitive should be going down at z = 0 box (landing)
// The last primitive should be "off", activated with landing threshold
static bool internalplan = false;
static int planlen = PLANLEN;
static int planloc = 0;
static int planx[PLANLEN] = {1,1,2,1,1,1,0};//{1,2,2,2,2,3,3,3,3,1,2,2,2,2,1,3,3,1,1,1,1,0};
static int plany[PLANLEN] = {1,1,1,1,1,1,0};//{1,1,1,1,1,1,1,1,1,1,1,1,1,1,2,1,1,3,1,1,1,0};
static int planz[PLANLEN] = {2,2,1,3,3,3,0};//{2,1,1,1,1,1,1,1,1,2,1,1,1,1,1,1,1,1,3,3,1,0};

static int primcur[3] = {PRIM_OFF, PRIM_OFF, PRIM_OFF};  // current primitives
static int curidxs[3] = {0, 0, 0}; // current box indices
static bool initbox = false;  // true when initialized

static float dt = 0.001f;  // update rate is 1000Hz
static float pos_mid[3] = {0, 0, 0};
static float pos_sp[3] = {0, 0, 0};
static float vel_sp[3] = {0, 0, 0};
static float t_0[3] = {0, 0, 0};  // time elapsed [s]
static float pos_0[3] = {0, 0, 0};  // start position in [m]
static float vel_0[3] = {0, 0, 0};  // start velocity in [m/s]
static float taus[3] = {0.2f, 0.2f, 0.2f}; //{1.0f, 1.0f, 1.0f};

static float box_lens[3] = {1.0f, 1.0f, 1.0f};  // box lengths in [m]
static float vel_noms[3] = {0.5f, 0.5f, 0.5f};  // nominal velocities in [m/s]


void box_idxs(int *idxs, const float *pos3_cur)
{
    for (int i = 0; i < 3; i++)
        idxs[i] = floorf(pos3_cur[i] / box_lens[i] + 0.5f);
}

// For backwards, put the negative of vel_nom
void ctr_forw(float *pos_sp, float *vel_sp, const float t, const float pos_0, const float vel_0,
          const float tau, const float vel_nom)
{
    float lam = expf(-t / tau);
    *pos_sp = pos_0 + vel_nom * t + tau * (vel_0 - vel_nom) * (1.0f - lam);
    *vel_sp = vel_nom + (vel_0 - vel_nom) * lam;
}

void ctr_hold(float *pos_sp, float *vel_sp, const float t, const float pos_0, const float pos_mid,
              const float tau)
{
    float lam = expf(-t / tau);
    *pos_sp = pos_mid + lam * (pos_0 - pos_mid);
    *vel_sp = lam * (pos_mid - pos_0) / tau;
}

/*
void ctr_hold(float *pos_sp, float *vel_sp, const float t, const float pos_0, const float vel_0,
          const float tau)
{
    float lam = expf(-t / tau);
    *pos_sp = pos_0 + tau * vel_0 * (1.0f - lam);
    *vel_sp = vel_0 * lam;
}
 */

// 0 = landed, 1 = hold, 2 = forward, 3 = backwards
bool verify_prims(const int *prims)
{
    for (int i = 0; i < 3; i++)
    {
        if (prims[i] < 0 || prims[i] > 3)
            return false;
    }
    return true;
}

void nextPrim(int *prims, const setpoint_t *setpoint, const state_t *state)
{
    // Update at lower frequency
    if (counter < MAXCOUNT)  // same prim as before
    {
        counter++;
        for (int j = 0; j < 3; j++)
            prims[j] = primcur[j];
        return;
    }
    counter = 0;  // reset and proceed

    int idxs[3] = {0, 0, 0};
    float pos_cur[3] = {state->position.x, state->position.y, state->position.z};
    box_idxs(idxs, pos_cur);

    if (!initbox)
    {
        // Update initial box indices the first time
        initbox = true;
        for (int j = 0; j < 3; j++)
            curidxs[j] = idxs[j];
    }

    if (internalplan)
    {
        if (setpoint->position.z > 0.0f)  // zero indicates motors off
        {
            prims[0] = planx[planloc];
            prims[1] = plany[planloc];
            prims[2] = planz[planloc];

            // Update plan index if box changed
            bool change = (idxs[0] != curidxs[0]) || (idxs[1] != curidxs[1]) || (idxs[2] != curidxs[2]);

            // Update location in plan
            if (planloc < planlen-2 && change)
                planloc++;
            else if (planloc == planlen-2 && state->position.z < LANDTHRES)  // land
                planloc++;
        }
        else
        {
            prims[0] = PRIM_OFF;
            prims[1] = PRIM_OFF;
            prims[2] = PRIM_OFF;
        }
    }
    else
    {
        prims[0] = (int)setpoint->position.x;
        prims[1] = (int)setpoint->position.y;
        prims[2] = (int)setpoint->position.z;
    }

    // Update current box indices and middle of boxes
    for (int j = 0; j < 3; j++)
    {
        curidxs[j] = idxs[j];
        pos_mid[j] = (float)curidxs[j] * box_lens[j];
    }

}

void computeSetpoints(setpoint_t *setpoint2, const state_t *state, const int *prims)
{
    //float pos_cur[3] = {state->position.x, state->position.y, state->position.z};
    //float vel_cur[3] = {state->velocity.x, state->velocity.y, state->velocity.z};

    for (int i = 0; i < 3; i++)
    {
        // Reset quantities if primitive changed
        if (prims[i] != primcur[i])
        {
            t_0[i] = 0;
            pos_0[i] = pos_sp[i]; //pos_cur[i];
            vel_0[i] = vel_sp[i]; //vel_cur[i];

            if (prims[i] == PRIM_HOLD)
            {
                // shouldn't need abs (tau > 0), and vel should be not zero
                taus[i] = fabs((pos_mid[i] - pos_0[i]) / vel_0[i]);
            }
            else
                taus[i] = box_lens[i] / (2.0f * vel_noms[i]);


            if (primcur[i] == PRIM_OFF && prims[i] == PRIM_HOLD)
            {
                // this achieves constant set-point at middle of current box
                pos_0[i] = pos_mid[i];
                vel_0[i] = 0;
                taus[i] = 1.0f;  // any nonzero dummy value
                //vel_0[i] = 0.01f; //10.0f*vel_noms[i];  // enforce movement initially
            }
        }

        // Calculate set points based on primitive
        if (prims[i] == PRIM_HOLD)
        {
            //ctr_hold(&(pos_sp[i]), &(vel_sp[i]), t_0[i], pos_0[i], vel_0[i], taus[i]);
            ctr_hold(&(pos_sp[i]), &(vel_sp[i]), t_0[i], pos_0[i], pos_mid[i], taus[i]);
        }
        else
        {
            float vel_nom = prims[i] == PRIM_FORW ? vel_noms[i] : -vel_noms[i];
            ctr_forw(&(pos_sp[i]), &(vel_sp[i]), t_0[i], pos_0[i], vel_0[i], taus[i], vel_nom);
        }

        t_0[i] += dt;  // update internal time
    }

    setpoint2->position.x = pos_sp[0];
    setpoint2->position.y = pos_sp[1];
    setpoint2->position.z = pos_sp[2];
    setpoint2->velocity.x = vel_sp[0];
    setpoint2->velocity.y = vel_sp[1];
    setpoint2->velocity.z = vel_sp[2];
}

void controllerPrimitives(setpoint_t *setpoint2, const setpoint_t *setpoint,
                                         const state_t *state)
{
    memcpy(setpoint2, setpoint, sizeof(setpoint_t));  // copy all other variables

    int prims[3] = {PRIM_OFF, PRIM_OFF, PRIM_OFF};
    nextPrim(prims, setpoint, state);

    bool off = (prims[0] == PRIM_OFF) || (prims[1] == PRIM_OFF) || (prims[2] == PRIM_OFF);

    if (off || !verify_prims(prims))  // disable motors
    {
        setpoint2->mode.z = modeDisable;
        setpoint2->thrust = 0.0f;
    }
    else
    {
        computeSetpoints(setpoint2, state, prims);
    }

    memcpy(primcur, prims, 3*sizeof(int));  // update latest primitives
}

/*
PARAM_GROUP_START(prims)
PARAM_ADD(PARAM_FLOAT, tau_xy, &tau_xy)
PARAM_ADD(PARAM_FLOAT, tau_z, &tau_z)
PARAM_ADD(PARAM_FLOAT, boxlen_x, &boxlen_x)
PARAM_ADD(PARAM_FLOAT, boxlen_y, &boxlen_y)
PARAM_ADD(PARAM_FLOAT, boxlen_z, &boxlen_z)
PARAM_GROUP_STOP(prims)
*/

LOG_GROUP_START(prims)
LOG_ADD(LOG_INT8, px, &(primcur[0]))
LOG_ADD(LOG_INT8, py, &(primcur[1]))
LOG_ADD(LOG_INT8, pz, &(primcur[2]))
LOG_ADD(LOG_INT8, bx, &(curidxs[0]))
LOG_ADD(LOG_INT8, by, &(curidxs[1]))
LOG_ADD(LOG_INT8, bz, &(curidxs[2]))
//LOG_ADD(LOG_INT8, ploc, &planloc)
//LOG_ADD(LOG_UINT8, count, &counterg)
LOG_GROUP_STOP(prims)

LOG_GROUP_START(prims2)
LOG_ADD(LOG_FLOAT, p0x, &(pos_0[0]))
LOG_ADD(LOG_FLOAT, p0y, &(pos_0[1]))
LOG_ADD(LOG_FLOAT, p0z, &(pos_0[2]))
LOG_ADD(LOG_FLOAT, v0x, &(vel_0[0]))
LOG_ADD(LOG_FLOAT, v0y, &(vel_0[1]))
LOG_ADD(LOG_FLOAT, v0z, &(vel_0[2]))
LOG_GROUP_STOP(prims2)

LOG_GROUP_START(prims3)
LOG_ADD(LOG_FLOAT, px, &(pos_sp[0]))
LOG_ADD(LOG_FLOAT, py, &(pos_sp[1]))
LOG_ADD(LOG_FLOAT, pz, &(pos_sp[2]))
LOG_ADD(LOG_FLOAT, vx, &(vel_sp[0]))
LOG_ADD(LOG_FLOAT, vy, &(vel_sp[1]))
LOG_ADD(LOG_FLOAT, vz, &(vel_sp[2]))
LOG_GROUP_STOP(prims3)