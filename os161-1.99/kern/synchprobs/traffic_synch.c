#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

/*
 * This simple default synchronization mechanism allows only vehicle at a time
 * into the intersection.   The intersectionSem is used as a a lock.
 * We use a semaphore rather than a lock so that this code will work even
 * before locks are implemented.
 */

/*
 * Replace this default synchronization mechanism with your own (better) mechanism
 * needed for your solution.   Your mechanism may use any of the available synchronzation
 * primitives, e.g., semaphores, locks, condition variables.   You are also free to
 * declare other global variables if your solution requires them.
 */

/*
 * replace this with declarations of any synchronization and other variables you need here
 */
static struct lock *intersectionLock;
static struct cv *cvDestinationNorth;
static struct cv *cvDestinationSouth;
static struct cv *cvDestinationEast;
static struct cv *cvDestinationWest;
// Boolean of blocked paths:
static volatile bool N2E, N2S, N2W;
static volatile bool E2S, E2W, E2N;
static volatile bool S2W, S2N, S2E;
static volatile bool W2N, W2E, W2S;
// Helper Functions:
static bool blockedNW(void);
static bool blockedNE(void);
static bool blockedSW(void);
static bool blockedSE(void);



/*
 * The simulation driver will call this function once before starting
 * the simulation
 *
 * You can use it to initialize synchronization and other variables.
 *
 */
void
intersection_sync_init(void)
{
  intersectionLock = lock_create("intersectionLock");
  if (intersectionLock == NULL) {
    panic("could not create intersection lock");
  }
  cvDestinationNorth = cv_create("cvDestinationNorth");
  if (cvDestinationNorth == NULL) {
    panic("could not create condition variable cvDestinationNorth");
  }
  cvDestinationSouth = cv_create("cvDestinationSouth");
  if (cvDestinationSouth == NULL) {
    panic("could not create condition variable cvDestinationSouth");
  }
  cvDestinationEast = cv_create("cvDestinationEast");
  if (cvDestinationEast == NULL) {
    panic("could not create condition variable cvDestinationEast");
  }
  cvDestinationWest = cv_create("cvDestinationWest");
  if (cvDestinationWest == NULL) {
    panic("could not create condition variable cvDestinationWest");
  }
  // All paths are free before traffic starts
  N2E = false;
  N2S = false;
  N2W = false;
  E2S = false;
  E2W = false;
  E2N = false;
  S2W = false;
  S2N = false;
  S2E = false;
  W2N = false;
  W2E = false;
  W2S = false;
  return;
}

/*
 * The simulation driver will call this function once after
 * the simulation has finished
 *
 * You can use it to clean up any synchronization and other variables.
 *
 */
void
intersection_sync_cleanup(void)
{
  KASSERT(intersectionLock != NULL);
  lock_destroy(intersectionLock);
  KASSERT(cvDestinationNorth != NULL);
  cv_destroy(cvDestinationNorth);
  KASSERT(cvDestinationSouth != NULL);
  cv_destroy(cvDestinationSouth);
  KASSERT(cvDestinationEast != NULL);
  cv_destroy(cvDestinationEast);
  KASSERT(cvDestinationWest != NULL);
  cv_destroy(cvDestinationWest);
}

/*
  Intersection will be split into 4 quadrants:
  * * * * * *
  * NW * NE *
  * * * * * *
  * SW * SE *
  * * * * * *
*/
bool
blockedNW(void)
{
  bool ret = (N2W && N2S && N2E && E2W && E2S && S2W);
  return ret;
}

bool
blockedNE(void)
{
  bool ret = (E2N && E2W && E2S && S2N && S2W && W2N);
  return ret;
}

bool
blockedSE(void)
{
  bool ret = (S2E && S2N && S2W && W2E && W2N && N2E);
  return ret;
}

bool
blockedSW(void)
{
  bool ret = (W2S && W2E && W2N && N2S && N2E && E2S);
  return ret;
}


/*
 * The simulation driver will call this function each time a vehicle
 * tries to enter the intersection, before it enters.
 * This function should cause the calling simulation thread
 * to block until it is OK for the vehicle to enter the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle is arriving
 *    * destination: the Direction in which the vehicle is trying to go
 *
 * return value: none
 */

void
intersection_before_entry(Direction origin, Direction destination)
{
  KASSERT(intersectionLock != NULL);
  lock_acquire(intersectionLock);
  switch (origin) {
    case north:
      switch (destination) {
        case south:
          if (!N2S) {
            KASSERT(cvDestinationSouth != NULL);
            while (blockedNW() && blockedSW()) {
              cv_wait(cvDestinationSouth,intersectionLock);
            }
            N2S = true;
          }
        case east:
          if (!N2E) {
            KASSERT(cvDestinationEast != NULL);
            while (blockedNW() && blockedSW() && blockedSE()) {
              cv_wait(cvDestinationEast,intersectionLock);
            }
            N2E = true;
          }
        case west:
          if (!N2W) {
            KASSERT(cvDestinationWest != NULL);
            while (blockedNW()) {
              cv_wait(cvDestinationWest,intersectionLock);
            }
            N2W = true;
          }
      }
    case east:
      switch (destination) {
        case west:
          if (!E2W) {
            KASSERT(cvDestinationWest != NULL);
            while (blockedNE() && blockedNW()) {
              cv_wait(cvDestinationWest,intersectionLock);
            }
            E2W = true;
          }
        case south:
          if (!E2S) {
            KASSERT(cvDestinationSouth != NULL);
            while (blockedNE() && blockedNW() && blockedSW()) {
              cv_wait(cvDestinationWest,intersectionLock);
            }
            E2S = true;
          }
        case north:
          if (!E2N) {
            KASSERT(cvDestinationNorth != NULL);
            while (blockedNE()) {
              cv_wait(cvDestinationNorth,intersectionLock);
            }
            E2N = true;
          }
      }
    case south:
      switch (destination) {
        case north:
          if (!S2N) {
            KASSERT(cvDestinationNorth != NULL);
            while (blockedSE() && blockedNE()) {
              cv_wait(cvDestinationNorth,intersectionLock);
            }
            S2N = true;
          }
        case east:
          if (!S2E) {
            KASSERT(cvDestinationEast != NULL);
            while (blockedSE()) {
              cv_wait(cvDestinationEast,intersectionLock);
            }
            S2E = true;
          }
        case west:
          if (!S2W) {
            KASSERT(cvDestinationWest != NULL);
            while (blockedSE() && blockedNE() && blockedNW()) {
              cv_wait(cvDestinationWest,intersectionLock);
            }
            S2W = true;
          }
      }
    case west:
      switch (destination) {
        case east:
          if (!W2E) {
            KASSERT(cvDestinationEast != NULL);
            while (blockedSW() && blockedSE()) {
              cv_wait(cvDestinationEast,intersectionLock);
            }
            W2E = true;
          }
        case south:
          if (!W2S) {
            KASSERT(cvDestinationSouth != NULL);
            while (blockedSW()) {
              cv_wait(cvDestinationSouth,intersectionLock);
            }
            W2S = true;
          }
        case north:
          if (!W2N) {
            KASSERT(cvDestinationNorth != NULL);
            while (blockedSW() && blockedSE() && blockedNE()) {
              cv_wait(cvDestinationNorth,intersectionLock);
            }
            W2N = true;
          }
        }
  }
  lock_release(intersectionLock);
}


/*
 * The simulation driver will call this function each time a vehicle
 * leaves the intersection.
 *
 * parameters:
 *    * origin: the Direction from which the vehicle arrived
 *    * destination: the Direction in which the vehicle is going
 *
 * return value: none
 */

void
intersection_after_exit(Direction origin, Direction destination)
{
  switch (origin) {
    case north:
      switch (destination) {
        case south:
          N2S = false;
          cv_signal(cvDestinationSouth,intersectionLock);
        case east:
          N2E = false;
          cv_signal(cvDestinationEast,intersectionLock);
        case west:
          N2W = false;
          cv_signal(cvDestinationWest,intersectionLock);
      }
    case east:
      switch (destination) {
        case west:
          E2W = false;
          cv_signal(cvDestinationWest,intersectionLock);
        case south:
          E2S = false;
          cv_signal(cvDestinationSouth,intersectionLock);
        case north:
          E2N = false;
          cv_signal(cvDestinationNorth,intersectionLock);
      }
    case south:
      switch (destination) {
        case north:
          S2N = false;
          cv_signal(cvDestinationNorth,intersectionLock);
        case east:
          S2E = false;
          cv_signal(cvDestinationEast,intersectionLock);
        case west:
          S2W = false;
          cv_signal(cvDestinationWest,intersectionLock);
      }
    case west:
      switch (destination) {
        case east:
          W2E = false;
          cv_signal(cvDestinationEast,intersectionLock);
        case south:
          W2S = false;
          cv_signal(cvDestinationSouth,intersectionLock);
        case north:
          W2N = false;
          cv_signal(cvDestinationNorth,intersectionLock);
        }
  }
}
