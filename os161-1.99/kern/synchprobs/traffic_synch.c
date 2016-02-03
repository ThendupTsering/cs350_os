#include <types.h>
#include <lib.h>
#include <synchprobs.h>
#include <synch.h>
#include <opt-A1.h>

static struct lock *intersectionLock;
static struct cv *cvDestinationNorth;
static struct cv *cvDestinationSouth;
static struct cv *cvDestinationEast;
static struct cv *cvDestinationWest;
// Boolean of blocked paths:
static volatile int N2E, N2S, N2W;
static volatile int E2S, E2W, E2N;
static volatile int S2W, S2N, S2E;
static volatile int W2N, W2E, W2S;
// Helper Functions:
static bool blockedNW(void);
static bool blockedNE(void);
static bool blockedSW(void);
static bool blockedSE(void);


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
  N2E = 0;
  N2S = 0;
  N2W = 0;
  E2S = 0;
  E2W = 0;
  E2N = 0;
  S2W = 0;
  S2N = 0;
  S2E = 0;
  W2N = 0;
  W2E = 0;
  W2S = 0;
  return;
}


void
intersection_sync_cleanup(void)
{
  KASSERT(cvDestinationNorth != NULL);
  cv_destroy(cvDestinationNorth);
  KASSERT(cvDestinationSouth != NULL);
  cv_destroy(cvDestinationSouth);
  KASSERT(cvDestinationEast != NULL);
  cv_destroy(cvDestinationEast);
  KASSERT(cvDestinationWest != NULL);
  cv_destroy(cvDestinationWest);
  KASSERT(intersectionLock != NULL);
  lock_destroy(intersectionLock);
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
  int ret = (N2W + N2S + N2E + E2W + E2S + S2W);
  return (ret > 0);
}

bool
blockedNE(void)
{
  int ret = (E2N + E2W + E2S + S2N + S2W + W2N);
  return (ret > 0);
}

bool
blockedSE(void)
{
  int ret = (S2E + S2N + S2W + W2E + W2N + N2E);
  return (ret > 0);
}

bool
blockedSW(void)
{
  int ret = (W2S + W2E + W2N + N2S + N2E + E2S);
  return (ret > 0);
}


void
intersection_before_entry(Direction origin, Direction destination)
{
  KASSERT(intersectionLock != NULL);
  lock_acquire(intersectionLock);
  if (origin == north) {
    int otherCarsN = E2S + E2W + E2N + S2W + S2N + S2E + W2N + W2E + W2S;
    if (destination == south) {
      // if (N2S == 0) {
        KASSERT(cvDestinationSouth != NULL);
        while (blockedNW() || blockedSW()) {
          if (otherCarsN == 0) {
            break;
          } else {
            cv_wait(cvDestinationSouth,intersectionLock);
          }
        }
      // }
      N2S = N2S + 1;
    } else if (destination == east) {
      // if (N2E == 0) {
        KASSERT(cvDestinationEast != NULL);
        while (blockedNW() || blockedSW() || blockedSE()) {
          if (otherCarsN == 0) {
            break;
          } else {
            cv_wait(cvDestinationEast,intersectionLock);
          }
        }
      // }
      N2E = N2E + 1;
    } else if (destination == west) {
      // if (N2W == 0) {
        KASSERT(cvDestinationWest != NULL);
        while (blockedNW()) {
          if (otherCarsN == 0) {
            break;
          } else {
            cv_wait(cvDestinationWest,intersectionLock);
          }
        }
      // }
      N2W = N2W + 1;
    }
  } else if (origin == east) {
    int otherCarsE = N2E + N2S + N2W + S2W + S2N + S2E + W2N + W2E + W2S;
    if (destination == west) {
      // if (E2W == 0) {
        KASSERT(cvDestinationWest != NULL);
        while (blockedNE() || blockedNW()) {
          if (otherCarsE == 0) {
            break;
          } else {
            cv_wait(cvDestinationWest,intersectionLock);
          }
        }
      // }
      E2W = E2W + 1;
    } else if (destination == south) {
      // if (E2S == 0) {
        KASSERT(cvDestinationSouth != NULL);
        while (blockedNE() || blockedNW() || blockedSW()) {
          if (otherCarsE == 0) {
            break;
          } else {
            cv_wait(cvDestinationSouth,intersectionLock);
          }
        }
      // }
      E2S = E2S + 1;
    } else if (destination == north) {
      // if (E2N == 0) {
        KASSERT(cvDestinationNorth != NULL);
        while (blockedNE()) {
          if (otherCarsE == 0) {
            break;
          } else {
            cv_wait(cvDestinationNorth,intersectionLock);
          }
        }
      // }
      E2N = E2N + 1;
    }
  } else if (origin == south) {
    int otherCarsS = E2S + E2W + E2N + N2E + N2S + N2W + W2N + W2E + W2S;
    if (destination == north) {
      // if (S2N == 0) {
        KASSERT(cvDestinationNorth != NULL);
        while (blockedSE() || blockedNE()) {
          if (otherCarsS == 0) {
            break;
          } else {
            cv_wait(cvDestinationNorth,intersectionLock);
          }
        }
      // }
      S2N = S2N + 1;
    } else if (destination == east) {
      // if (S2E == 0) {
        KASSERT(cvDestinationEast != NULL);
        while (blockedSE()) {
          if (otherCarsS == 0) {
            break;
          } else {
            cv_wait(cvDestinationEast,intersectionLock);
          }
        }
      // }
      S2E = S2E + 1;
    } else if (destination == west) {
      // if (S2W == 0) {
        KASSERT(cvDestinationWest != NULL);
        while (blockedSE() || blockedNE() || blockedNW()) {
          if (otherCarsS == 0) {
            break;
          } else {
            cv_wait(cvDestinationWest,intersectionLock);
          }
        }
      // }
      S2W = S2W + 1;
    }
  } else if (origin == west) {
    int otherCarsW = E2S + E2W + E2N + S2W + S2N + S2E + N2E + N2S + N2W;
    if (destination == east) {
      // if (W2E == 0) {
        KASSERT(cvDestinationEast != NULL);
        while (blockedSW() || blockedSE()) {
          if (otherCarsW == 0) {
            break;
          } else {
            cv_wait(cvDestinationEast,intersectionLock);
          }
        }
      // }
      W2E = W2E + 1;
    } else if (destination == south) {
      // if (W2S == 0) {
        KASSERT(cvDestinationSouth != NULL);
        while (blockedSW()) {
          if (otherCarsW == 0) {
            break;
          } else {
            cv_wait(cvDestinationSouth,intersectionLock);
          }
        }
      // }
      W2S = W2S + 1;
    } else if (destination == north) {
      // if (W2N == 0) {
        KASSERT(cvDestinationNorth != NULL);
        while (blockedSW() || blockedSE() || blockedNE()) {
          if (otherCarsW == 0) {
            break;
          } else {
            cv_wait(cvDestinationNorth,intersectionLock);
          }
        }
      // }
      W2N = W2N + 1;
    }
  }
  lock_release(intersectionLock);
}


void
intersection_after_exit(Direction origin, Direction destination)
{
  KASSERT(intersectionLock != NULL);
  lock_acquire(intersectionLock);
  if (origin == north) {
    if (destination == south) {
      N2S = N2S - 1;
    } else if (destination == east) {
      N2E = N2E - 1;
    } else if (destination == west) {
      N2W = N2W - 1;
    }
  } else if (origin == east) {
    if (destination == west) {
      E2W = E2W - 1;
    } else if (destination == south) {
      E2S = E2S - 1;
    } else if (destination == north) {
      E2N = E2N - 1;
    }
  } else if (origin == south) {
    if (destination == north) {
      S2N = S2N - 1;
    } else if (destination == east) {
      S2E = S2E - 1;
    } else if (destination == west) {
      S2W = S2W - 1;
    }
  } else if (origin == west) {
    if (destination == east) {
      W2E = W2E - 1;
    } else if (destination == south) {
      W2S = W2S - 1;
    } else if (destination == north) {
      W2N = W2N - 1;
    }
  }
  cv_broadcast(cvDestinationNorth,intersectionLock);
  cv_broadcast(cvDestinationEast,intersectionLock);
  cv_broadcast(cvDestinationSouth,intersectionLock);
  cv_broadcast(cvDestinationWest,intersectionLock);
  lock_release(intersectionLock);
}
