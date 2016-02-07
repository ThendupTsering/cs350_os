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

void
intersection_before_entry(Direction origin, Direction destination)
{
  KASSERT(intersectionLock != NULL);
  lock_acquire(intersectionLock);
  if (origin == north) {
    int otherCarsN = E2S + E2W + S2W + W2N + W2E + W2S;
    int otherCarsN2 = E2S + E2W + S2W + S2N + S2E + W2N + W2E + W2S;
    int otherCarsN3 = E2S + E2W + S2W;
    if (destination == south) {
      KASSERT(cvDestinationSouth != NULL);
      while (otherCarsN != 0) {
        cv_wait(cvDestinationSouth,intersectionLock);
      }
      N2S = N2S + 1;
    } else if (destination == east) {
      KASSERT(cvDestinationEast != NULL);
      while (otherCarsN2 != 0) {
        cv_wait(cvDestinationEast,intersectionLock);
      }
      N2E = N2E + 1;
    } else if (destination == west) {
      KASSERT(cvDestinationWest != NULL);
      while (otherCarsN3 != 0) {
        cv_wait(cvDestinationWest,intersectionLock);
      }
      N2W = N2W + 1;
    }
  } else if (origin == east) {
    int otherCarsE = N2E + N2S + N2W + S2W + S2N + W2N;
    int otherCarsE2 = N2E + N2S + N2W + S2W + S2N + W2N + W2E + W2S;
    int otherCarsE3 = S2W + S2N + W2N;
    if (destination == west) {
      KASSERT(cvDestinationWest != NULL);
      while (otherCarsE != 0) {
        cv_wait(cvDestinationWest,intersectionLock);
      }
      E2W = E2W + 1;
    } else if (destination == south) {
      KASSERT(cvDestinationSouth != NULL);
      while (otherCarsE2 != 0) {
        cv_wait(cvDestinationSouth,intersectionLock);
      }
      E2S = E2S + 1;
    } else if (destination == north) {
      KASSERT(cvDestinationNorth != NULL);
      while (otherCarsE3 != 0) {
        cv_wait(cvDestinationNorth,intersectionLock);
      }
      E2N = E2N + 1;
    }
  } else if (origin == south) {
    int otherCarsS = E2S + E2W + E2N + N2E + W2N + W2E;
    int otherCarsS2 = N2E + W2N + W2E;
    int otherCarsS3 = E2S + E2W + E2N + N2E + N2S + N2W + W2N + W2E;
    if (destination == north) {
      KASSERT(cvDestinationNorth != NULL);
      while (otherCarsS != 0) {
        cv_wait(cvDestinationNorth,intersectionLock);
      }
      S2N = S2N + 1;
    } else if (destination == east) {
      KASSERT(cvDestinationEast != NULL);
      while (otherCarsS2 != 0) {
        cv_wait(cvDestinationEast,intersectionLock);
      }
      S2E = S2E + 1;
    } else if (destination == west) {
      KASSERT(cvDestinationWest != NULL);
      while (otherCarsS3 != 0) {
        cv_wait(cvDestinationWest,intersectionLock);
      }
      S2W = S2W + 1;
    }
  } else if (origin == west) {
    int otherCarsW = E2S + S2W + S2N + S2E + N2E + N2S;
    int otherCarsW2 = E2S + N2E + N2S;
    int otherCarsW3 = E2S + E2W + E2N + S2W + S2N + S2E + N2E + N2S;
    if (destination == east) {
      KASSERT(cvDestinationEast != NULL);
      while (otherCarsW != 0) {
        cv_wait(cvDestinationEast,intersectionLock);
      }
      W2E = W2E + 1;
    } else if (destination == south) {
      KASSERT(cvDestinationSouth != NULL);
      while (otherCarsW2 != 0) {
        cv_wait(cvDestinationSouth,intersectionLock);
      }
      W2S = W2S + 1;
    } else if (destination == north) {
      KASSERT(cvDestinationNorth != NULL);
      while (otherCarsW3 != 0) {
        cv_wait(cvDestinationNorth,intersectionLock);
      }
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
