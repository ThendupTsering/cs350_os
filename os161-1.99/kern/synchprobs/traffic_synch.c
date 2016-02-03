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
static volatile int N2E, N2S, N2W;
static volatile int E2S, E2W, E2N;
static volatile int S2W, S2N, S2E;
static volatile int W2N, W2E, W2S;
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
  if (origin == north) {
    if (destination == south) {
      // Straight
      // if (N2S > 0) {
        KASSERT(cvDestinationSouth != NULL);
        while (blockedNW() || blockedSW()) {
          kprintf("N2S\nNo. of cars: " + N2S);
          cv_wait(cvDestinationSouth,intersectionLock);
        }
      // }
      N2S = N2S + 1;
    } else if (destination == east) {
      // Left
      // if (N2E > 0) {
        KASSERT(cvDestinationEast != NULL);
        while (blockedNW() || blockedSW() || blockedSE()) {
          kprintf("N2E\nNo. of cars: " + N2E);
          cv_wait(cvDestinationEast,intersectionLock);
        }
      // }
      N2E = N2E + 1;
    } else if (destination == west) {
      // Right
      // if (N2W > 0) {
        KASSERT(cvDestinationWest != NULL);
        while (blockedNW()) {
          kprintf("N2W\nNo. of cars: " + N2W);
          cv_wait(cvDestinationWest,intersectionLock);
        }
      // }
      N2W = N2W + 1;
    }
  } else if (origin == east) {
    if (destination == west) {
      // Straight
      // if (E2W > 0) {
        KASSERT(cvDestinationWest != NULL);
        while (blockedNE() || blockedNW()) {
          kprintf("E2W\nNo. of cars: " + E2W);
          cv_wait(cvDestinationWest,intersectionLock);
        }
      // }
      E2W = E2W + 1;
    } else if (destination == south) {
      // Left
      // if (E2S > 0) {
        KASSERT(cvDestinationSouth != NULL);
        while (blockedNE() || blockedNW() || blockedSW()) {
          kprintf("E2S\nNo. of cars: " + E2S);
          cv_wait(cvDestinationWest,intersectionLock);
        }
      // }
      E2S = E2S + 1;
    } else if (destination == north) {
      // Right
      // if (E2N > 0) {
        KASSERT(cvDestinationNorth != NULL);
        while (blockedNE()) {
          kprintf("E2N\nNo. of cars: " + E2N);
          cv_wait(cvDestinationNorth,intersectionLock);
        }
      // }
      E2N = E2N + 1;
    }
  } else if (origin == south) {
    if (destination == north) {
      // Straight
      // if (S2N > 0) {
        KASSERT(cvDestinationNorth != NULL);
        while (blockedSE() || blockedNE()) {
          kprintf("S2N\nNo. of cars: " + S2N);
          cv_wait(cvDestinationNorth,intersectionLock);
        }
      // }
      S2N = S2N + 1;
    } else if (destination == east) {
      // Right
      // if (S2E > 0) {
        KASSERT(cvDestinationEast != NULL);
        while (blockedSE()) {
          kprintf("S2E\nNo. of cars: " + S2E);
          cv_wait(cvDestinationEast,intersectionLock);
        }
      // }
      S2E = S2E + 1;
    } else if (destination == west) {
      // Left
      // if (S2W > 0) {
        KASSERT(cvDestinationWest != NULL);
        while (blockedSE() || blockedNE() || blockedNW()) {
          kprintf("S2W\nNo. of cars: " + S2W);
          cv_wait(cvDestinationWest,intersectionLock);
        }
      // }
      S2W = S2W + 1;
    }
  } else if (origin == west) {
    if (destination == east) {
      // Straight
      // if (W2E > 0) {
        KASSERT(cvDestinationEast != NULL);
        while (blockedSW() || blockedSE()) {
          kprintf("W2E\nNo. of cars: " + W2E);
          cv_wait(cvDestinationEast,intersectionLock);
        }
      // }
      W2E = W2E + 1;
    } else if (destination == south) {
      // Right
      // if (W2S > 0) {
        KASSERT(cvDestinationSouth != NULL);
        while (blockedSW()) {
          kprintf("W2S\nNo. of cars: " + W2S);
          cv_wait(cvDestinationSouth,intersectionLock);
        }
      // }
      W2S = W2S + 1;
    } else if (destination == north) {
      // Left
      // if (W2N > 0) {
        KASSERT(cvDestinationNorth != NULL);
        while (blockedSW() || blockedSE() || blockedNE()) {
          kprintf("W2N\nNo. of cars: " + W2N);
          cv_wait(cvDestinationNorth,intersectionLock);
        }
      // }
      W2N = W2N + 1;
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
  KASSERT(intersectionLock != NULL);
  lock_acquire(intersectionLock);
  if (origin == north) {
    if (destination == south) {
      N2S = N2S - 1;
			if (N2S == 0) {
	      cv_broadcast(cvDestinationSouth,intersectionLock);
			}
    } else if (destination == east) {
      N2E = N2E - 1;
			if (N2E == 0) {
	      cv_broadcast(cvDestinationEast,intersectionLock);
			}
    } else if (destination == west) {
      N2W = N2W - 1;
			if (N2W == 0) {
	      cv_broadcast(cvDestinationWest,intersectionLock);
			}
    }
  } else if (origin == east) {
    if (destination == west) {
      E2W = E2W - 1;
			if (E2W == 0) {
	      cv_broadcast(cvDestinationWest,intersectionLock);
			}
    } else if (destination == south) {
      E2S = E2S - 1;
			if (E2S == 0) {
	      cv_broadcast(cvDestinationSouth,intersectionLock);
			}
    } else if (destination == north) {
      E2N = E2N - 1;
			if (E2N == 0) {
	      cv_broadcast(cvDestinationNorth,intersectionLock);
			}
    }
  } else if (origin == south) {
    if (destination == north) {
      S2N = S2N - 1;
			if (S2N == 0) {
	      cv_broadcast(cvDestinationNorth,intersectionLock);
			}
    } else if (destination == east) {
      S2E = S2E - 1;
			if (S2E == 0) {
	      cv_broadcast(cvDestinationEast,intersectionLock);
			}
    } else if (destination == west) {
      S2W = S2W - 1;
			if (S2W == 0) {
	      cv_broadcast(cvDestinationWest,intersectionLock);
			}
    }
  } else if (origin == west) {
    if (destination == east) {
      W2E = W2E - 1;
			if (W2E == 0) {
	      cv_broadcast(cvDestinationEast,intersectionLock);
			}
    } else if (destination == south) {
      W2S = W2S - 1;
			if (W2S == 0) {
	      cv_broadcast(cvDestinationSouth,intersectionLock);
			}
    } else if (destination == north) {
      W2N = W2N - 1;
			if (W2N == 0) {
	      cv_broadcast(cvDestinationNorth,intersectionLock);
			}
    }
  }
  lock_release(intersectionLock);
}
