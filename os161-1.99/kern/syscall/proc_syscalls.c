#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>
#include "opt-A2.h"
#if OPT_A2
  #include <synch.h>
  #include <mips/trapframe.h>
#endif

  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */


#if OPT_A2
  // Check is proc with pid is curProc's child
  struct proc *findChild(struct proc *curProc, int pid) {
  	int length = array_num(curProc->p_children);
  	struct proc *child = NULL;
  	for (int i = 0; i < length; i++) {
  		child = array_get(curProc->p_children, i);
  		if (child->p_pid == pid) {
  			return child;
  		} else {
  			child = NULL;
  		}
  	}
  	return child;
  }
#endif

void sys__exit(int exitcode) {

  struct addrspace *as;
  struct proc *p = curproc;
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */
  // (void)exitcode;

  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

  KASSERT(curproc->p_addrspace != NULL);
  as_deactivate();
  /*
   * clear p_addrspace before calling as_destroy. Otherwise if
   * as_destroy sleeps (which is quite possible) when we
   * come back we'll be calling as_activate on a
   * half-destroyed address space. This tends to be
   * messily fatal.
   */
  as = curproc_setas(NULL);
  as_destroy(as);

  /* detach this thread from its process */
  /* note: curproc cannot be used after this call */
  proc_remthread(curthread);

  #if OPT_A2
  DEBUG(DB_SYSCALL,"Exit Proc: %d\n",p->p_pid);

  p->p_exit_status = _MKWAIT_EXIT(exitcode);
  p->p_canExit = true;

  lock_acquire(p->p_lock_wait);
  cv_broadcast(p->p_cv_wait, p->p_lock_wait);
  lock_release(p->p_lock_wait);

  #endif

  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
  proc_destroy(p);

  thread_exit();
  /* thread_exit() does not return, so we should never get here */
  panic("return from thread_exit in sys_exit\n");
}


/* stub handler for getpid() system call                */
int
sys_getpid(pid_t *retval)
{
  #if OPT_A2
  *retval = curproc->p_pid;
  #else
  /* for now, this is just a stub that always returns a PID of 1 */
  /* you need to fix this to make it work properly */
  *retval = 1;
  #endif
  return(0);
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid,
	    userptr_t status,
	    int options,
	    pid_t *retval)
{
  int exitstatus;
  int result;

#if OPT_A2
  bool isCallerRelated = false;
  struct proc *caller_proc = findChild(curproc, pid);

  if (caller_proc != NULL) {
    isCallerRelated = true;
  } else {
    kprintf ("Current proc: %d is not parent of %d\n", curproc->p_pid, pid);
    return(ECHILD);
  }

  lock_acquire(caller_proc->p_lock_wait);
  while (caller_proc->p_canExit == false)  {
    cv_wait(caller_proc->p_cv_wait, caller_proc->p_lock_wait);
  }
  lock_release(caller_proc->p_lock_wait);
  exitstatus = caller_proc->p_exit_status;

#else
  /* this is just a stub implementation that always reports an
     exit status of 0, regardless of the actual exit status of
     the specified process.
     In fact, this will return 0 even if the specified process
     is still running, and even if it never existed in the first place.

     Fix this!
  */
  /* for now, just pretend the exitstatus is 0 */
  exitstatus = 0;
#endif
  if (options != 0) {
    return(EINVAL);
  }
  result = copyout((void *)&exitstatus,status,sizeof(int));
  if (result) {
    return(result);
  }
  *retval = pid;
  return(0);
}

#if OPT_A2

pid_t sys_fork(struct trapframe *tf, int *retval) {

  // Create Proc for Child Process
	struct proc *fork_child = proc_create_runprogram("fork_child_proc");
  DEBUG(DB_SYSCALL,"Cur Proc PID: %d\n", curproc->p_pid);
  DEBUG(DB_SYSCALL,"Fork child PID: %d\n", fork_child->p_pid);

  KASSERT (fork_child->p_pid > 0);
	if (fork_child == NULL)	{ // proc_create_runprogram returned NULL
    return ENOMEM;
	}

  // Create and Copy Addr Space
  int as_copy_res;
	as_copy_res = as_copy(curproc->p_addrspace, &fork_child->p_addrspace);
	if (as_copy_res) { // as_copy returned an error
    return ENOMEM;
	}

  // Assign PID to child and create parent child relationship
  fork_child->p_parent = curproc;
  int addRes = array_add(curproc->p_children, fork_child, NULL);
  if (addRes) {
    kprintf("Adding child failed for Parent: %d, Child: %d\n", curproc->p_pid, fork_child->p_pid);
  }

  // Create Trapframe Space
  struct trapframe *child_trapframe = kmalloc(sizeof(struct trapframe));
  if (child_trapframe == NULL) {
    kfree(child_trapframe);
    return ENOMEM;
  }

  // Pass trapframe to child thread
  *child_trapframe = *tf;
  KASSERT(fork_child->p_pid > 0);
  *retval = fork_child->p_pid;

  int thread_fork_res;
  // Run Helper function defined in syscall.c, it will advance the PC and call mips_usermode
  thread_fork_res = thread_fork("fork_child", fork_child, &enter_forked_process, child_trapframe, 0);

  if (thread_fork_res) {
    kfree(child_trapframe);
    return thread_fork_res;
  }

  if (fork_child->p_pid < 0) {
    *retval = -1;
    return ENOMEM;
  } else {
    return 0;
  }
}
#endif
