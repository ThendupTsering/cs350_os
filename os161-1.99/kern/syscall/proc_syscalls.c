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
#include <vm.h>
#include <vfs.h>
#include <kern/fcntl.h>
#include <limits.h>

#include "opt-A2.h"
#if OPT_A2
  #include <synch.h>
  #include <mips/trapframe.h>
#endif

  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */


#if OPT_A2

  struct ProcHolder *getProcHolder(struct array *hayStack, int needle) {
    int length = array_num(hayStack);
    struct ProcHolder *candProcHolder = NULL;
    for (int i = 0; i < length; i++) {
      candProcHolder = array_get(hayStack, i);
      if (candProcHolder->p_pid == needle) {
        // kprintf("Index: %d and Pid: %d\n",i+1,candProcHolder->p_pid);
        KASSERT(i+1 == candProcHolder->p_pid);
        return candProcHolder;
      } else {
        candProcHolder = NULL;
      }
    }
    return candProcHolder;
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
  lock_acquire(procTableLock);
  struct ProcHolder *curproc_holder = NULL;
  curproc_holder = getProcHolder(processTable, p->p_pid);
  lock_release(procTableLock);
  DEBUG(DB_SYSCALL,"Exit %d: Proc %d is exiting\n",p->p_pid,p->p_pid);

  curproc_holder->p_canExit = true;
  curproc_holder->p_exit_status = _MKWAIT_EXIT(exitcode);
  DEBUG(DB_SYSCALL,"Exit %d: Proc %d has exited, now broadcasting\n",p->p_pid,p->p_pid);

  if (curproc_holder->p_parent != NULL) {
    lock_acquire(curproc_holder->p_parent->p_lock_wait);
    cv_broadcast(curproc_holder->p_parent->p_cv_wait, curproc_holder->p_parent->p_lock_wait);
    lock_release(curproc_holder->p_parent->p_lock_wait);
  }

  DEBUG(DB_SYSCALL,"Exit %d: Proc %d has exited completely\n",p->p_pid,p->p_pid);

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
  *retval = -1;
  if(options!=0)
	{
		return(EINVAL);
	}
  // Any proc that calls waitpid should be a child proc
  lock_acquire(procTableLock);
  struct ProcHolder *cur_proc_holder = getProcHolder(processTable, curproc->p_pid);
  lock_release(procTableLock);
  lock_acquire(procTableLock);
  struct ProcHolder *child_proc_holder = getProcHolder(processTable, pid);
  lock_release(procTableLock);

  DEBUG(DB_SYSCALL,"Wait %d: Proc %d is in WaitPID\n", pid, pid);
  if (child_proc_holder->p_parent->p_pid == cur_proc_holder->p_pid) { // Calling proc is the of child of current proc
    lock_acquire(cur_proc_holder->p_lock_wait);
    while (child_proc_holder->p_canExit == false)  {
      DEBUG(DB_SYSCALL,"Wait %d: Waiting for PID %d to be exitable\n", pid, child_proc_holder->p_pid);
      cv_wait(cur_proc_holder->p_cv_wait, cur_proc_holder->p_lock_wait);
    }
    lock_release(cur_proc_holder->p_lock_wait);

    exitstatus = child_proc_holder->p_exit_status;
  } else {
    DEBUG(DB_SYSCALL,"Wait %d: Proc %d is NOT child of %d\n", pid, pid, cur_proc_holder->p_pid);
    return(ECHILD);
  }
  DEBUG(DB_SYSCALL,"Wait %d: ProcessHolder Array after WaitPID is now:\n", pid);
  if (dbflags == DB_SYSCALL) {
    printArr();
  }
  DEBUG(DB_SYSCALL,"Wait %d: Proc %d is leaving WaitPID\n", pid, pid);
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
  lock_acquire(procTableLock);
  struct ProcHolder *fork_child_holder = getProcHolder(processTable, fork_child->p_pid);
  lock_release(procTableLock);
  lock_acquire(procTableLock);
  struct ProcHolder *curproc_holder = getProcHolder(processTable, curproc->p_pid);
  lock_release(procTableLock);

  DEBUG(DB_SYSCALL,"Fork %d: Cur Proc %d if being forked to create child %d\n", curproc->p_pid, curproc->p_pid, fork_child->p_pid);

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
  fork_child_holder->p_parent = curproc_holder;
  int addRes = array_add(curproc_holder->p_children, fork_child_holder, NULL);
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
  DEBUG(DB_SYSCALL,"Fork %d: ProcessHolder Array after Fork is now:\n", curproc->p_pid);
  if (dbflags == DB_SYSCALL) {
    printArr();
  }
  DEBUG(DB_SYSCALL,"Fork %d: Child %d has been created\n", curproc->p_pid, fork_child->p_pid);
  if (fork_child->p_pid < 0) {
    *retval = -1;
    return ENOMEM;
  } else {
    *retval = fork_child->p_pid;
    return 0;
  }
}

int sys_execv(userptr_t progName, userptr_t args) {
  DEBUG(DB_SYSCALL,"ExecV: Begin\n");
  // Check for error in passing args
  if ( ((char*) progName == NULL) || ((char**) args == NULL) ) {
    return EFAULT;
  }

  DEBUG(DB_SYSCALL,"ExecV: Copy Args into Kernel Begin\n");
  // Count the no. of arguments and copy them into kernel
  char **kernArgPtrs = kmalloc(ARG_MAX);
  bool paramsAvailable = true;
  int paramsCopied = 0;
  while (paramsAvailable) {
    char *param = NULL;
    userptr_t paramAddr = args + (paramsCopied * sizeof(char*));
    copyin(paramAddr, &param, sizeof(char*));
    kernArgPtrs[paramsCopied] = param;
    if (param == NULL) {
      paramsAvailable = false;
      break;
    }
    if (strlen(param) > 1024) {
      return E2BIG;
    }
    paramsCopied++;
  }
  DEBUG(DB_SYSCALL,"ExecV: %d Params Copied into Kernel\n", paramsCopied);
	if (paramsCopied > 64) {
		return E2BIG;
	}

  // Copy Program path into kernel
  char *kernProgName = kmalloc(PATH_MAX);
  size_t *progNameLength;
  int res = copyinstr(progName, kernProgName, PATH_MAX, progNameLength);
  DEBUG(DB_SYSCALL,"ExecV: CopyInStr Passed\n");
  if (res != 0) {
    return res;
  }

  // Create array for offsets for mem alignment later
  int memForArgs = paramsCopied * sizeof(size_t);
  size_t *argOffsets = kmalloc(memForArgs);
  char *kernArgs = kmalloc(ARG_MAX);
  int offset = 0;
  int count = 0;
  while (count < paramsCopied) {
    size_t argLength;
    int copyStrRes = copyinstr((userptr_t) kernArgPtrs[count], kernArgs + offset, ARG_MAX, &argLength);
    if (copyStrRes){
			return copyStrRes;
		}
    argOffsets[count] = offset;
    argLength += 1;
    offset += ROUNDUP(argLength, 8);
    count++;
  }
  DEBUG(DB_SYSCALL,"ExecV: Array of offsets created, %d params done\n", count);

  // Get CurProc AddrSpace and save it (will delete later)
  struct addrspace *deleteAS = curproc_getas();

  struct addrspace *as;
	struct vnode *v;
	vaddr_t entrypoint, stackptr;
	int result;

	/* Open the file. */
	result = vfs_open(kernProgName, O_RDONLY, 0, &v);
	if (result) {
		return result;
	}

	/* Create a new address space. */
	as = as_create();
	if (as == NULL) {
		vfs_close(v);
		return ENOMEM;
	}

	/* Switch to it and activate it. */
	curproc_setas(as);
	as_activate();

	/* Load the executable. */
	result = load_elf(v, &entrypoint);
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		vfs_close(v);
		return result;
	}

	/* Done with the file now. */
	vfs_close(v);

  /* Define the user stack in the address space */
	result = as_define_stack(as, &stackptr);
	if (result) {
		/* p_addrspace will go away when curproc is destroyed */
		return result;
	}

  DEBUG(DB_SYSCALL,"ExecV: Copy back to user space, begin\n");
  vaddr_t curPtr = stackptr - offset;
  res = copyout(kernArgs, (userptr_t) curPtr, offset);
	if (res) {
		return res;
	}

  int paramsWithNull = paramsCopied+1;
  int paramMemSize = paramsWithNull * sizeof(userptr_t);
  userptr_t tempPtr;
  userptr_t *argOffsetsReverse = kmalloc(paramMemSize);
	for (int i = 0; i < paramsCopied; i++) {
		tempPtr = (userptr_t) curPtr + argOffsets[i];
		argOffsetsReverse[i] = tempPtr;
	}
	argOffsetsReverse[paramsCopied] = NULL;
	curPtr = curPtr - paramMemSize;
	res = copyout(argOffsetsReverse, (userptr_t)curPtr, paramMemSize);
	if (res) {
		return res;
	}
  DEBUG(DB_SYSCALL,"ExecV: Copy back to user space, done\n");

  as_destroy(deleteAS);
	kfree(kernArgPtrs);
	kfree(kernProgName);
  kfree(argOffsets);
  kfree(kernArgs);

  DEBUG(DB_SYSCALL,"ExecV: End\n");
  // Call enter_new_process
	/* Warp to user mode. */
	enter_new_process(paramsCopied, (userptr_t) curPtr, curPtr, entrypoint);

	/* enter_new_process does not return. */
	panic("enter_new_process returned\n");
	return EINVAL;

}

#endif
