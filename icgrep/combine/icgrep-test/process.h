#ifndef PROCESS_H
#define PROCESS_H

#include "process.h"
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <sys/types.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <unistd.h>

#define TIME_LIMIT 10   //determine time limit in sec

int timeout = 0;
int child_done = 0;
int errNum = 0;

void child_handler(int sig)
{
    child_done = 1;
}

void alarm_handler(int sig)
{
    timeout = 1;
}

void segfault_handler(int sig){
	errNum = sig;
}

int run_test(vector<string> stringArgs, string fileName) {
	  timeout = 0;
    child_done = 0;
    errNum = 0;
    int fd;

	pid_t pid = fork();
    if (pid == -1) {
        perror("fork failed");
        exit(1);
    }
    else if (pid == 0) {
    	signal(SIGSEGV, segfault_handler);

    	const char **argv = new const char* [stringArgs.size()+1];
	    for (int i = 0;  i < stringArgs.size();  ++i) {
	    	argv [i] = stringArgs[i] .c_str();
        cout << stringArgs[i] << " ";
	    }
      cout << endl;
      
	    argv[stringArgs.size()] = (char*)NULL;

      if ((fd = open(fileName.c_str(), O_WRONLY | O_CREAT, S_IRUSR | S_IWUSR)) == -1){
			     fprintf(stderr, "Cannot open %s. Try again later.\n", fileName.c_str());
		  }
      // redirect stdout
      dup2(fd, 1);
      dup2(fd, 2);
      close(fd);

      execv(argv[0], (char**)argv);
      cout << endl;
      exit(0);
    }
    else {

      // set up the signal handlers after forking so the child doesn't inherit them
      signal(SIGSEGV, SIG_DFL);
      signal(SIGALRM, alarm_handler);
      signal(SIGCHLD, child_handler);

      alarm(TIME_LIMIT);  // install an alarm to be fired after TIME_LIMIT
      pause();
      if (timeout) {
          printf("alarm triggered\n");
          printf("killing child\n");
          kill(pid, SIGKILL);
          wait(NULL);
          errNum = -1;
          // } else {
          //     printf("alarm triggered, but child finished normally\n");
          // }
      }
      else if (child_done) {
          printf("child finished normally\n");
          wait(NULL);
      }
    }

	return errNum;

}

#endif // STRINGGEN_H
