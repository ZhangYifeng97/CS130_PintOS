#ifndef USERPROG_PROCESS_H
#define USERPROG_PROCESS_H

#include "threads/thread.h"

tid_t process_execute (const char *file_name);
int process_wait (tid_t);
void process_exit (void);
void process_activate (void);
static void extract_command_name(char * cmd_string, char *command_name);
static void extract_command_args(char * cmd_string, char* argv[], int *argc);
void process_close_all(void);
void process_init(void);
#endif /* userprog/process.h */
