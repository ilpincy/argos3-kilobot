#ifndef __DEBUG_H__
#define __DEBUG_H__

#ifdef ARGOS_simulator_BUILD

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/mman.h>

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
extern "C" {
#endif

static int debug_info_fd;
static debug_info_t* debug_info_shm;

void debug_info_destroy() {
   // Unmap the debug info
   munmap(debug_info_shm, sizeof(debug_info_t));
   // Close filename
   close(debug_info_fd);
   // Make file name from robot uid and process id
   char* debug_info_fname;
   asprintf(&debug_info_fname, "/ARGoS_DEBUG_%ld_%s", (long)getppid(), kilo_str_id);
   // Unlink shared memory file
   shm_unlink(debug_info_fname);
   // Get rid of file name
   free(debug_info_fname);
}

void debug_info_create() {
   // Make file name from robot uid and process id
   char* debug_info_fname;
   asprintf(&debug_info_fname, "/ARGoS_DEBUG_%ld_%s", (long)getppid(), kilo_str_id);
   // Open shared file
   debug_info_fd = shm_open(debug_info_fname, O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
   // Check for errors
   if(debug_info_fd < 0) {
      fprintf(stderr, "Opening the shared memory file of Kilobot %u: %s\n", kilo_uid, strerror(errno));
      exit(1);
   }
   // Resize debug shared memory area to contain the debug info, filling it with zeros */
   ftruncate(debug_info_fd, sizeof(debug_info_t));
   // Map file to memory
   debug_info_shm =
      (debug_info_t*)mmap(NULL,
                          sizeof(debug_info_t),
                          PROT_READ | PROT_WRITE,
                          MAP_SHARED,
                          debug_info_fd,
                          0);
   if(debug_info_shm == MAP_FAILED) {
      fprintf(stderr, "Mmapping the shared memory area of kilobot %u: %s\n", kilo_uid, strerror(errno));
      close(debug_info_fd);
      shm_unlink(debug_info_fname);
      exit(1);
   }
   // Get rid of file name
   free(debug_info_fname);
   // Make sure to cleanup when exiting
   atexit(debug_info_destroy);
}

#ifdef __cplusplus /* If this is a C++ compiler, use C linkage */
}
#endif

#define debug_info_get(FIELD) (debug_info_shm->FIELD)
#define debug_info_set(FIELD, VALUE) debug_info_get(FIELD) = (VALUE)
#define debug_print(MSG, ...) fprintf(stderr, "[kb%u] " MSG, kilo_uid, ##__VA_ARGS__);

#else

void debug_info_create() {}
void debug_info_destroy() {}

#define debug_info_get(FIELD)
#define debug_info_set(FIELD, VALUE)
#define debug_print(MSG, ...)

#endif // ARGOS_simulator_BUILD

#endif // __DEBUG_H__
