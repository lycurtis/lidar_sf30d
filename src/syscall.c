/**
 * @file syscall.c
 * @brief Newlib syscall stubs — routes printf to USART1 (debug serial)
 */

#include <sys/stat.h>
#include <errno.h>
#include "usart.h"

int _write(int fd, const char *buf, int len)
{
    (void)fd;
    for (int i = 0; i < len; i++)
        usart1_putc(buf[i]);
    return len;
}

int _read(int fd, char *buf, int len)
{
    (void)fd;
    (void)buf;
    (void)len;
    return 0;
}

int _close(int fd) { (void)fd; return -1; }
int _lseek(int fd, int ptr, int dir) { (void)fd; (void)ptr; (void)dir; return 0; }
int _fstat(int fd, struct stat *st) { (void)fd; st->st_mode = S_IFCHR; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
void *_sbrk(int incr) { extern char end; static char *heap = 0; if (!heap) heap = &end; char *prev = heap; heap += incr; return prev; }
