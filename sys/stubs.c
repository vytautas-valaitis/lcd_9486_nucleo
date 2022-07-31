#pragma GCC diagnostic ignored "-Wunused-parameter"

extern int errno;
extern int __io_putchar(int ch) __attribute__((weak));
extern int __io_getchar(void) __attribute__((weak));

/*
int _write(int fd, const void *buffer, unsigned int count) {
    return -1;
}*/

int _write(int file, char *ptr, int len) {
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		__io_putchar(*ptr++);
	}
	return len;
}

int _read(int fd, void *buffer, unsigned int count) {
    return -1;
}
/*
__attribute__((weak)) int _read(int file, char *ptr, int len) {
	int DataIdx;
 
	for (DataIdx = 0; DataIdx < len; DataIdx++)
	{
		*ptr++ = __io_getchar();
	}
  return len;
}
*/
int _close(int fd) {
    return -1;
}

int _fstat(int fd, void *buffer) {
    return -1;
}

int _isatty(int fd) {
    return -1;
}

long _lseek(int fd, long offset, int origin) {
    return -1;
}

void _exit(int status) {

}

