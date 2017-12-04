#ifdef __x86_64

#undef abort
#undef close
#undef closedir
#undef _exit
#undef fsync
#undef getenv
#undef gettimeofday
#undef isatty
#undef lseek
#undef mkdir
#undef open
#undef opendir
#undef read
#undef readdir
#undef realpath
#undef sigemptyset
#undef signal
#undef stat
#undef strerror
#undef system
#undef tcgetattr
#undef tcsetattr
#undef unlink
#undef write

#include <dirent.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <stdio.h>
#include <sys/time.h>

#include <signal.h>

void _abort_ (void) { abort(); }
int _close_ (int __fd) { return close(__fd); }
int _closedir_ (DIR *__dirp) { return closedir(__dirp); }
void __exit_ (int __status) { _exit(__status); }
int _fsync_ (int __fd) { return fsync(__fd); }
char *_getenv_ (const char *__name) { return getenv(__name); }
int _isatty_ (int __fd) { return isatty(__fd); }
__off_t _lseek_ (int __fd, __off_t __offset, int __whence)  { return lseek(__fd, __offset, __whence); }
int _mkdir_ (const char *__path, __mode_t __mode) { return mkdir(__path, __mode); }
int _open_(const char *pathname, int flags, ...) { return open(pathname, flags, 0666); }
DIR *_opendir_ (const char *__name) { return opendir (__name); }
ssize_t _read_ (int __fd, void *__buf, size_t __nbytes) { return read(__fd, __buf, __nbytes); }
ssize_t _write_ (int __fd, const void *__buf, size_t __n) { return write(__fd, __buf, __n); }
struct dirent *_readdir_ (DIR *__dirp) { return readdir(__dirp); }
int _stat_ (const char *__restrict __file,   struct stat *__restrict __buf) { return stat(__file, __buf); }
int _gettimeofday_(struct timeval *tv, struct timezone *tz) { return gettimeofday(tv, tz); }
int _sigemptyset_(sigset_t *set) { return sigemptyset(set); }
int _sigaction_(int signum, const struct sigaction *act, struct sigaction *oldact)
{
  return sigaction(signum, act, oldact);
}

typedef void (*sighandler_t)(int);
sighandler_t _signal_(int signum, sighandler_t handler) { return signal(signum, handler); }
char *_strerror_(int errnum) { return strerror(errnum); }

int _tcgetattr_(int fd, struct termios *termios_p) { return tcgetattr(fd, termios_p); }
int _tcsetattr_(int fd, int optional_actions, const struct termios *termios_p)
{
  return tcsetattr(fd, optional_actions, termios_p);
}

int _unlink_(const char *pathname) { return unlink(pathname); }
char *_realpath_(const char *path, char *resolved_path) { return realpath(path, resolved_path); }

STATIC void sighandler(int signum) {
    if (signum == SIGINT) {
        #if MICROPY_ASYNC_KBD_INTR
        mp_obj_exception_clear_traceback(MP_OBJ_FROM_PTR(&MP_STATE_VM(mp_kbd_exception)));
        sigset_t mask;
        sigemptyset(&mask);
        // On entry to handler, its signal is blocked, and unblocked on
        // normal exit. As we instead perform longjmp, unblock it manually.
        sigprocmask(SIG_SETMASK, &mask, NULL);
        nlr_raise(MP_OBJ_FROM_PTR(&MP_STATE_VM(mp_kbd_exception)));
        #else
        if (MP_STATE_VM(mp_pending_exception) == MP_OBJ_FROM_PTR(&MP_STATE_VM(mp_kbd_exception))) {
            // this is the second time we are called, so die straight away
            exit(1);
        }
        mp_obj_exception_clear_traceback(MP_OBJ_FROM_PTR(&MP_STATE_VM(mp_kbd_exception)));
        MP_STATE_VM(mp_pending_exception) = MP_OBJ_FROM_PTR(&MP_STATE_VM(mp_kbd_exception));
        #endif
    }
}

void mp_hal_set_interrupt_char(char c) {
    // configure terminal settings to (not) let ctrl-C through
    if (c == CHAR_CTRL_C) {
        #ifndef _WIN32
        // enable signal handler
        struct sigaction sa;
        sa.sa_flags = 0;
        sa.sa_handler = sighandler;
        sigemptyset(&sa.sa_mask);
        sigaction(SIGINT, &sa, NULL);
        #endif
    } else {
        #ifndef _WIN32
        // disable signal handler
        struct sigaction sa;
        sa.sa_flags = 0;
        sa.sa_handler = SIG_DFL;
        sigemptyset(&sa.sa_mask);
        sigaction(SIGINT, &sa, NULL);
        #endif
    }
}

#if MICROPY_USE_READLINE == 1

static struct termios orig_termios;

void mp_hal_stdio_mode_raw(void) {
    // save and set terminal settings
    tcgetattr(0, &orig_termios);
    static struct termios termios;
    termios = orig_termios;
    termios.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    termios.c_cflag = (termios.c_cflag & ~(CSIZE | PARENB)) | CS8;
    termios.c_lflag = 0;
    termios.c_cc[VMIN] = 1;
    termios.c_cc[VTIME] = 0;
    tcsetattr(0, TCSAFLUSH, &termios);
}

void mp_hal_stdio_mode_orig(void) {
    // restore terminal settings
    tcsetattr(0, TCSAFLUSH, &orig_termios);
}

#endif

#if MICROPY_PY_OS_DUPTERM
static int call_dupterm_read(size_t idx) {
    nlr_buf_t nlr;
    if (nlr_push(&nlr) == 0) {
        mp_obj_t read_m[3];
        mp_load_method(MP_STATE_VM(dupterm_objs[idx]), MP_QSTR_read, read_m);
        read_m[2] = MP_OBJ_NEW_SMALL_INT(1);
        mp_obj_t res = mp_call_method_n_kw(1, 0, read_m);
        if (res == mp_const_none) {
            return -2;
        }
        mp_buffer_info_t bufinfo;
        mp_get_buffer_raise(res, &bufinfo, MP_BUFFER_READ);
        if (bufinfo.len == 0) {
            mp_printf(&mp_plat_print, "dupterm: EOF received, deactivating\n");
            MP_STATE_VM(dupterm_objs[idx]) = MP_OBJ_NULL;
            return -1;
        }
        nlr_pop();
        return *(byte*)bufinfo.buf;
    } else {
        // Temporarily disable dupterm to avoid infinite recursion
        mp_obj_t save_term = MP_STATE_VM(dupterm_objs[idx]);
        MP_STATE_VM(dupterm_objs[idx]) = NULL;
        mp_printf(&mp_plat_print, "dupterm: ");
        mp_obj_print_exception(&mp_plat_print, nlr.ret_val);
        MP_STATE_VM(dupterm_objs[idx]) = save_term;
    }

    return -1;
}
#endif

int mp_hal_stdin_rx_chr(void) {
    unsigned char c;
#if MICROPY_PY_OS_DUPTERM
    // TODO only support dupterm one slot at the moment
    if (MP_STATE_VM(dupterm_objs[0]) != MP_OBJ_NULL) {
        int c;
        do {
             c = call_dupterm_read(0);
        } while (c == -2);
        if (c == -1) {
            goto main_term;
        }
        if (c == '\n') {
            c = '\r';
        }
        return c;
    } else {
        main_term:;
#endif
        int ret = read(0, &c, 1);
        if (ret == 0) {
            c = 4; // EOF, ctrl-D
        } else if (c == '\n') {
            c = '\r';
        }
        return c;
#if MICROPY_PY_OS_DUPTERM
    }
#endif
}

void mp_hal_stdout_tx_strn(const char *str, size_t len) {
    int ret = write(1, str, len);
    mp_uos_dupterm_tx_strn(str, len);
    (void)ret; // to suppress compiler warning
}

mp_uint_t mp_hal_ticks_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000 + tv.tv_usec / 1000;
}

mp_uint_t mp_hal_ticks_us(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * 1000000 + tv.tv_usec;
}
