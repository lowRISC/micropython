/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/mman.h>
#include "stubs.h"
#include "py/mpconfig.h"
#include "py/mphal.h"
#include "py/runtime.h"
#include "extmod/misc.h"
#include "extmod/vfs_fat.h"

#ifdef __x86_64

void printm(const char* s, ...)
{
  char buf[256];
  va_list vl;

  va_start(vl, s);
  vsnprintf(buf, sizeof buf, s, vl);
  va_end(vl);

  mp_hal_stdout_tx_str(buf);
}

extern uint8_t uart_recv()
{
#undef read
  extern ssize_t read (int __fd, void *__buf, size_t __nbytes);
  unsigned char c;
  int ret = read(0, &c, 1);
  if (ret == 0) {
    c = 4; // EOF, ctrl-D
  } else if (c == '\n') {
    c = '\r';
  }
  return c;
}

extern void uart_send_buf(const char *str, const int32_t len)
{
#undef write
  extern ssize_t write (int __fd, const void *__buf, size_t __n);
  int ret = write(1, str, len);
  (void)ret; // to suppress compiler warning
}

#undef sbrk
void *sbrk (intptr_t __delta);
void *_sbrk_ (intptr_t __delta)
{
  if (__delta < 0) __delta = 0;
  void *ptr = sbrk(__delta);
  printm("sbrk(%d) returned %p\n", __delta, ptr);
  return ptr;
}

#else
extern uint8_t uart_recv();
extern void uart_send_buf(const char *buf, const int32_t len);

void *_sbrk_(intptr_t siz)
{
  static uint8_t *sbrk = (uint8_t *)0x87000000;
  void *ptr = sbrk;
  if (siz < 0) siz = 0;
  printm("sbrk(%d) returned %p\n", siz, ptr);
  sbrk += ((siz-1)|7)+1;
  return ptr;
}
#endif

#undef __x86_64
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

#else

void mp_hal_set_interrupt_char(char c)
{

}

void mp_hal_stdio_mode_raw(void)
{

}

void mp_hal_stdio_mode_orig(void)
{

}

/*
 * Core UART functions to implement for a port
 */

// Receive single character
int mp_hal_stdin_rx_chr(void)
{
  return uart_recv();
}

// Send string of given length
void mp_hal_stdout_tx_strn(const char *str, mp_uint_t len)
{
  uart_send_buf(str, len);
}

#include "lib/oofatfs/ff.h"
#define SDCARD_BLOCK_SIZE (512)

struct termios;
typedef void DIR;
fs_user_mount_t fs_user_mount_sd;

FIL *files[32];

#undef abort
#undef exit
#define abort() { printm("Abort in file "__FILE__" at line %d\n", __LINE__); while(1); }

static int mapfd;
static void *mapping;

typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

void sdcard_init(void);

bool sdcard_is_present(void) {
  return 1;
}

bool sdcard_power_on(void) {
  if (!mapfd)
    sdcard_init();
  return true;
}

void sdcard_power_off(void) {
}

uint64_t sdcard_get_capacity_in_bytes(void) {
    return 1<<20;
}

mp_uint_t sdcard_read_blocks(uint8_t *dest, uint32_t block_num, uint32_t num_blocks) {
    HAL_StatusTypeDef err = HAL_OK;
    memcpy(dest, block_num*512+(char*)mapping, num_blocks*512);
    return err;
}

mp_uint_t sdcard_write_blocks(const uint8_t *src, uint32_t block_num, uint32_t num_blocks) {
    HAL_StatusTypeDef err = HAL_OK;
    memcpy(block_num*512+(char*)mapping, src, num_blocks*512);
    return err;
}

STATIC mp_obj_t sd_present(mp_obj_t self) {
    return mp_obj_new_bool(sdcard_is_present());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sd_present_obj, sd_present);

STATIC mp_obj_t sd_power(mp_obj_t self, mp_obj_t state) {
    bool result;
    if (mp_obj_is_true(state)) {
        result = sdcard_power_on();
    } else {
        sdcard_power_off();
        result = true;
    }
    return mp_obj_new_bool(result);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(sd_power_obj, sd_power);

STATIC mp_obj_t sd_info(mp_obj_t self) {
    mp_obj_t tuple[3] = {
      mp_obj_new_int_from_ull(1<<20),
        mp_obj_new_int_from_uint(512),
        mp_obj_new_int(1),
    };
    return mp_obj_new_tuple(3, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(sd_info_obj, sd_info);

// now obsolete, kept for backwards compatibility
STATIC mp_obj_t sd_read(mp_obj_t self, mp_obj_t block_num) {
    uint8_t *dest = m_new(uint8_t, SDCARD_BLOCK_SIZE);
    mp_uint_t ret = sdcard_read_blocks(dest, mp_obj_get_int(block_num), 1);

    if (ret != 0) {
        m_del(uint8_t, dest, SDCARD_BLOCK_SIZE);
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_Exception, "sdcard_read_blocks failed [%u]", ret));
    }

    return mp_obj_new_bytearray_by_ref(SDCARD_BLOCK_SIZE, dest);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(sd_read_obj, sd_read);

// now obsolete, kept for backwards compatibility
STATIC mp_obj_t sd_write(mp_obj_t self, mp_obj_t block_num, mp_obj_t data) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(data, &bufinfo, MP_BUFFER_READ);
    if (bufinfo.len % SDCARD_BLOCK_SIZE != 0) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "writes must be a multiple of %d bytes", SDCARD_BLOCK_SIZE));
    }

    mp_uint_t ret = sdcard_write_blocks(bufinfo.buf, mp_obj_get_int(block_num), bufinfo.len / SDCARD_BLOCK_SIZE);

    if (ret != 0) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_Exception, "sdcard_write_blocks failed [%u]", ret));
    }

    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(sd_write_obj, sd_write);

STATIC mp_obj_t pyb_sdcard_readblocks(mp_obj_t self, mp_obj_t block_num, mp_obj_t buf) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_WRITE);
    mp_uint_t ret = sdcard_read_blocks(bufinfo.buf, mp_obj_get_int(block_num), bufinfo.len / SDCARD_BLOCK_SIZE);
    return mp_obj_new_bool(ret == 0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_sdcard_readblocks_obj, pyb_sdcard_readblocks);

STATIC mp_obj_t pyb_sdcard_writeblocks(mp_obj_t self, mp_obj_t block_num, mp_obj_t buf) {
    mp_buffer_info_t bufinfo;
    mp_get_buffer_raise(buf, &bufinfo, MP_BUFFER_READ);
    mp_uint_t ret = sdcard_write_blocks(bufinfo.buf, mp_obj_get_int(block_num), bufinfo.len / SDCARD_BLOCK_SIZE);
    return mp_obj_new_bool(ret == 0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_sdcard_writeblocks_obj, pyb_sdcard_writeblocks);

STATIC mp_obj_t pyb_sdcard_ioctl(mp_obj_t self, mp_obj_t cmd_in, mp_obj_t arg_in) {
    mp_int_t cmd = mp_obj_get_int(cmd_in);
    switch (cmd) {
        case BP_IOCTL_INIT:
            if (!sdcard_power_on()) {
                return MP_OBJ_NEW_SMALL_INT(-1); // error
            }
            return MP_OBJ_NEW_SMALL_INT(0); // success

        case BP_IOCTL_DEINIT:
            sdcard_power_off();
            return MP_OBJ_NEW_SMALL_INT(0); // success

        case BP_IOCTL_SYNC:
            // nothing to do
            return MP_OBJ_NEW_SMALL_INT(0); // success

        case BP_IOCTL_SEC_COUNT:
            return MP_OBJ_NEW_SMALL_INT(0); // TODO

        case BP_IOCTL_SEC_SIZE:
            return MP_OBJ_NEW_SMALL_INT(SDCARD_BLOCK_SIZE);

        default: // unknown command
            return MP_OBJ_NEW_SMALL_INT(-1); // error
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(pyb_sdcard_ioctl_obj, pyb_sdcard_ioctl);

STATIC const mp_rom_map_elem_t pyb_sdcard_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_present), MP_ROM_PTR(&sd_present_obj) },
    { MP_ROM_QSTR(MP_QSTR_power), MP_ROM_PTR(&sd_power_obj) },
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&sd_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_read), MP_ROM_PTR(&sd_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_write), MP_ROM_PTR(&sd_write_obj) },
    // block device protocol
    { MP_ROM_QSTR(MP_QSTR_readblocks), MP_ROM_PTR(&pyb_sdcard_readblocks_obj) },
    { MP_ROM_QSTR(MP_QSTR_writeblocks), MP_ROM_PTR(&pyb_sdcard_writeblocks_obj) },
    { MP_ROM_QSTR(MP_QSTR_ioctl), MP_ROM_PTR(&pyb_sdcard_ioctl_obj) },
};

STATIC MP_DEFINE_CONST_DICT(pyb_sdcard_locals_dict, pyb_sdcard_locals_dict_table);

STATIC mp_obj_t pyb_sdcard_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args);

const mp_obj_type_t pyb_sdcard_type = {
    { &mp_type_type },
    .name = MP_QSTR_SDCard,
    .make_new = pyb_sdcard_make_new,
    .locals_dict = (mp_obj_dict_t*)&pyb_sdcard_locals_dict,
};

const mp_obj_base_t pyb_sdcard_obj = {&pyb_sdcard_type};

STATIC mp_obj_t pyb_sdcard_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 0, 0, false);

    // return singleton object
    return (mp_obj_t)&pyb_sdcard_obj;
}

int mount_and_get_fd(void)
{
  int fd = 3;
  fs_user_mount_t *vfs_fat = &fs_user_mount_sd;
  if (!vfs_fat->flags)
    {
      int part = 1;
      vfs_fat->flags = FSUSER_NATIVE|FSUSER_HAVE_IOCTL;
      vfs_fat->fatfs.drv = vfs_fat;
      vfs_fat->fatfs.part = part;
      vfs_fat->readblocks[0] = (mp_obj_t)&pyb_sdcard_readblocks_obj;
      vfs_fat->readblocks[1] = (mp_obj_t)&pyb_sdcard_obj;
      vfs_fat->readblocks[2] = (mp_obj_t)sdcard_read_blocks; // native version
      vfs_fat->writeblocks[0] = (mp_obj_t)&pyb_sdcard_writeblocks_obj;
      vfs_fat->writeblocks[1] = (mp_obj_t)&pyb_sdcard_obj;
      vfs_fat->writeblocks[2] = (mp_obj_t)sdcard_write_blocks; // native version
      vfs_fat->u.ioctl[0] = (mp_obj_t)&pyb_sdcard_ioctl_obj;
      vfs_fat->u.ioctl[1] = (mp_obj_t)&pyb_sdcard_obj;
      // try to mount the flash
      FRESULT res = f_mount(&vfs_fat->fatfs);
      if (res)
	{
	  printm("Mount failed\n");
	  abort();
	}
    }
  while (files[fd])
    ++fd;
  files[fd] = calloc(1, sizeof(FIL));
  return fd;
}

void _abort_ (void) { abort(); }
int _close_ (int __fd) { abort(); }
int _closedir_ (DIR *__dirp) { abort(); }
char *_getenv_ (const char *__name) { return NULL; }
int _isatty_ (int __fd) { return __fd < 3; }
__off_t _lseek_ (int __fd, __off_t __offset, int __whence)  { abort(); }
int _mkdir_ (const char *__path, __mode_t __mode) { abort(); }
int _open_(const char *pathname, int flags, ...) {
  FRESULT fr;
  fs_user_mount_t *vfs_fat = &fs_user_mount_sd;
  int fd = mount_and_get_fd();
  fr = f_open(&vfs_fat->fatfs, files[fd], pathname, FA_READ);
  if (fr) return -1;
  return fd;
}
DIR *_opendir_ (const char *__name) { abort(); }

ssize_t _read_ (int __fd, void *__buf, size_t __nbytes)
{
  FRESULT fr;
  UINT br;
  fr = f_read(files[__fd], __buf, __nbytes, &br);
  if (fr) return -1;  
  //  printm("read(%d,%p,%d);\n", __fd, __buf, __nbytes);
  return br;
}

ssize_t _write_ (int __fd, const void *__buf, size_t __n)
{
  FRESULT fr;
  UINT br;
  switch(__fd)
    {
    case 1:
    case 2:
      mp_hal_stdout_tx_strn(__buf, __n);
      return __n;
      break;
    default:
      fr = f_write(files[__fd], __buf, __n, &br);
      if (fr) return -1;  
      printm("write(%d,%p,%d);\n", __fd, __buf, __n);
      return br;
    }
}

struct dirent *_readdir_ (DIR *__dirp) { abort(); }
int _stat_ (const char *__restrict __file,   struct stat *__restrict __buf) { abort(); }
//int _gettimeofday_(struct timeval *tv, struct timezone *tz) { abort(); }
int _sigemptyset_(sigset_t *set) { abort(); }
int _sigaction_(int signum, const struct sigaction *act, struct sigaction *oldact)
{
  abort();
}

typedef void (*sighandler_t)(int);
sighandler_t _signal_(int signum, sighandler_t handler) { return handler; }
char *_strerror_(int errnum) { abort(); }

int _tcgetattr_(int fd, struct termios *termios_p) { abort(); }
int _tcsetattr_(int fd, int optional_actions, const struct termios *termios_p)
{
  abort();
}

int _unlink_(const char *pathname) { abort(); }

char *_realpath_(const char *path, char *resolved_path)
{
  strcpy(resolved_path, path);
  return resolved_path;
}

int _fsync_(int fd)
{
  return 0;
}

void exit (int __status) { abort(); }
void _exit_ (int __status) { abort(); }
void __exit_ (int __status) { abort(); }

int __errno;

#undef abort

void abort(void)
{
  while (1);
}

#endif

void mp_hal_stdout_tx_str(const char *str) {
    mp_hal_stdout_tx_strn(str, strlen(str));
}

// cooked is same as uncooked because the terminal does some postprocessing
void mp_hal_stdout_tx_strn_cooked(const char *str, size_t len) {
    mp_hal_stdout_tx_strn(str, len);
}


void sdcard_init(void) {
#undef open  
#undef lseek
  int open(const char *pathname, int flags, mode_t mode) ;
  __off_t lseek (int __fd, __off_t __offset, int __whence);
  mapfd = open("sdcard.img", O_RDWR, 0666);
  if (mapfd > 0)
    {
      int len = lseek(mapfd, 0, SEEK_END);
      printm("Mapping length %d bytes\n", len);
      mapping = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, mapfd, 0);
    }
}
