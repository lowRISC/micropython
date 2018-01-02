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
#include "stubs.h"
#include "py/mpconfig.h"
#include "py/mphal.h"
#include "py/runtime.h"
#include "extmod/misc.h"
#include "extmod/vfs_fat.h"

typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;

static int mapfd;
fs_user_mount_t fs_user_mount_sd;
#define SDCARD_BLOCK_SIZE (512)

int mount_and_check(void);
void sdcard_init(void);
extern uint8_t uart_recv();
extern void uart_send_buf(const char *buf, const int32_t len);
mp_uint_t sdcard_read_blocks(uint8_t *dest, uint32_t block_num, uint32_t num_blocks);
mp_uint_t sdcard_write_blocks(const uint8_t *src, uint32_t block_num, uint32_t num_blocks);

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

struct termios;

FIL *files[32];

#undef exit

bool sdcard_is_present(void) {
  return 1;
}

bool sdcard_power_on(void) {
  if (!mapfd)
    {
      sdcard_init();
    }
  return true;
}

void sdcard_power_off(void) {
}

uint64_t sdcard_get_capacity_in_bytes(void) {
    return 1<<20;
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

static int my_fr;

int mount_and_check(void)
{
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
      my_fr = f_mount(&vfs_fat->fatfs);
      if (my_fr)
	{
	  printm("Mount failed\n");
          return -1;
	}
    }
  return 0;
}

int mount_and_get_fd(void)
{
  int fd = 3;
  if (mount_and_check())
    return -1;
  while (files[fd])
    ++fd;
  files[fd] = calloc(1, sizeof(FIL));
  return fd;
}

int _close_ (int __fd)
{
  my_fr = f_close(files[__fd]);
  if (my_fr) return -1;
  free(files[__fd]);
  files[__fd] = 0;
  return 0;
}

char *_getenv_ (const char *__name) { return NULL; }
int _isatty_ (int __fd) { return __fd < 3; }

__off_t _lseek_ (int __fd, __off_t __offset, int __whence)
{
  switch(__whence)
    {
    case SEEK_SET:
      my_fr = f_lseek(files[__fd], __offset);
      if (my_fr == FR_OK)
	return __offset;
      else
	return -1;
    case SEEK_CUR:
      return -1;
    case SEEK_END:
      return -1;
    default:
      return -1;
    }
}

int _open_(const char *pathname, int flags, ...) {
  int fa;
  fs_user_mount_t *vfs_fat = &fs_user_mount_sd;
  int fd = mount_and_get_fd();
  if (fd < 0) return -1;
  switch(flags & O_ACCMODE)
    {
    case O_RDONLY: fa = FA_READ; break;
    case O_WRONLY: fa = FA_WRITE; break;
    case O_RDWR: fa = FA_READ|FA_WRITE; break;
    default: fa = 0; break;
    }
  if (flags & O_APPEND) fa |= FA_OPEN_APPEND;
  if (flags & O_CREAT) fa |= FA_CREATE_NEW;
  if ((flags & O_CREAT) && (flags & O_TRUNC))
    {
    my_fr = f_unlink(&vfs_fat->fatfs, pathname);
    if ((my_fr != FR_OK) && (my_fr != FR_NO_FILE)) return -1;
    }
  my_fr = f_open(&vfs_fat->fatfs, files[fd], pathname, fa);
  if (my_fr) return -1;
  return fd;
}

FF_DIR *opendir(const char *path)
{
  fs_user_mount_t *vfs_fat = &fs_user_mount_sd;
  FF_DIR *dir = calloc(1, sizeof(FF_DIR));
  if (mount_and_check())
    return 0;
  my_fr = f_opendir(&vfs_fat->fatfs, dir, path);
  if (my_fr) return 0;
  return dir;
}

struct dirent *readdir(FF_DIR *dir)
{
  static struct dirent retval;  
  static FILINFO fno;
  memset(&fno, 0, sizeof(fno));
  my_fr = f_readdir(dir, &fno);
  if (my_fr || !fno.fname[0])
    {
      return 0;
    }
  retval.d_name = fno.fname;
  return &retval;
}

void closedir(FF_DIR *dir)
{
  my_fr = f_closedir(dir);
  if (my_fr == FR_OK)
    free(dir);
}

ssize_t _read_ (int __fd, void *__buf, size_t __nbytes)
{
  UINT br;
  switch(__fd)
    {
    case 0:
      *(char *)__buf = mp_hal_stdin_rx_chr();
      return 1;
      break;
    case 1:
    case 2:
      return -1;
    default:
      my_fr = f_read(files[__fd], __buf, __nbytes, &br);
      if (my_fr) return -1;  
      printm("read(%d,%p,%d);\n", __fd, __buf, __nbytes);
      return br;
    }
}

ssize_t _write_ (int __fd, const void *__buf, size_t __n)
{
  UINT br;
  switch(__fd)
    {
    case 1:
    case 2:
      mp_hal_stdout_tx_strn(__buf, __n);
      return __n;
      break;
    default:
      my_fr = f_write(files[__fd], __buf, __n, &br);
      if (my_fr) return -1;  
      printm("write(%d,%p,%d);\n", __fd, __buf, __n);
      return br;
    }
}

int _stat_ (const char *__restrict __file,   struct stat *__restrict __buf)
{
  FILINFO fno;
  fs_user_mount_t *vfs_fat = &fs_user_mount_sd;
  if (mount_and_check())
    return -1;
  my_fr = f_stat(&vfs_fat->fatfs, __file, &fno);
  if (my_fr) return -1;
  memset(__buf, 0, sizeof(struct stat));
  __buf->st_size = fno.fsize;
  __buf->st_mtime = fno.fdate;
  __buf->st_mtime = fno.ftime;
  __buf->st_mode = fno.fattrib;
  return 0;
}

int _mkdir_ (const char *__path, __mode_t __mode)
{
  fs_user_mount_t *vfs_fat = &fs_user_mount_sd;
  if (mount_and_check())
    return -1;
  my_fr = f_mkdir(&vfs_fat->fatfs, __path);
  if (my_fr) return -1;
  return 0;
}

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

int _unlink_(const char *pathname)
{
  fs_user_mount_t *vfs_fat = &fs_user_mount_sd;
  if (mount_and_check())
    return -1;
  my_fr = f_unlink(&vfs_fat->fatfs, pathname);
  if (my_fr) return -1;
  return 0;
}

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

void mp_hal_stdout_tx_str(const char *str) {
    mp_hal_stdout_tx_strn(str, strlen(str));
}

// cooked is same as uncooked because the terminal does some postprocessing
void mp_hal_stdout_tx_strn_cooked(const char *str, size_t len) {
    mp_hal_stdout_tx_strn(str, len);
}

void _abort_ (void) { abort(); }

#ifdef __x86_64

#include <sys/mman.h>

static void *mapping;

void sdcard_init(void) {
#undef open  
#undef lseek
  int open(const char *pathname, int flags, mode_t mode) ;
  __off_t lseek (int __fd, __off_t __offset, int __whence);
  mapfd = open("/var/tmp/sdcard.img", O_RDWR, 0666);
  if (mapfd > 0)
    {
      int len = lseek(mapfd, 0, SEEK_END);
      printm("Mapping length %d bytes\n", len);
      mapping = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED, mapfd, 0);
    }
}

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

#else
#include "sd_diskio.h"
#include "extmod/vfs_fat.h"

mp_uint_t sdcard_read_blocks(uint8_t *dest, uint32_t block_num, uint32_t num_blocks) {
    fs_user_mount_t *vfs_fat = &fs_user_mount_sd;
    DRESULT rslt = sd_disk_read(0, dest, block_num, num_blocks);
    return rslt == RES_OK ? HAL_OK : HAL_ERROR;
}

mp_uint_t sdcard_write_blocks(const uint8_t *src, uint32_t block_num, uint32_t num_blocks) {
    fs_user_mount_t *vfs_fat = &fs_user_mount_sd;
    DRESULT rslt = sd_disk_write(0, src, block_num, num_blocks);
    return rslt == RES_OK ? HAL_OK : HAL_ERROR;
}

void sdcard_init(void)
{
  mapfd = 1024; // an invalid value
  printm("sdcard init\n");
  sd_disk_initialize(0);
}

void *_sbrk_(intptr_t siz)
{
  static uint8_t *sbrk = (uint8_t *)0x87000000;
  void *ptr = sbrk;
  if (siz < 0) siz = 0;
  printm("sbrk(%d) returned %p\n", siz, ptr);
  sbrk += ((siz-1)|7)+1;
  return ptr;
}
#undef abort

void abort(void)
{
  while (1);
}

int *__errno(void)
{
  return &my_fr;
}

#endif
