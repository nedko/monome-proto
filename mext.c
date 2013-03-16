/* -*- Mode: C ; c-basic-offset: 2 -*- */
/*****************************************************************************
 *
 * Copyright (c) 2013 Nedko Arnaudov <nedko@arnaudov.name>
 *
 * test mext devices
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 *****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <assert.h>
#include <string.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>

#include "mext.h"

bool device_open(const char * device, speed_t speed, int * fd_ptr, int * error_ptr)
{
  struct termios params;
  int fd;

  fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd == -1)
  {
    *error_ptr = errno;
    return false;
  }

  if (tcgetattr(fd, &params) == -1)
  {
    *error_ptr = errno;
    close(fd);
    return false;
  }

  cfsetispeed(&params, speed);
  cfsetospeed(&params, speed);

  /* parity (8N1) */
  params.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
  params.c_cflag |=  (CS8 | CLOCAL | CREAD);

  /* no line processing */
  params.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | IEXTEN);

  /* raw input */
  params.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);

  /* raw output */
  params.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);

  params.c_cc[VMIN]  = 0;
  params.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &params) == -1)
  {
    *error_ptr = errno;
    close(fd);
    return false;
  }

  if (tcflush(fd, TCIOFLUSH) == -1)
  {
    *error_ptr = errno;
    close(fd);
    return false;
  }

  *fd_ptr = fd;
  return true;
}

uint8_t buffer[33];
size_t buffer_mark;

void device_read(int fd)
{
  ssize_t sret;
  size_t size;

  assert(sizeof(buffer) > buffer_mark);
  size = sizeof(buffer) - buffer_mark;
loop:
  sret = read(fd, buffer + buffer_mark, size);
  //debug("read(%d, %zu) returned %lld", fd, read_size, (signed long long)sret);
  if (sret > 0)
  {
    buffer_mark += sret;
    goto loop;
  }

  if (sret == 0) return;

  abort();
}


bool decode_query(uint8_t * buffer)
{
  printf(
    "sectionx=%u count=%u\n",
    buffer[1],
    buffer[2]);

  return true;
}

bool decode_id(uint8_t * buffer)
{
  char id[33];
  memcpy(id, buffer + 1, 32);
  id[32] = 0;
  printf("id='%s'\n", id);

  return true;
}

bool decode_xy(uint8_t * buffer)
{
  printf("%u; x=%u y=%u\n", buffer[0], buffer[1], buffer[2]);
  return true;
}

bool decode_key(uint8_t * buffer)
{
  printf(
    "key %s x=%u y=%u\n",
    MEXT_COMMAND(buffer[0]) == MEXT_RESPONSE_KEY_DOWN ? "down" : "up  ",
    buffer[1],
    buffer[2]);

  return true;
}

struct decoder_map_entry
{
  size_t payload_size;
  bool (* decoder)(uint8_t * buffer);
};

#define DECODER(section, cmd, decoder) \
  [MEXT_HEADER(MEXT_SECTION_ ## section, MEXT_RESPONSE_ ## cmd)] = \
  { MEXT_RESPONSE_ ## cmd ## _PAYLOAD_SIZE, decoder }

static struct decoder_map_entry decoder_map[256] =
{
//  [0 ... 255] = { 0, NULL },

  DECODER(SYSTEM, SYSTEM_QUERY, decode_query),
  DECODER(SYSTEM, SYSTEM_ID, decode_id),
  DECODER(SYSTEM, SYSTEM_GRIDSZ, decode_xy),
  DECODER(KEY_GRID, KEY_UP, decode_key),
  DECODER(KEY_GRID, KEY_DOWN, decode_key),
};

void decode(void)
{
  struct decoder_map_entry * entry_ptr;
loop:
  if (buffer_mark < 1) return;

  printf("%zu bytes; 0x%02X\n", buffer_mark, buffer[0]);

  entry_ptr = decoder_map + buffer[0];
  if (entry_ptr->decoder == NULL) abort(); /* unknown command */

  if (buffer_mark < entry_ptr->payload_size + 1) goto loop;

  entry_ptr->decoder(buffer);

  if (buffer_mark > entry_ptr->payload_size + 1)
  {
    memmove(
      buffer,
      buffer + entry_ptr->payload_size + 1,
      buffer_mark - entry_ptr->payload_size - 1);
  }
  buffer_mark -= entry_ptr->payload_size + 1;
  goto loop;
}

void loop(int fd)
{
  struct pollfd pollfd;
  int status;

  pollfd.fd = fd;
  pollfd.events = POLLIN | POLLERR;

poll:
  status = poll(&pollfd, 1, -1);
  //log_debug("poll() returned %d", status);
  if (status == 0)
  {
    /* poll() timed out when called for infinite timeout */
    abort();
  }

  if (status < 0)
  {
    if (errno == EINTR)
    {
      goto poll;
    }

    //log_error("poll() failed"); /* errno */
    abort();
  }

  if ((pollfd.revents & POLLHUP) != 0)
  {
    printf("device disconnected\n");
    return;
  }
  if ((pollfd.revents & POLLERR) != 0) abort();

  device_read(fd);
  decode();

  goto poll;
}

bool test(const char * device)
{
  int fd;
  int error;

  if (!device_open(device, B115200, &fd, &error))
  {
    printf("device not connected\n");
    return false;
  }

#if 0
  buffer[0] = MEXT_HEADER(MEXT_SECTION_SYSTEM, MEXT_COMMAND_SYSTEM_QUERY);
  write(fd, buffer, 1);

  buffer[0] = MEXT_HEADER(MEXT_SECTION_SYSTEM, MEXT_COMMAND_SYSTEM_GET_ID);
  write(fd, buffer, 1);
#endif

#if 0
  buffer[0] = MEXT_HEADER(MEXT_SECTION_SYSTEM, MEXT_COMMAND_SYSTEM_SET_ID);
  memset(buffer + 1, 0, 32);
  memcpy(buffer + 1, "trololo", 7);
  write(fd, buffer, 33);
#endif

#if 0
  buffer[0] = MEXT_HEADER(MEXT_SECTION_SYSTEM, MEXT_COMMAND_SYSTEM_GET_OFFSETS);
  write(fd, buffer, 1);
#endif

#if 0
  buffer[0] = MEXT_HEADER(MEXT_SECTION_SYSTEM, MEXT_COMMAND_SYSTEM_SET_GRIDSZ);
  buffer[1] = 8;
  buffer[2] = 8;
  write(fd, buffer, 3);
#endif

#if 0
  buffer[0] = MEXT_HEADER(MEXT_SECTION_SYSTEM, MEXT_COMMAND_SYSTEM_GET_GRIDSZ);
  write(fd, buffer, 1);
#endif

#if 0
  buffer[0] = MEXT_HEADER(MEXT_SECTION_SYSTEM, MEXT_COMMAND_SYSTEM_GET_ADDR);
  write(fd, buffer, 1);
#endif

#if 0
  buffer[0] = MEXT_HEADER(MEXT_SECTION_SYSTEM, MEXT_COMMAND_SYSTEM_GET_VERSION);
  write(fd, buffer, 1);
#endif

#if 1
  buffer[0] = MEXT_HEADER(MEXT_SECTION_LED_GRID, MEXT_COMMAND_LED_ALL_ON);
  write(fd, buffer, 1);
  usleep(1000000);
  buffer[0] = MEXT_HEADER(MEXT_SECTION_LED_GRID, MEXT_COMMAND_LED_ALL_OFF);
  write(fd, buffer, 1);
#endif

  loop(fd);

  close(fd);
  return false;
}

int main(int argc, char ** argv)
{
  if (argc < 2) return EXIT_FAILURE;

  if (!test(argv[1])) return EXIT_FAILURE;

  return EXIT_SUCCESS;
}
