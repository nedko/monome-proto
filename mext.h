/* -*- Mode: C ; c-basic-offset: 2 -*- */
/*****************************************************************************
 *
 * Copyright (c) 2013 Nedko Arnaudov <nedko@arnaudov.name>
 *
 * Definitions of structures that can be used for implementing mext protocol
 * encoders and decoders
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

#ifndef MEXT_H__BD2E6747_BDD5_477D_AC49_E01D6223A705__INCLUDED
#define MEXT_H__BD2E6747_BDD5_477D_AC49_E01D6223A705__INCLUDED

#define _mext_packed_ __attribute__ ((packed))

#define MEXT_SECTION_SYSTEM      0x0
#define MEXT_SECTION_LED_GRID    0x1
#define MEXT_SECTION_KEY_GRID    0x2
#define MEXT_SECTION_DIGITAL_OUT 0x3
#define MEXT_SECTION_DIGITAL_IN  0x4
#define MEXT_SECTION_ENCODER     0x5
#define MEXT_SECTION_ANALOG_IN   0x6
#define MEXT_SECTION_ANALOG_OUT  0x7
#define MEXT_SECTION_TILT        0x8
#define MEXT_SECTION_LED_RING    0x9

/* to device */
#define MEXT_COMMAND_SYSTEM_QUERY       0x0
#define MEXT_COMMAND_SYSTEM_QUERY_PAYLOAD_SIZE 0
#define MEXT_COMMAND_SYSTEM_GET_ID      0x1
#define MEXT_COMMAND_SYSTEM_GET_ID_PAYLOAD_SIZE 0
#define MEXT_COMMAND_SYSTEM_SET_ID      0x2
#define MEXT_COMMAND_SYSTEM_SET_ID_PAYLOAD_SIZE 32
#define MEXT_COMMAND_SYSTEM_GET_OFFSETS 0x3
#define MEXT_COMMAND_SYSTEM_GET_OFFSETS_PAYLOAD_SIZE 0
#define MEXT_COMMAND_SYSTEM_SET_OFFSET  0x4
#define MEXT_COMMAND_SYSTEM_SET_OFFSET_PAYLOAD_SIZE 3
#define MEXT_COMMAND_SYSTEM_GET_GRIDSZ  0x5
#define MEXT_COMMAND_SYSTEM_GET_GRIDSZ_PAYLOAD_SIZE 0
#define MEXT_COMMAND_SYSTEM_SET_GRIDSZ  0x6
#define MEXT_COMMAND_SYSTEM_SET_GRIDSZ_PAYLOAD_SIZE 2
#define MEXT_COMMAND_SYSTEM_GET_ADDR    0x7
#define MEXT_COMMAND_SYSTEM_GET_ADDR_PAYLOAD_SIZE 0
#define MEXT_COMMAND_SYSTEM_SET_ADDR    0x8
#define MEXT_COMMAND_SYSTEM_SET_ADDR_PAYLOAD_SIZE 2
#define MEXT_COMMAND_SYSTEM_GET_VERSION 0xF
#define MEXT_COMMAND_SYSTEM_GET_VERSION_PAYLOAD_SIZE 0

/* from device */
#define MEXT_RESPONSE_SYSTEM_QUERY          0x0
#define MEXT_RESPONSE_SYSTEM_QUERY_PAYLOAD_SIZE 2
#define MEXT_RESPONSE_SYSTEM_ID             0x1
#define MEXT_RESPONSE_SYSTEM_ID_PAYLOAD_SIZE 32
#define MEXT_RESPONSE_SYSTEM_GRID_OFFSET    0x2
#define MEXT_RESPONSE_SYSTEM_GRID_OFFSET_PAYLOAD_SIZE 3
#define MEXT_RESPONSE_SYSTEM_GRIDSZ         0x3
#define MEXT_RESPONSE_SYSTEM_GRIDSZ_PAYLOAD_SIZE 2
#define MEXT_RESPONSE_SYSTEM_ADDR           0x4
#define MEXT_RESPONSE_SYSTEM_ADDR_PAYLOAD_SIZE 2
#define MEXT_RESPONSE_SYSTEM_VERSION        0xF
#define MEXT_RESPONSE_SYSTEM_VERSION_PAYLOAD_SIZE 8


struct mext_packet_payload_command_system_set_id
{
  uint8_t id[32];
} _mext_packed_;

struct mext_packet_payload_command_system_set_offset
{
  uint8_t grid_no;
  uint8_t x_offset;
  uint8_t y_offset;
} _mext_packed_;

struct mext_packet_payload_command_system_set_gridsz
{
  uint8_t x;
  uint8_t y;
} _mext_packed_;

struct mext_packet_payload_command_system_set_addr
{
  uint8_t addr;
  uint8_t value;
} _mext_packed_;

struct mext_packet_payload_response_system_query
{
  uint8_t section;
  uint8_t count;
} _mext_packed_;

struct mext_packet_payload_response_system_id
{
  uint8_t id[32];
} _mext_packed_;

struct mext_packet_payload_response_system_grid_offset
{
  uint8_t grid_no;
  uint8_t x_offset;
  uint8_t y_offset;
} _mext_packed_;

struct mext_packet_payload_response_system_gridsz
{
  uint8_t x;
  uint8_t y;
} _mext_packed_;

struct mext_packet_payload_response_system_addr
{
  uint8_t addr;
  uint8_t type;
} _mext_packed_;

struct mext_packet_payload_response_system_version
{
  uint8_t version[8];
} _mext_packed_;

#endif /* #ifndef MEXT_H__BD2E6747_BDD5_477D_AC49_E01D6223A705__INCLUDED */
