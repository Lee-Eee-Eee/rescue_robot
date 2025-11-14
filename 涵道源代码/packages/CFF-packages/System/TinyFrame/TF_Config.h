#ifndef TF_CONFIG_H
#define TF_CONFIG_H

#include <stdint.h>
#include <rtthread.h>

//----------------------------- FRAME FORMAT ---------------------------------
// The format can be adjusted to fit your particular application needs

// If the connection is reliable, you can disable the SOF byte and checksums.
// That can save up to 9 bytes of overhead.

// ,-----+-----+-----+------+------------+- - - -+-------------,                
// | SOF | ID  | LEN | TYPE | HEAD_CKSUM | DATA  | DATA_CKSUM  |                
// | 0-1 | 1-4 | 1-4 | 1-4  | 0-4        | ...   | 0-4         | <- size (bytes)
// '-----+-----+-----+------+------------+- - - -+-------------'                

// !!! BOTH PEERS MUST USE THE SAME SETTINGS !!!

// Adjust sizes as desired (1,2,4)
#define TF_ID_BYTES     TINYFRAME_CONFIG_ID_BYTES
#define TF_LEN_BYTES    TINYFRAME_CONFIG_LEN_BYTES
#define TF_TYPE_BYTES   TINYFRAME_CONFIG_TYPE_BYTES

// Checksum type. Options:
//   TF_CKSUM_NONE, TF_CKSUM_XOR, TF_CKSUM_CRC8, TF_CKSUM_CRC16, TF_CKSUM_CRC32
//   TF_CKSUM_CUSTOM8, TF_CKSUM_CUSTOM16, TF_CKSUM_CUSTOM32
// Custom checksums require you to implement checksum functions (see TinyFrame.h)
#if defined(TINYFRAME_CONFIG_CKSUM_NONE)
#define TF_CKSUM_TYPE TF_CKSUM_CRC16
#elif defined(TINYFRAME_CONFIG_CKSUM_XOR)
#define TF_CKSUM_TYPE TF_CKSUM_XOR
#elif defined(TINYFRAME_CONFIG_CKSUM_CRC8)
#define TF_CKSUM_TYPE TF_CKSUM_CRC8
#elif defined(TINYFRAME_CONFIG_CKSUM_CRC16)
#define TF_CKSUM_TYPE TF_CKSUM_CRC16
#elif defined(TINYFRAME_CONFIG_CKSUM_CRC32)
#define TF_CKSUM_TYPE TF_CKSUM_CRC32
#elif defined(TINYFRAME_CONFIG_CKSUM_CUSTOM8)
#define TF_CKSUM_TYPE TF_CKSUM_CUSTOM8
#elif defined(TINYFRAME_CONFIG_CKSUM_CUSTOM16)
#define TF_CKSUM_TYPE TF_CKSUM_CUSTOM16
#elif defined(TINYFRAME_CONFIG_CKSUM_CUSTOM32)
#define TF_CKSUM_TYPE TF_CKSUM_CUSTOM32
#endif

// Use a SOF byte to mark the start of a frame
#ifdef TINYFRAME_CONFIG_USE_SOF
#define TF_USE_SOF_BYTE 1
// Value of the SOF byte (if TF_USE_SOF_BYTE == 1)
#define TF_SOF_BYTE     TINYFRAME_CONFIG_SOF_BYTE
#endif
//----------------------- PLATFORM COMPATIBILITY ----------------------------

// used for timeout tick counters - should be large enough for all used timeouts
typedef uint16_t TF_TICKS;

// used in loops iterating over listeners
typedef uint8_t TF_COUNT;

//----------------------------- PARAMETERS ----------------------------------

// Maximum received payload size (static buffer)
// Larger payloads will be rejected.
#define TF_MAX_PAYLOAD_RX TINYFRAME_CONFIG_MAX_PAYLOAD_RX
// Size of the sending buffer. Larger payloads will be split to pieces and sent
// in multiple calls to the write function. This can be lowered to reduce RAM usage.
#define TF_SENDBUF_LEN    TINYFRAME_CONFIG_SENDBUF_LEN

// --- Listener counts - determine sizes of the static slot tables ---

// Frame ID listeners (wait for response / multi-part message)
#define TF_MAX_ID_LST   TINYFRAME_CONFIG_MAX_ID_LST
// Frame Type listeners (wait for frame with a specific first payload byte)
#define TF_MAX_TYPE_LST TINYFRAME_CONFIG_MAX_TYPE_LST
// Generic listeners (fallback if no other listener catches it)
#define TF_MAX_GEN_LST  TINYFRAME_CONFIG_MAX_GEN_LST

// Timeout for receiving & parsing a frame
// ticks = number of calls to TF_Tick()
#define TF_PARSER_TIMEOUT_TICKS TINYFRAME_CONFIG_PARSER_TIMEOUT_TICKS

// Whether to use mutex - requires you to implement TF_ClaimTx() and TF_ReleaseTx()
#ifdef TINYFRAME_CONFIG_USE_MUTEX
#define TF_USE_MUTEX  1
#else
#define TF_USE_MUTEX  0
#endif

// Error reporting function. To disable debug, change to empty define
#define TF_Error(format, ...) rt_kprintf("[TinyFrame] " format "\n", ##__VA_ARGS__)

//------------------------- End of user config ------------------------------

#endif //TF_CONFIG_H
