// tegang_protocol.h : definisi paket ESP-NOW yang dipakai ROBOT dan REMOTE.
// File ini HARUS identik di kedua sketch (salin ke folder robot_tegang dan remote_tegang).
#pragma once
#include <stdint.h>
#include <stddef.h>

#define PKT_VER_REMOTE   1       // versi format paket remote -> robot
#define PKT_VER_STATUS   1       // versi format paket robot -> remote
#define ESPNOW_CHANNEL   1       // channel WiFi tetap (sama di robot dan remote)
#define LINK_TIMEOUT_MS  500     // tidak ada paket sepanjang ini = link putus

// Bit tombol di RemotePacket.buttons
#define BTN_SW1  0x01            // ARM / DISARM (ON = arm)
#define BTN_SW2  0x02            // E-STOP (ON = berhenti total)
#define BTN_SW3  0x04            // cadangan
#define BTN_SW4  0x08            // cadangan

// Bit flags di RobotStatusPacket.flags
#define FLG_BATT_WARN  0x01
#define FLG_OBST_FRONT 0x02
#define FLG_OBST_BACK  0x04
#define FLG_TUNING     0x08

enum RobotState : uint8_t { RS_DISARMED = 0, RS_ARMED = 1, RS_FAULT = 2 };
enum RobotFault : uint8_t {
  RF_NONE = 0, RF_FALL = 1, RF_STALL = 2, RF_SATURATED = 3,
  RF_ESTOP = 4, RF_LOWBATT = 5, RF_IMU = 6, RF_RUNAWAY = 7
};

// Remote -> Robot (14 byte)
typedef struct __attribute__((packed)) {
  uint8_t  ver;        // PKT_VER_REMOTE
  uint8_t  buttons;    // bit0..3 = SW1..SW4 (1 = ON)
  uint16_t seq;        // nomor urut, naik tiap paket
  int16_t  axes[4];    // -1000..1000 : [0]=LX [1]=LY(maju+) [2]=RX(kanan+) [3]=RY
  uint8_t  rbat_dV;    // tegangan baterai remote x10 (0 = tidak diukur)
  uint8_t  crc;        // CRC8 dari semua byte sebelumnya
} RemotePacket;

// Robot -> Remote (10 byte)
typedef struct __attribute__((packed)) {
  uint8_t  ver;        // PKT_VER_STATUS
  uint8_t  state;      // RobotState
  uint8_t  fault;      // RobotFault
  uint8_t  flags;      // FLG_*
  uint16_t vbat_cV;    // tegangan baterai robot x100 (0 = tidak terbaca)
  int16_t  pitch_cdeg; // sudut pitch x100 derajat
  uint8_t  crc;
} RobotStatusPacket;

static inline uint8_t tegangCrc8(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  while (n--) {
    c ^= *d++;
    for (int i = 0; i < 8; i++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

static inline const char* stateName(uint8_t s) {
  switch (s) { case RS_DISARMED: return "DISARMED"; case RS_ARMED: return "ARMED"; case RS_FAULT: return "FAULT"; }
  return "?";
}
static inline const char* faultName(uint8_t f) {
  switch (f) {
    case RF_NONE: return "-"; case RF_FALL: return "JATUH"; case RF_STALL: return "STALL";
    case RF_SATURATED: return "SATURASI"; case RF_ESTOP: return "E-STOP";
    case RF_LOWBATT: return "BATERAI-RENDAH"; case RF_IMU: return "IMU"; case RF_RUNAWAY: return "RUNAWAY";
  }
  return "?";
}
