#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Preferences.h>
#include <string.h>
#if defined(__has_include)
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif
#endif

namespace rclink {

enum : uint8_t {
  RC_DATA  = 0x01,
  FC_ALIVE = 0x10,
  FC_DATA  = 0x11,
  PAIR_REQ = 0xFE,
  PAIR_RES = 0xFF,
};

static const uint8_t WIFI_CH_MIN = 1;
static const uint8_t WIFI_CH_MAX = 13;
static const int PWM_MIN = 880;
static const int PWM_CENTER = 1500;
static const int PWM_MAX = 2120;
static const uint32_t DISCOVERY_HOP_MS = 200;
static const uint32_t RELOCK_AFTER_MS = 5000;
static const uint32_t SAVED_CH_DWELL_MS = 2500;
static const int MAX_DRONES = 8;
static const int RX_QUEUE_LEN = 16;
static const int RC_CHANNELS = 8;

struct __attribute__((packed)) MsgRc {
  uint8_t type = RC_DATA;
  int16_t ch1 = 1500;
  int16_t ch2 = 1500;
  int16_t ch3 = 1000;
  int16_t ch4 = 1500;
  int8_t ch5 = 0;
  int8_t ch6 = 0;
  int8_t ch7 = 0;
  int8_t ch8 = 0;
  uint8_t csum = 0;
};

struct __attribute__((packed)) MsgPairReq {
  uint8_t type;
  uint8_t channel;
  uint8_t csum;
};

struct __attribute__((packed)) MsgPairRes {
  uint8_t type = PAIR_RES;
  uint8_t csum = 0;
};

static_assert(sizeof(MsgRc) == 14, "RC frame must match espnow-rclink");
static_assert(sizeof(MsgPairReq) == 3, "pair request must match espnow-rclink");
static_assert(sizeof(MsgPairRes) == 2, "pair response must match espnow-rclink");

inline uint8_t checksum(const uint8_t *data, size_t len) {
  uint8_t c = 0x55;
  for (size_t i = 0; i < len; i++) c ^= data[i];
  return c;
}

template <typename M>
inline void seal(M &m) {
  m.csum = checksum(reinterpret_cast<const uint8_t *>(&m), sizeof(M) - 1);
}

inline int clampInt(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

inline int8_t encodeAux(int us) {
  int x = clampInt(us, PWM_MIN, PWM_MAX) - PWM_CENTER;
  int round = x > 0 ? 2 : -2;
  return (int8_t)((x + round) / 5);
}

inline bool macIsZero(const uint8_t *m) {
  for (int i = 0; i < 6; i++) if (m[i]) return false;
  return true;
}

inline void printMac(Print &p, const uint8_t *m) {
  for (int i = 0; i < 6; i++) {
    if (m[i] < 16) p.print('0');
    p.print(m[i], HEX);
    if (i < 5) p.print(':');
  }
}

class MultiTx {
 public:

  bool begin(const uint8_t (*macs)[6], int count, Print *log = nullptr) {
    _log = log;
    _count = count > MAX_DRONES ? MAX_DRONES : count;
    for (int i = 0; i < _count; i++) {
      memcpy(_slots[i].mac, macs[i], 6);
      _slots[i].learn = macIsZero(macs[i]);
      _slots[i].paired = false;
      _slots[i].lastHeardMs = 0;
    }
    _instance = this;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_ps(WIFI_PS_NONE);
    if (esp_now_init() != ESP_OK) {
      if (_log) _log->println("[rclink] esp_now_init failed");
      return false;
    }
    esp_now_register_recv_cb(&MultiTx::_onRecv);
    _lockedChannel = 0;

    _prefs.begin("rclink", false);
    _savedChannel = _prefs.getUChar("ch", 0);
    const bool haveSaved = _savedChannel >= WIFI_CH_MIN && _savedChannel <= WIFI_CH_MAX;
    _hopChannel = haveSaved ? _savedChannel : WIFI_CH_MIN;
    _setChannel(_hopChannel);
    _nextHopMs = millis() + (haveSaved ? SAVED_CH_DWELL_MS : DISCOVERY_HOP_MS);
    return true;
  }

  void update() {
    const uint32_t now = millis();
    _drainQueue(now);

    if (_lockedChannel == 0) {

      if ((int32_t)(now - _nextHopMs) >= 0) {
        _hopChannel = _hopChannel >= WIFI_CH_MAX ? WIFI_CH_MIN : _hopChannel + 1;
        _setChannel(_hopChannel);
        _nextHopMs = now + DISCOVERY_HOP_MS;
      }
    } else if (now - _lastAnyHeardMs > RELOCK_AFTER_MS) {

      if (_log) _log->println("[rclink] no drone heard for 5 s, rescanning channels");
      _lockedChannel = 0;
      for (int i = 0; i < _count; i++) _slots[i].paired = false;
      _nextHopMs = now;
    }
  }

  bool send(int slot, const uint16_t ch[RC_CHANNELS]) {
    if (slot < 0 || slot >= _count || !_slots[slot].paired) return false;
    MsgRc m;
    m.ch1 = (int16_t)clampInt(ch[0], PWM_MIN, PWM_MAX);
    m.ch2 = (int16_t)clampInt(ch[1], PWM_MIN, PWM_MAX);
    m.ch3 = (int16_t)clampInt(ch[2], PWM_MIN, PWM_MAX);
    m.ch4 = (int16_t)clampInt(ch[3], PWM_MIN, PWM_MAX);
    m.ch5 = encodeAux(ch[4]);
    m.ch6 = encodeAux(ch[5]);
    m.ch7 = encodeAux(ch[6]);
    m.ch8 = encodeAux(ch[7]);
    seal(m);
    return esp_now_send(_slots[slot].mac, reinterpret_cast<const uint8_t *>(&m), sizeof(m)) == ESP_OK;
  }

  bool paired(int slot) const { return slot >= 0 && slot < _count && _slots[slot].paired; }

  uint32_t heardAgeMs(int slot) const {
    if (!paired(slot)) return UINT32_MAX;
    return millis() - _slots[slot].lastHeardMs;
  }
  const uint8_t *mac(int slot) const { return _slots[slot].mac; }
  uint8_t channel() const { return _lockedChannel; }

 private:
  struct Slot {
    uint8_t mac[6];
    bool learn;
    bool paired;
    uint32_t lastHeardMs;
  };
  struct RxItem {
    uint8_t mac[6];
    uint8_t len;
    uint8_t data[sizeof(MsgRc)];
  };

  static MultiTx *_instance;

  Slot _slots[MAX_DRONES];
  int _count = 0;
  Print *_log = nullptr;
  uint8_t _lockedChannel = 0;
  uint8_t _hopChannel = WIFI_CH_MIN;
  uint32_t _nextHopMs = 0;
  uint32_t _lastAnyHeardMs = 0;
  uint8_t _savedChannel = 0;
  Preferences _prefs;

  RxItem _queue[RX_QUEUE_LEN];
  volatile int _qHead = 0, _qTail = 0;
  portMUX_TYPE _qMux = portMUX_INITIALIZER_UNLOCKED;

  static void _setChannel(uint8_t ch) {
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
  }

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  static void _onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    if (_instance) _instance->_push(info->src_addr, data, len);
  }
#else
  static void _onRecv(const uint8_t *mac, const uint8_t *data, int len) {
    if (_instance) _instance->_push(mac, data, len);
  }
#endif

  void _push(const uint8_t *mac, const uint8_t *data, int len) {
    if (len < 2 || len > (int)sizeof(MsgRc)) return;
    portENTER_CRITICAL(&_qMux);
    int next = (_qHead + 1) % RX_QUEUE_LEN;
    if (next != _qTail) {
      RxItem &it = _queue[_qHead];
      memcpy(it.mac, mac, 6);
      it.len = (uint8_t)len;
      memcpy(it.data, data, len);
      _qHead = next;
    }
    portEXIT_CRITICAL(&_qMux);
  }

  bool _pop(RxItem &out) {
    bool ok = false;
    portENTER_CRITICAL(&_qMux);
    if (_qTail != _qHead) {
      out = _queue[_qTail];
      _qTail = (_qTail + 1) % RX_QUEUE_LEN;
      ok = true;
    }
    portEXIT_CRITICAL(&_qMux);
    return ok;
  }

  void _lock(uint8_t ch) {
    _lockedChannel = ch;
    _setChannel(ch);
    if (ch != _savedChannel) {
      _prefs.putUChar("ch", ch);
      _savedChannel = ch;
    }
  }

  int _findSlot(const uint8_t *mac) const {
    for (int i = 0; i < _count; i++)
      if (!macIsZero(_slots[i].mac) && memcmp(_slots[i].mac, mac, 6) == 0) return i;
    return -1;
  }

  int _claimLearnSlot(const uint8_t *mac) {
    for (int i = 0; i < _count; i++) {
      if (_slots[i].learn && macIsZero(_slots[i].mac)) {
        memcpy(_slots[i].mac, mac, 6);
        if (_log) {
          _log->print("[rclink] slot ");
          _log->print(i);
          _log->print(" learned drone MAC ");
          printMac(*_log, mac);
          _log->println("  (paste it into DRONE_MACS to pin it)");
        }
        return i;
      }
    }
    return -1;
  }

  void _ensurePeer(const uint8_t *mac) {
    if (esp_now_is_peer_exist(mac)) return;
    esp_now_peer_info_t peer;
    memset(&peer, 0, sizeof(peer));
    memcpy(peer.peer_addr, mac, 6);
    peer.channel = 0;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_now_add_peer(&peer);
  }

  void _drainQueue(uint32_t now) {
    RxItem it;
    while (_pop(it)) {
      if (checksum(it.data, it.len - 1) != it.data[it.len - 1]) continue;
      const uint8_t type = it.data[0];
      int slot = _findSlot(it.mac);

      if (type == PAIR_REQ && it.len == sizeof(MsgPairReq)) {
        if (slot < 0) slot = _claimLearnSlot(it.mac);
        if (slot < 0) continue;
        const uint8_t ch = it.data[1];
        if (ch < WIFI_CH_MIN || ch > WIFI_CH_MAX) continue;
        if (_lockedChannel == 0) {
          _lock(ch);
        } else if (ch != _lockedChannel) {
          if (_log) {
            _log->print("[rclink] drone ");
            _log->print(slot);
            _log->print(" is on Wi-Fi channel ");
            _log->print(ch);
            _log->print(" but the link is locked to ");
            _log->println(_lockedChannel);
          }
          continue;
        }
        _ensurePeer(it.mac);
        MsgPairRes res;
        seal(res);
        esp_now_send(it.mac, reinterpret_cast<const uint8_t *>(&res), sizeof(res));
        if (!_slots[slot].paired && _log) {
          _log->print("[rclink] paired drone ");
          _log->print(slot);
          _log->print(" (");
          printMac(*_log, it.mac);
          _log->print(") on channel ");
          _log->println(ch);
        }
        _slots[slot].paired = true;
        _slots[slot].lastHeardMs = now;
        _lastAnyHeardMs = now;
      } else if (type == FC_ALIVE || type == FC_DATA) {
        if (slot >= 0 && _slots[slot].paired) {
          _slots[slot].lastHeardMs = now;
          _lastAnyHeardMs = now;
          continue;
        }

        if (slot < 0) slot = _claimLearnSlot(it.mac);
        if (slot < 0) continue;
        if (_lockedChannel == 0) _lock(_hopChannel);
        _ensurePeer(it.mac);
        _slots[slot].paired = true;
        _slots[slot].lastHeardMs = now;
        _lastAnyHeardMs = now;
        if (_log) {
          _log->print("[rclink] re-attached drone ");
          _log->print(slot);
          _log->print(" (");
          printMac(*_log, it.mac);
          _log->print(") on channel ");
          _log->println(_lockedChannel);
        }
      }
    }
  }
};

MultiTx *MultiTx::_instance = nullptr;

}
