#include "mbus.h"

namespace esphome {
namespace wmbus {

  static const char *TAG = "mbus";

  bool mBusDecode(WMbusData &t_in, WMbusFrame &t_frame) {
    bool retVal{false};
    if (t_in.mode == 'C') {
      // correct length in C mode - remove 2 bytes preamble
      t_in.length -= 2;
      if (t_in.block == 'A') {
        ESP_LOGD(TAG, "Received C1 A frame");
        std::vector<unsigned char> frame(t_in.data, t_in.data + t_in.length);
        std::string telegram = format_hex_pretty(frame);
        telegram.erase(std::remove(telegram.begin(), telegram.end(), '.'), telegram.end());
        ESP_LOGV(TAG, "Frame: %s [with CRC]", telegram.c_str());
        if (mBusDecodeFormatA(t_in, t_frame)) {
          retVal = true;
        }
      }
      else if (t_in.block == 'B') {
        ESP_LOGD(TAG, "Received C1 B frame");
        std::vector<unsigned char> frame(t_in.data, t_in.data + t_in.length);
        std::string telegram = format_hex_pretty(frame);
        telegram.erase(std::remove(telegram.begin(), telegram.end(), '.'), telegram.end());
        ESP_LOGV(TAG, "Frame: %s [with CRC]", telegram.c_str());
        if (mBusDecodeFormatB(t_in, t_frame)) {
          retVal = true;
        }
      }
    }
    else if (t_in.mode == 'T') {
      ESP_LOGD(TAG, "Received T1 A frame");
      std::vector<unsigned char> rawFrame(t_in.data, t_in.data + t_in.length);
      std::string telegram = format_hex_pretty(rawFrame);
      telegram.erase(std::remove(telegram.begin(), telegram.end(), '.'), telegram.end());
      if (telegram.size() > 400) {  // ToDo: rewrite
        std::string tel_01 = telegram.substr(0,400);
        ESP_LOGV(TAG, "Frame: %s [RAW]", tel_01.c_str());
        std::string tel_02 = telegram.substr(400,800);
        ESP_LOGV(TAG, "       %s [RAW]", tel_02.c_str());
      }
      else {
        ESP_LOGV(TAG, "Frame: %s [RAW]", telegram.c_str());
      }

      if (decode3OutOf6(&t_in, packetSize(t_in.lengthField))) {
        std::vector<unsigned char> frame(t_in.data, t_in.data + t_in.length);
        std::string telegram = format_hex_pretty(frame);
        telegram.erase(std::remove(telegram.begin(), telegram.end(), '.'), telegram.end());
        ESP_LOGV(TAG, "Frame: %s [with CRC]", telegram.c_str());
        if (mBusDecodeFormatA(t_in, t_frame)) {
          retVal = true;
        }
      }

    }
    else if (t_in.mode == 'S') {
      ESP_LOGD(TAG, "Received S1 A frame");
      // Mode S uses Manchester encoding, decode it first
      if (mBusDecodeModeS(t_in, t_frame)) {
        retVal = true;
      }
    }
    if (retVal) {
      std::string telegram = format_hex_pretty(t_frame.frame);
      telegram.erase(std::remove(telegram.begin(), telegram.end(), '.'), telegram.end());
      ESP_LOGV(TAG, "Frame: %s [without CRC]", telegram.c_str());
    }
    return retVal;
  }


/*
  Format A

  L-field = length without CRC fields and without L (1 byte)

    Block 1
  ---------------------------------------------------
  | L-field | C-field | M-field | A-field |   CRC   |
  |  1 byte |  1 byte | 2 bytes | 6 bytes | 2 bytes |
  ---------------------------------------------------

    Block 2
  ---------------------------------------------------
  | CI-field |         Data-field         |   CRC   |
  |  1 byte  | 15 or (((L-9) mod 16) – 1) | 2 bytes |
  ---------------------------------------------------

    Block n (optional)
  ---------------------------------------------------
  |               Data-field              |   CRC   |
  |       16 or ((L-9) mod 16) bytes      | 2 bytes |
  ---------------------------------------------------
*/
  bool mBusDecodeFormatA(const WMbusData &t_in, WMbusFrame &t_frame) {
    uint8_t L = t_in.data[0];

    // Validate CRC
    ESP_LOGV(TAG, "Validating CRC for Block1");
    if (!crcValid(t_in.data, (BLOCK1A_SIZE - 2))) {
      return false;
    }

    // Check length of package is sufficient
    uint8_t numDataBlocks = (L - 9 + 15) / 16;                                           // Data blocks are 16 bytes long + 2 CRC bytes (not counted in L)
    if ((L < 9) || (((L - 9 + (numDataBlocks * 2))) > (t_in.length - BLOCK1A_SIZE))) {   // add CRC bytes for each data block
      ESP_LOGV(TAG, "Package (%u) too short for packet Length: %u", t_in.length, L);
      ESP_LOGV(TAG, "  %u > %u", (L - 9 + (numDataBlocks * 2)), (t_in.length - BLOCK1A_SIZE));
      return false;
    }

    t_frame.frame.insert(t_frame.frame.begin(), t_in.data, ( t_in.data + (BLOCK1A_SIZE - 2)));
    // Get all remaining data blocks and concatenate into data array (removing CRC bytes)
    for (uint8_t n{0}; n < numDataBlocks; ++n) {
      const uint8_t *blockStartPtr = (t_in.data + BLOCK1A_SIZE + (n * 18));  // Pointer to where data starts. Each block is 18 bytes
      uint8_t blockSize    = (MIN((L - 9 - (n * 16)), 16));                  // Maximum block size is 16 Data (without 2 CRC)

      // Validate CRC
      ESP_LOGV(TAG, "Validating CRC for Block%u", (n + 2));
      if (!crcValid(blockStartPtr, (blockSize))) {
        return false;
      }

      // Get block data
      t_frame.frame.insert((t_frame.frame.begin() + ((n * 16) + BLOCK1A_SIZE - 2)), blockStartPtr, (blockStartPtr + blockSize));
    }

    return true;
  }

/*
  Format B

  L-field = length with CRC fields and without L (1 byte)

    Block 1
  ---------------------------------------------------
  |   L-field  |   C-field  |  M-field  |  A-field  |
  |    1 byte  |    1 byte  |  2 bytes  |  6 bytes  |
  ---------------------------------------------------

    Block 2
  ---------------------------------------------------
  | CI-field |         Data-field         |   CRC   |
  |  1 byte  |      115 (L-12) bytes      | 2 bytes |
  ---------------------------------------------------

    Block 3 (optional)
  ---------------------------------------------------
  |               Data-field              |   CRC   |
  |             (L-129) bytes             | 2 bytes |
  ---------------------------------------------------
*/
  bool mBusDecodeFormatB(const WMbusData &t_in, WMbusFrame &t_frame) {
    uint8_t L = t_in.data[0];
    const uint8_t *blockStartPtr{nullptr};
    uint8_t blockSize{0};

    // Check length of package is sufficient
    if ((L < 12) || ((L + 1) > t_in.length)) {  // pod len mam miec zapisane ile bajtow odebralem
      ESP_LOGV(TAG, "Package (%u) too short for packet Length: %u", t_in.length, L);
      ESP_LOGV(TAG, "  %u > %u", (L + 1), t_in.length);
      return false;
    }

    blockSize = MIN((L - 1), (BLOCK1B_SIZE + BLOCK2B_SIZE - 2));
    blockStartPtr = t_in.data;
    // Validate CRC for Block1 + Block2
    ESP_LOGV(TAG, "Validating CRC for Block1 + Block2");
    if (!crcValid(t_in.data, blockSize)) {
      return false;
    }

    // Get data from Block1 + Block2
    t_frame.frame.insert(t_frame.frame.begin(), blockStartPtr, (blockStartPtr + blockSize));
    t_frame.frame[0] -= 2;

    // Check if Block3 is present (long telegrams)
    const uint8_t L_OFFSET = (BLOCK1B_SIZE + BLOCK2B_SIZE);
    if (L > (L_OFFSET + 2)) {
      blockSize = (L - L_OFFSET - 1);
      blockStartPtr = (t_in.data + L_OFFSET);
      // Validate CRC for Block3
      ESP_LOGV(TAG, "Validating CRC for Block3");
      if (!crcValid(blockStartPtr, blockSize)) {
        return false;
      }
      // Get Block3
      t_frame.frame.insert((t_frame.frame.end()), blockStartPtr, (blockStartPtr + blockSize));
      t_frame.frame[0] -= 2;
    }
    return true;
  }

  bool mBusDecodeModeS(const WMbusData &t_in, WMbusFrame &t_frame) {
    // Mode S uses Manchester encoding - decode from Manchester to binary
    std::vector<unsigned char> rawFrame(t_in.data, t_in.data + t_in.length);
    std::string telegram = format_hex_pretty(rawFrame);
    telegram.erase(std::remove(telegram.begin(), telegram.end(), '.'), telegram.end());
    ESP_LOGV(TAG, "Frame: %s [RAW Manchester]", telegram.c_str());
    
    // Manchester decode: each bit is represented by 2 bits (01 = 0, 10 = 1)
    std::vector<unsigned char> decoded_data;
    for (size_t i = 0; i < t_in.length; i++) {
      unsigned char byte = t_in.data[i];
      unsigned char decoded_byte = 0;
      
      // Decode 4 Manchester bits to 2 regular bits
      for (int bit = 0; bit < 4; bit++) {
        unsigned char manchester_pair = (byte >> (6 - bit * 2)) & 0x03;
        if (manchester_pair == 0x01) {  // 01 = 0
          // decoded_byte already has 0
        } else if (manchester_pair == 0x02) {  // 10 = 1
          decoded_byte |= (1 << (3 - bit));
        } else {
          ESP_LOGW(TAG, "Invalid Manchester encoding at byte %zu, bit %d: %02X", i, bit, manchester_pair);
          return false;
        }
      }
      decoded_data.push_back(decoded_byte);
    }
    
    // Create a temporary WMbusData structure with decoded data
    WMbusData decoded_t_in = t_in;
    decoded_t_in.length = decoded_data.size();
    std::copy(decoded_data.begin(), decoded_data.end(), decoded_t_in.data);
    
    std::string decoded_telegram = format_hex_pretty(decoded_data);
    decoded_telegram.erase(std::remove(decoded_telegram.begin(), decoded_telegram.end(), '.'), decoded_telegram.end());
    ESP_LOGV(TAG, "Frame: %s [decoded from Manchester]", decoded_telegram.c_str());
    
    // Mode S typically uses Format A structure after Manchester decoding
    return mBusDecodeFormatA(decoded_t_in, t_frame);
  }

}
}