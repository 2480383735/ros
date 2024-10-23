#ifndef __AMC_SERIAL_HPP__
#define __AMC_SERIAL_HPP__

#include <ros/ros.h>

template <uint16_t _Size>
class __attribute__((packed)) UartMessage
{
private:
    uint8_t header1 = 0x0A;
    uint8_t header2 = 0x0C;
    uint16_t dataLength;
    uint8_t cmd;
    std::array<uint8_t, _Size> data;
    uint8_t checkSum;
public:
    UartMessage() {
        this->dataLength = _Size > 0U ? _Size : 1U;
    }
    ~UartMessage() {}
    inline void setCmd(const uint8_t& cmd) noexcept {this->cmd = cmd;}
    template <class _Ty = uint8_t>
    void setData(const uint16_t& index, const uint32_t& value) {
        uint16_t size = sizeof(_Ty);
        for (uint16_t i = 0; i < size; i++) {
            this->data.at(index+i) = value >> (i * 8U) & 0xFF;
        }
    }
    inline void setCheckSum(uint8_t checkSum) noexcept { this->checkSum = checkSum; }
    inline uint8_t getCmd() const noexcept { return this->cmd; }
    inline uint8_t getCheckSum() const noexcept { return this->checkSum; }
    template <class _Ty = uint8_t>
    _Ty getData(const uint16_t& index) const {
        uint16_t size = sizeof(_Ty);
        _Ty result = 0;
        for (uint16_t i = 0; i < size; i++) {
            result |= this->data.at(index+i) << (i * 8U);
        }
        return result;
    }
    UartMessage& check() {
        uint8_t temp = 0U;
        temp ^= (this->dataLength & 0xFF) ^ (this->dataLength >> 8 & 0xFF) ^ (this->cmd);
        for(auto var : this->data) {
            temp ^= var;
        }
        this->checkSum = temp;
        return *this;
    }
    inline uint16_t size() const noexcept { return this->dataLength + 6U; }
    inline uint8_t* begin() noexcept {return reinterpret_cast<uint8_t*>(this); }
    inline uint16_t getDataLength() const noexcept { return this->dataLength; }
};

#endif // !__AMC_SERIAL_HPP__
