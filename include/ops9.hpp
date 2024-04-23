#if !defined(OPS9_HPP)
#define OPS9_HPP
#include "HardwareSerial.h"
class ops9
{
public:
    ops9(HardwareSerial *serial)
    {
        _serial = serial;
    };
    void setup()
    {
        _serial->begin(115200);
        xTaskCreate(ops9_task, "read_data", 4096, this, 2, NULL);
    }
    void read_data()
    {
        uint8_t data[28];
        // uint8_t front_data[2];

        while (1)
        {
            if (_serial->available())
            {
                _serial->read(data, 2);
            }
            if (data[0] == 0x0d && data[1] == 0x0a)
            {
                delay(3);
                break;
            }
            delay(1);
            if (_serial->available() > 0)
            {
                uint8_t Receive_data = _serial->read();

                // 清零
                if (Receive_data == 'ACT0')
                {
                    zangle = 0, xangle = 0, yangle = 0, x = 0, y = 0, zangle_speed = 0;
                }
            }
        }
        if (_serial->available())
        {
            _serial->read(data, 28);
        }
        if (data[26] == 0x0a && data[27] == 0x0d)
        {
            int temp = data[2] + (data[3] << 8) + (data[4] << 16) + (data[5] << 24);
            int *temp2 = &temp;
            zangle = *(float *)temp2;
            temp = data[6] + (data[7] << 8) + (data[8] << 16) + (data[9] << 24);
            xangle = *(float *)temp2;
            temp = data[10] + (data[11] << 8) + (data[12] << 16) + (data[13] << 24);
            yangle = *(float *)temp2;
            temp = data[14] + (data[15] << 8) + (data[16] << 16) + (data[17] << 24);
            x = *(float *)temp2;
            temp = data[18] + (data[19] << 8) + (data[20] << 16) + (data[21] << 24);
            y = *(float *)temp2;
            temp = data[22] + (data[23] << 8) + (data[24] << 16) + (data[25] << 24);
            zangle_speed = *(float *)temp2;
        }
    }

    ~ops9();

private:
    HardwareSerial *_serial;
    float zangle = 0;       // 航向角
    float xangle = 0;       // 俯仰角
    float yangle = 0;       // 横滚角
    float x = 0;            // 坐标X
    float y = 0;            // 坐标Y
    float zangle_speed = 0; // 航向角速度
};

void ops9_task(void *n)
{
    ops9 *obj = (ops9 *)n;
    while (1)
    {
        obj->read_data();
    }
};

#endif // MACRO