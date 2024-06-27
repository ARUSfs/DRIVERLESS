#include <stdio.h>

class eposHandle
{
public:
    eposHandle(int max_acc, int max_dec, int prof_vel);
    void connect_to_device();
    void disconnect_device();
    void enable();
    void disable();
    void move_to(float wheel_angle);

private:
    int max_acc;
    int max_dec;
    int prof_vel;
    bool is_connected;
    bool is_enabled;
    bool is_centered;
    void* g_pKeyHandle;

};