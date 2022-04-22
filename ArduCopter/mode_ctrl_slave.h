#pragma once
#include "Copter.h"
#include "mode.h"

#define SERIAL_INSTANCE_M 5
class ModeCtrlSlave : public Mode {

public:
    enum CMD
    {
        
        TAKEPOS, 
        TAKEALL,
        REALESE,
        
    };
    struct __attribute__((__packed__)) {
        uint8_t head =0xAA;
        float gyro[3];
        float acc[3];
        float roll,pitch,yaw;
        float alt;
        uint8_t motor_state;
   

    }pack_send;
    struct __attribute__((__packed__)) {
        uint8_t head =0xAA;
        uint8_t cmd;
        float roll;
        float pitch;
        float yaw;
        float throttle;
    
    }pack_cmd;
    // inherit constructor
    using Mode::Mode;
    Number mode_number() const override { return Number::CTRL_SLAVE; }

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return false; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override {
        return !must_navigate;
    }
    bool allows_autotune() const override { return true; }
    bool allows_flip() const override { return true; }
    void output_to_motors() override;

protected:

    const char *name() const override { return "CTRL_SLAVE"; }
    const char *name4() const override { return "CTSL"; }

private:
    void alt_hold();
    bool taken;
    int tick_max = 4,tick_count=0;
    bool receive_cmd();
    bool send_state();
    uint8_t ctrl_mode=0; // 0 pos- control // 1 todo
    int16_t init_throttle;
    AP_HAL::UARTDriver *uart = nullptr;
};