
#include "Copter.h"
#include <AP_SerialManager/AP_SerialManager.h>
/*
 * Modifico el modo althold,
 * La idea es que este mantenga la altitud con el barometro hasta que la raspberry se conecte con este por puerto serie y tome el control.
 * Cualquier cambio en rc se vuelve a mantener la altitud.
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeCtrlSlave::init(bool ignore_checks)
{

    // initialise the vertical position controller
    if (!pos_control->is_active_z())
    {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    taken = false;
    tick_count = 0;
    // uart = AP::serialmanager().find_serial(AP_SerialManager::SerialProtocol_None, SERIAL_INSTANCE_M);
    if (uart == nullptr)
    {
        uart = hal.serial(SERIAL_INSTANCE_M);
        if (uart == nullptr)
        {

            gcs().send_text(MAV_SEVERITY_CRITICAL, "Puerto serie no  abierto ");
            return false;
        }
        uint32_t baud = 115200;
        uart->begin(baud);
    }

    // uint32_t baud =AP::serialmanager().find_baudrate(AP_SerialManager::SerialProtocol_None, SERIAL_INSTANCE_M);

    init_throttle = channel_throttle->get_control_in();
    pack_cmd.roll = 0;
    pack_cmd.pitch = 0;
    pack_cmd.yaw = 0;
    pack_cmd.throttle = motors->get_throttle();
    pack_cmd.cmd = CMD::TAKEPOS;
    ctrl_mode = CMD::TAKEPOS;
    taken = true;
    return true;
}

void ModeCtrlSlave::run()
{

    int16_t delta = channel_throttle->get_control_in() - init_throttle;

    if (!taken)
    {
        alt_hold();
        return;
    }

    if (delta < -20 || delta > 20)
    {
        taken = false;
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Modo esclavo off por control");
        alt_hold();
        return;
    }

    tick_count++;
    if (tick_count == 1)
    {
        send_state();
    }
    if (tick_count == tick_max)
    {
        if (receive_cmd())
        {

            ctrl_mode = pack_cmd.cmd;
        }
        else
        {
            taken = false;
            gcs().send_text(MAV_SEVERITY_CRITICAL, "Modo esclavo off por error puerto serie ");
            alt_hold();
        }

        tick_count = 0;
    }


    switch (ctrl_mode)
    {
    case CMD::TAKEPOS:
        // call attitude controller
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(pack_cmd.roll * 100, pack_cmd.pitch * 100, pack_cmd.yaw * 100);

        // output pilot's throttle
        attitude_control->set_throttle_out(constrain_float(pack_cmd.throttle, 0, 1), true, g.throttle_filt);

        break;
    case CMD::TAKEALL:
        motors->set_roll(constrain_float(pack_cmd.roll, -1, 1));
        motors->set_roll_ff(0);

        motors->set_pitch(constrain_float(pack_cmd.pitch, -1, 1));
        motors->set_pitch_ff(0);

        motors->set_yaw(constrain_float(pack_cmd.yaw, -1, 1));

        motors->set_yaw_ff(0);

        motors->set_throttle(constrain_float(pack_cmd.throttle, 0, 1));
        motors->output();
        break;

    case CMD::REALESE:
        taken = false;
        gcs().send_text(MAV_SEVERITY_INFO, "Modo esclavo desactivado por el maestro");
        alt_hold();

        return;

    default:
        break;
    }
}




// althold_run - runs the althold controller
// should be called at 100hz or more

void ModeCtrlSlave::alt_hold()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());

    // get pilot's desired yaw rate
    float target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

    // get pilot desired climb rate
    float target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
    target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state)
    {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f); // forces throttle output to decay to zero
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f); // forces throttle output to decay to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running())
        {
            takeoff.start(constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}

bool ModeCtrlSlave::send_state()
{
    Vector3f gyro_latest = ahrs.get_gyro_latest();
    Vector3f acc = ahrs.get_accel();

    pack_send.gyro[0] = gyro_latest[0];
    pack_send.gyro[1] = gyro_latest[1];
    pack_send.gyro[2] = gyro_latest[2];
    pack_send.acc[0] = acc[0];
    pack_send.acc[1] = acc[1];
    pack_send.acc[2] = acc[2];
    pack_send.roll = ahrs.get_roll();
    pack_send.pitch = ahrs.get_pitch();
    pack_send.yaw = ahrs.get_yaw();
    pack_send.alt = inertial_nav.get_position_z_up_cm();

    pack_send.motor_state = static_cast<uint8_t>(motors->get_spool_state());
    ::printf("%f, %d\n", pack_send.yaw, pack_send.motor_state);
    if (uart->tx_pending())
        return false;

    uart->write(&pack_send.head, sizeof(pack_send));
    return true;
}

bool ModeCtrlSlave::receive_cmd()
{
    size_t c = 0;
    uint32_t retry =10;
    while (c ==0 && retry--)
        c =uart->read(&pack_cmd.head, sizeof(pack_cmd));
    
    ::printf("uart  %ld, %x, %f, %f, %f, %f ",c,pack_cmd.head,pack_cmd.throttle,pack_cmd.roll,pack_cmd.pitch,pack_cmd.yaw);
    
    return c == sizeof(pack_cmd);
}

void ModeCtrlSlave::output_to_motors() 
{
    if (ctrl_mode != CMD::TAKEALL)
        motors->output();
}