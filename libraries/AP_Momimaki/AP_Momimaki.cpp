#include "AP_Momimaki.h"

#include <stdlib.h>

// include譛ｪ謨ｴ逅�
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>

// ------------------------------
#define CAM_DEBUG DISABLED

const AP_Param::GroupInfo AP_Momimaki::var_info[] = {
    
    // @Param: DENSITY
    // @DisplayName: Density of sowing
    // @Description: Density of sowing
    // @User: Standard
    // @Units: pcs/m^2
    // @Range: 0 10
    AP_GROUPINFO("DENSITY",  0, AP_Momimaki, _default_density, AP_MOMIMAKI_DEFAULT_DENSITY ),
    
    // @Param: RADIUS
    // @DisplayName: radius of sowing area
    // @Description: radius of sowing area
    // @User: Standard
    // @Units: m
    // @Range: 2 10
    AP_GROUPINFO("RADIUS",  1, AP_Momimaki, _default_radius, AP_MOMIMAKI_DEFAULT_RADIUS ),

    // @Param: ANGLE
    // @DisplayName: angle of sowing area
    // @Description: angle of sowing area
    // @User: Standard
    // @Units: degree
    // @Range: 30 180
    AP_GROUPINFO("ANGLE",  2, AP_Momimaki, _angle, AP_MOMIMAKI_DEFAULT_ANGLE ),

#if false
    // @Param: R_TO_PRM
    // @DisplayName: radius to pwm convert rate
    // @Description: radius to pwm convert rate
    // @User: Standard
    // @Units: pcs/m^2
    // @Range: 0 10
    AP_GROUPINFO("R2PRM",  3, AP_Momimaki, _r_to_pwm, AP_MOMIMAKI_DEFAULT_R_TO_PRM ),

    // @Param: SOW_DENSITY
    // @DisplayName: Density of sowing
    // @Description: Density of sowing
    // @User: Standard
    // @Units: pcs/m^2
    // @Range: 0 10
    AP_GROUPINFO("F2PRM",  4, AP_Momimaki, _feed_to_prm, AP_MOMIMAKI_DEFAULT_F_TO_PRM ),
#endif






    // @Param: FRMAX
    // @DisplayName: max rpm of feeder
    // @Description: max rpm of feeder gear at full throttle
    // @User: Standard
    // @Units: rpm
    // @Range: 1 1000
    AP_GROUPINFO("FRMAX",  3, AP_Momimaki, _feeder_max_rpm, AP_MOMIMAKI_DEFAULT_FDR_MAX ),

    // @Param: FNPR
    // @DisplayName: feed num per gear rotation
    // @Description: feed num per gear rotation
    // @User: Standard
    // @Units: pcs / rotate
    // @Range: 1 100
    AP_GROUPINFO("FNPR",  4, AP_Momimaki, _feed_num_par_rotate, AP_MOMIMAKI_DEFAULT_FED_NPR ),


    // @Param: SRMAX
    // @DisplayName: max radius of spreading
    // @Description: max radius of spreading at full throttle
    // @User: Standard
    // @Units: m
    // @Range: 0.5 20
    AP_GROUPINFO("SRMAX",  5, AP_Momimaki, _spreader_max_radius, AP_MOMIMAKI_DEFAULT_SPR_RAD_MAX ),


    
#if false   
    // 莉･荳九�、P_Camera縺ｮ繝代Λ繝｡繝ｼ繧ｿ
    
    // @Param: TRIGG_TYPE
    // @DisplayName: Camera shutter (trigger) type
    // @Description: how to trigger the camera to take a picture
    // @Values: 0:Servo,1:Relay
    // @User: Standard
    AP_GROUPINFO("TRIGG_TYPE",  0, AP_Camera, _trigger_type, AP_CAMERA_TRIGGER_DEFAULT_TRIGGER_TYPE),

    // @Param: DURATION
    // @DisplayName: Duration that shutter is held open
    // @Description: How long the shutter will be held open in 10ths of a second (i.e. enter 10 for 1second, 50 for 5seconds)
    // @Units: ds
    // @Range: 0 50
    // @User: Standard
    AP_GROUPINFO("DURATION",    1, AP_Camera, _trigger_duration, AP_CAMERA_TRIGGER_DEFAULT_DURATION),

    // @Param: SERVO_ON
    // @DisplayName: Servo ON PWM value
    // @Description: PWM value in microseconds to move servo to when shutter is activated
    // @Units: PWM
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SERVO_ON",    2, AP_Camera, _servo_on_pwm, AP_CAMERA_SERVO_ON_PWM),

    // @Param: SERVO_OFF
    // @DisplayName: Servo OFF PWM value
    // @Description: PWM value in microseconds to move servo to when shutter is deactivated
    // @Units: PWM
    // @Range: 1000 2000
    // @User: Standard
    AP_GROUPINFO("SERVO_OFF",   3, AP_Camera, _servo_off_pwm, AP_CAMERA_SERVO_OFF_PWM),

    // @Param: TRIGG_DIST
    // @DisplayName: Camera trigger distance
    // @Description: Distance in meters between camera triggers. If this value is non-zero then the camera will trigger whenever the GPS position changes by this number of meters regardless of what mode the APM is in. Note that this parameter can also be set in an auto mission using the DO_SET_CAM_TRIGG_DIST command, allowing you to enable/disable the triggering of the camera during the flight.
    // @User: Standard
    // @Units: m
    // @Range: 0 1000
    AP_GROUPINFO("TRIGG_DIST",  4, AP_Camera, _trigg_dist, 0),

    // @Param: RELAY_ON
    // @DisplayName: Relay ON value
    // @Description: This sets whether the relay goes high or low when it triggers. Note that you should also set RELAY_DEFAULT appropriately for your camera
    // @Values: 0:Low,1:High
    // @User: Standard
    AP_GROUPINFO("RELAY_ON",    5, AP_Camera, _relay_on, 1),

    // @Param: MIN_INTERVAL
    // @DisplayName: Minimum time between photos
    // @Description: Postpone shooting if previous picture was taken less than preset time(ms) ago.
    // @User: Standard
    // @Units: ms
    // @Range: 0 10000
    AP_GROUPINFO("MIN_INTERVAL",  6, AP_Camera, _min_interval, 0),

    // @Param: MAX_ROLL
    // @DisplayName: Maximum photo roll angle.
    // @Description: Postpone shooting if roll is greater than limit. (0=Disable, will shoot regardless of roll).
    // @User: Standard
    // @Units: deg
    // @Range: 0 180
    AP_GROUPINFO("MAX_ROLL",  7, AP_Camera, _max_roll, 0),

    // @Param: FEEDBACK_PIN
    // @DisplayName: Camera feedback pin
    // @Description: pin number to use for save accurate camera feedback messages. If set to -1 then don't use a pin flag for this, otherwise this is a pin number which if held high after a picture trigger order, will save camera messages when camera really takes a picture. A universal camera hot shoe is needed. The pin should be held high for at least 2 milliseconds for reliable trigger detection. See also the CAM_FEEDBACK_POL option.
    // @Values: -1:Disabled,50:AUX1,51:AUX2,52:AUX3,53:AUX4,54:AUX5,55:AUX6
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("FEEDBACK_PIN",  8, AP_Camera, _feedback_pin, AP_CAMERA_FEEDBACK_DEFAULT_FEEDBACK_PIN),

    // @Param: FEEDBACK_POL
    // @DisplayName: Camera feedback pin polarity
    // @Description: Polarity for feedback pin. If this is 1 then the feedback pin should go high on trigger. If set to 0 then it should go low
    // @Values: 0:TriggerLow,1:TriggerHigh
    // @User: Standard
    AP_GROUPINFO("FEEDBACK_POL",  9, AP_Camera, _feedback_polarity, 1),

    // @Param: AUTO_ONLY
    // @DisplayName: Distance-trigging in AUTO mode only
    // @Description: When enabled, trigging by distance is done in AUTO mode only.
    // @Values: 0:Always,1:Only when in AUTO
    // @User: Standard
    AP_GROUPINFO("AUTO_ONLY",  10, AP_Camera, _auto_mode_only, 0),

    // @Param: TYPE
    // @DisplayName: Type of camera (0: None, 1: BMMCC)
    // @Description: Set the camera type that is being used, certain cameras have custom functions that need further configuration, this enables that.
    // @Values: 0:Default,1:BMMCC
    // @User: Standard
    AP_GROUPINFO("TYPE",  11, AP_Camera, _type, 0),
#endif
    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;

#if false
/// Servo operated camera
void
AP_Momimaki::servo_pic()
{
    SRV_Channels::set_output_pwm(SRV_Channel::k_cam_trigger, _servo_on_pwm);

    // leave a message that it should be active for this many loops (assumes 50hz loops)
    _trigger_counter = constrain_int16(_trigger_duration*5,0,255);
}

/// basic relay activation
void
AP_Momimaki::relay_pic()
{
    AP_Relay *_apm_relay = AP::relay();
    if (_apm_relay == nullptr) {
        return;
    }
    if (_relay_on) {
        _apm_relay->on(0);
    } else {
        _apm_relay->off(0);
    }

    // leave a message that it should be active for this many loops (assumes 50hz loops)
    _trigger_counter = constrain_int16(_trigger_duration*5,0,255);
}

/// single entry point to take pictures
///  set send_mavlink_msg to true to send DO_DIGICAM_CONTROL message to all components
void AP_Momimaki::trigger_pic()
{
#if false
    setup_feedback_callback();

    _image_index++;
    switch (_trigger_type) {
    case AP_CAMERA_TRIGGER_TYPE_SERVO:
        servo_pic();                    // Servo operated camera
        break;
    case AP_CAMERA_TRIGGER_TYPE_RELAY:
        relay_pic();                    // basic relay activation
        break;
    }

    log_picture();
#endif
}

/// de-activate the trigger after some delay, but without using a delay() function
/// should be called at 50hz
void
AP_Momimaki::trigger_pic_cleanup()
{
#if false
    if (_trigger_counter) {
        _trigger_counter--;
    } else {
        switch (_trigger_type) {
        case AP_CAMERA_TRIGGER_TYPE_SERVO:
            SRV_Channels::set_output_pwm(SRV_Channel::k_cam_trigger, _servo_off_pwm);
            break;
        case AP_CAMERA_TRIGGER_TYPE_RELAY: {
            AP_Relay *_apm_relay = AP::relay();
            if (_apm_relay == nullptr) {
                break;
            }
            if (_relay_on) {
                _apm_relay->off(0);
            } else {
                _apm_relay->on(0);
            }
            break;
        }
        }
    }

    if (_trigger_counter_cam_function) {
        _trigger_counter_cam_function--;
    } else {
        switch (_type) {
        case AP_Camera::CAMERA_TYPE_BMMCC:
            SRV_Channels::set_output_pwm(SRV_Channel::k_cam_iso, _servo_off_pwm);
            break;
        }
    }
#endif
}
#endif


#if false
/// decode deprecated MavLink message that controls camera.
void
AP_Momimaki::control_msg(const mavlink_message_t &msg)
{
    __mavlink_digicam_control_t packet;
    mavlink_msg_digicam_control_decode(&msg, &packet);

    control(packet.session, packet.zoom_pos, packet.zoom_step, packet.focus_lock, packet.shot, packet.command_id);
}

void AP_Momimaki::configure(float shooting_mode, float shutter_speed, float aperture, float ISO, float exposure_type, float cmd_id, float engine_cutoff_time)
{
    // we cannot process the configure command so convert to mavlink message
    // and send to all components in case they and process it

    mavlink_message_t msg;
    mavlink_command_long_t mav_cmd_long = {};

    // convert mission command to mavlink command_long
    mav_cmd_long.command = MAV_CMD_DO_DIGICAM_CONFIGURE;
    mav_cmd_long.param1 = shooting_mode;
    mav_cmd_long.param2 = shutter_speed;
    mav_cmd_long.param3 = aperture;
    mav_cmd_long.param4 = ISO;
    mav_cmd_long.param5 = exposure_type;
    mav_cmd_long.param6 = cmd_id;
    mav_cmd_long.param7 = engine_cutoff_time;

    // Encode Command long into MAVLINK msg
    mavlink_msg_command_long_encode(0, 0, &msg, &mav_cmd_long);

    // send to all components
    GCS_MAVLINK::send_to_components(msg);

#if false
    if (_type == AP_Camera::CAMERA_TYPE_BMMCC) {
        // Set a trigger for the additional functions that are flip controlled (so far just ISO and Record Start / Stop use this method, will add others if required)
        _trigger_counter_cam_function = constrain_int16(_trigger_duration*5,0,255);

        // If the message contains non zero values then use them for the below functions
        if (ISO > 0) {
            SRV_Channels::set_output_pwm(SRV_Channel::k_cam_iso, _servo_on_pwm);
        }

        if (aperture > 0) {
            SRV_Channels::set_output_pwm(SRV_Channel::k_cam_aperture, (int)aperture);
        }

        if (shutter_speed > 0) {
            SRV_Channels::set_output_pwm(SRV_Channel::k_cam_shutter_speed, (int)shutter_speed);
        }

        // Use the shooting mode PWM value for the BMMCC as the focus control - no need to modify or create a new MAVlink message type.
        if (shooting_mode > 0) {
            SRV_Channels::set_output_pwm(SRV_Channel::k_cam_focus, (int)shooting_mode);
        }
    }
#endif
}
#endif


// AP_Mission::momimaki_controlから呼び出し
//void AP_Momimaki::control(bool enable_spreader, bool enable_feeder, float spread_radius, float spread_density )
void AP_Momimaki::control(int8_t enable_spreader, int8_t enable_feeder, float spread_radius, float spread_density )
{
    gcs().send_text(MAV_SEVERITY_NOTICE, "AP_Momimaki::control() was called");


    gcs().send_text(MAV_SEVERITY_INFO, "AP_Momimaki::control( %d, %d, %f, %f )",
            static_cast<int>( enable_spreader ),
            static_cast<int>( enable_feeder ),
            static_cast<double>( spread_radius ),
            static_cast<double>( spread_density ) );

    _enable_spreader = (bool)enable_spreader;
    _enable_feeder = (bool)enable_feeder;

    // 送り動かす場合は拡散も動かすこと。
    if( _enable_feeder )
        _enable_spreader = true;


    // 半径更新
    if( spread_radius >= 0.0 )
        _radius = spread_radius;
    else
        _radius = _default_radius;      // パラメータ値

    // 密度更新
    if( spread_density >= 0.0 )
        _density = spread_density;
    else
        _density = _default_density;    // パラメータ値


    // サーボ出力に反映する
    update();



#if false
    // take picture
    if (is_equal(shooting_cmd,1.0f)) {
        trigger_pic();
    }

    mavlink_message_t msg;
    mavlink_command_long_t mav_cmd_long = {};

    // convert command to mavlink command long
    mav_cmd_long.command = MAV_CMD_DO_DIGICAM_CONTROL;
    mav_cmd_long.param1 = session;
    mav_cmd_long.param2 = zoom_pos;
    mav_cmd_long.param3 = zoom_step;
    mav_cmd_long.param4 = focus_lock;
    mav_cmd_long.param5 = shooting_cmd;
    mav_cmd_long.param6 = cmd_id;

    // Encode Command long into MAVLINK msg
    mavlink_msg_command_long_encode(0, 0, &msg, &mav_cmd_long);

    // send to all components
    GCS_MAVLINK::send_to_components(msg);
#endif
}


#if false

/*
  Send camera feedback to the GCS
 */
void AP_Momimaki::send_feedback(mavlink_channel_t chan)
{
    const AP_AHRS &ahrs = AP::ahrs();

    float altitude, altitude_rel;
    if (current_loc.relative_alt) {
        altitude = current_loc.alt+ahrs.get_home().alt;
        altitude_rel = current_loc.alt;
    } else {
        altitude = current_loc.alt;
        altitude_rel = current_loc.alt - ahrs.get_home().alt;
    }

    mavlink_msg_camera_feedback_send(
        chan,
        AP::gps().time_epoch_usec(),
        0, 0, _image_index,
        current_loc.lat, current_loc.lng,
        altitude*1e-2f, altitude_rel*1e-2f,
        ahrs.roll_sensor*1e-2f, ahrs.pitch_sensor*1e-2f, ahrs.yaw_sensor*1e-2f,
        0.0f, CAMERA_FEEDBACK_PHOTO, _camera_trigger_logged);
}
#endif


/*
 * 籾送り、籾播きを動作させるかチェックする
 */
void AP_Momimaki::status_check( bool& feeder_sts, bool& spreader_sts)
{
    feeder_sts = false;
    spreader_sts = false;

    if( !_is_in_auto_mode )
    {
        return;
    }


    if (AP::gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        return;
    }

    if (is_zero(_density)) {
        return;
    }
    if (is_zero(_radius)) {
        return;
    }
    if(_angle == 0) {
        return;
    }
    if (is_zero(_feeder_max_rpm)) {
        return;
    }
    if (is_zero(_feed_num_par_rotate)) {
        return;
    }
    if (is_zero(_spreader_max_radius)) {
        return;
    }

    // waypointsコマンドによるON/OFFを反映
    spreader_sts = _enable_spreader;
    feeder_sts   =_enable_feeder;

}

/*  update; triggers by distance moved
*/
void AP_Momimaki::update()
{
    bool feeder_sts;
    bool spreader_sts;
    
    // 籾播ききを動作させるか決める
    status_check( feeder_sts, spreader_sts );
    
    // 機体速度
    float forward_speed;
    if (!atti_ctrl.get_forward_speed(forward_speed) ) {
        feeder_sts      = false;
        spreader_sts    = false;
    }
    
    if( feeder_sts )
    {
        // 機体速度に応じても見送り量を決める
        // 1秒あたりの籾送り量 momi_num : [pcs / sec ]
        float momi_num = _density * _radius * forward_speed * radians(_angle) / 2.0;


        // 1秒あたりの籾送り量から籾送りモーターの出力（Rate）を決める。
        float feed_rate = Calc_Momiokuri_FeedRate( momi_num );
        // 籾送り量　PWM出力
        pwm_output( SRV_Channel::k_momimaki_feeder, feed_rate );
    } else {
        pwm_output( SRV_Channel::k_momimaki_feeder, 0.0 );
    }
    

    if( spreader_sts )
    {
        // 籾拡散出力（Rate）
        float spread_rate = _radius / _spreader_max_radius;
        // 籾拡散量　PWM出力
        pwm_output( SRV_Channel::k_momimaki_spreader, spread_rate );
    }
    else
    {
        pwm_output( SRV_Channel::k_momimaki_spreader, 0 );
    }
    
    
}


/*
// 籾送り量と送り回転数（RPM）の換算
// args   : 目的とする籾送り量[ num / sec ]
// return : 必要な送り(最大送り量に対する％）
 */
float AP_Momimaki::Calc_Momiokuri_FeedRate( float tgt_num_per_sec )
{
    // 値チェックしていないので注意
    double feed_percentr = tgt_num_per_sec * 60.0 / _feeder_max_rpm / _feed_num_par_rotate;
    if( feed_percentr > 100.0 ) feed_percentr = 100.0;
    return feed_percentr;
}


/*
// 籾送り量と送り回転数（RPM）の換算
// args   : 籾播き半径[ m ]
// return : 必要な拡散量(最大拡散量に対する％）
 */
float AP_Momimaki::Calc_Momiokuri_SpreadRate(void)
{
    // 値チェックしていないので注意
    double feed_percentr = _radius * 60.0 / _feeder_max_rpm / _feed_num_par_rotate;
    if( feed_percentr > 100.0 ) feed_percentr = 100.0;
    return feed_percentr;
}



/*
  set servo_out and angle_min/max, then calc_pwm and output a
  value. This is used to move a AP_Mount servo
 */
void AP_Momimaki::pwm_output( SRV_Channel::Aux_servo_function_t function, float value )
{
    SRV_Channels::move_servo_totech(function, value );


    // for debug
    // 以下デバッグ用に表示
    static float bkup_feeder_val = 0;
    static float bkup_spreader_val = 0;

    if( function == SRV_Channel::k_momimaki_feeder)
    {
        if( fabs( bkup_feeder_val - value ) > 0.01 )
        {
            gcs().send_text(MAV_SEVERITY_NOTICE, "SRV_Channel::k_momimaki_feeder pwm changed to %f", value );

            bkup_feeder_val = value;
        }
    }
    if( function == SRV_Channel::k_momimaki_spreader)
    {
        if( fabs(bkup_spreader_val - value) > 0.01 )
        {
            gcs().send_text(MAV_SEVERITY_NOTICE, "SRV_Channel::k_momimaki_spreader pwm changed to %f", value );

            bkup_spreader_val = value;
        }
    }


#if false
    if (!SRV_Channels::function_assigned(function)) {
        return;
    }

    float v = constrain_float( value, 0.0f, 1.0f);

    SRV_Channel* c = SRV_Channels::get_channel_for( function );
    if( c != nullptr ) {
        float v2 = c->get_reversed() ? (1-v) : v;
        uint16_t pwm = c->servo_min + v2 * (c->servo_max - c->servo_min);
        c->set_output_pwm(pwm);
    }
#endif
}




#if false

/*
  interrupt handler for interrupt based feedback trigger
 */
void AP_Camera::feedback_pin_isr(uint8_t pin, bool high, uint32_t timestamp_us)
{
    _feedback_timestamp_us = timestamp_us;
    _camera_trigger_count++;
}

/*
  check if feedback pin is high for timer based feedback trigger, when
  attach_interrupt fails
 */
void AP_Camera::feedback_pin_timer(void)
{
    uint8_t pin_state = hal.gpio->read(_feedback_pin);
    uint8_t trigger_polarity = _feedback_polarity==0?0:1;
    if (pin_state == trigger_polarity &&
        _last_pin_state != trigger_polarity) {
        _feedback_timestamp_us = AP_HAL::micros();
        _camera_trigger_count++;
    }
    _last_pin_state = pin_state;
}

/*
  setup a callback for a feedback pin. When on PX4 with the right FMU
  mode we can use the microsecond timer.
 */
void AP_Camera::setup_feedback_callback(void)
{
    if (_feedback_pin <= 0 || _timer_installed || _isr_installed) {
        // invalid or already installed
        return;
    }

    // ensure we are in input mode
    hal.gpio->pinMode(_feedback_pin, HAL_GPIO_INPUT);

    // enable pullup/pulldown
    uint8_t trigger_polarity = _feedback_polarity==0?0:1;
    hal.gpio->write(_feedback_pin, !trigger_polarity);

    if (hal.gpio->attach_interrupt(_feedback_pin, FUNCTOR_BIND_MEMBER(&AP_Camera::feedback_pin_isr, void, uint8_t, bool, uint32_t),
                                   trigger_polarity?AP_HAL::GPIO::INTERRUPT_RISING:AP_HAL::GPIO::INTERRUPT_FALLING)) {
        _isr_installed = true;
    } else {
        // install a 1kHz timer to check feedback pin
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Camera::feedback_pin_timer, void));

        _timer_installed = true;
    }
}

// log_picture - log picture taken and send feedback to GCS
void AP_Camera::log_picture()
{
    AP_Logger *logger = AP_Logger::get_singleton();
    if (logger == nullptr) {
        return;
    }
    if (!using_feedback_pin()) {
        gcs().send_message(MSG_CAMERA_FEEDBACK);
        if (logger->should_log(log_camera_bit)) {
            logger->Write_Camera(current_loc);
        }
    } else {
        if (logger->should_log(log_camera_bit)) {
            logger->Write_Trigger(current_loc);
        }
    }
}

// take_picture - take a picture
void AP_Camera::take_picture()
{
    // take a local picture:
    trigger_pic();

    // tell all of our components to take a picture:
    mavlink_command_long_t cmd_msg {};
    cmd_msg.command = MAV_CMD_DO_DIGICAM_CONTROL;
    cmd_msg.param5 = 1;

    // forward to all components
    GCS_MAVLINK::send_to_components(MAVLINK_MSG_ID_COMMAND_LONG, (char*)&cmd_msg, sizeof(cmd_msg));
}

/*
  update camera trigger - 50Hz
 */
void AP_Camera::update_trigger()
{
    trigger_pic_cleanup();
    
    if (_camera_trigger_logged != _camera_trigger_count) {
        uint32_t timestamp32 = _feedback_timestamp_us;
        _camera_trigger_logged = _camera_trigger_count;

        gcs().send_message(MSG_CAMERA_FEEDBACK);
        AP_Logger *logger = AP_Logger::get_singleton();
        if (logger != nullptr) {
            if (logger->should_log(log_camera_bit)) {
                uint32_t tdiff = AP_HAL::micros() - timestamp32;
                uint64_t timestamp = AP_HAL::micros64();
                logger->Write_Camera(current_loc, timestamp - tdiff);
            }
        }
    }
}
#endif

// singleton instance
AP_Momimaki *AP_Momimaki::_singleton;

namespace AP {

AP_Momimaki *momimaki()
{
    return AP_Momimaki::get_singleton();
}

}
