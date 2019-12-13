/// @file	AP_Momimaki.h
/// @brief	Photo or video camera manager, with EEPROM-backed storage of constants.
#pragma once

#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS.h>
#include <APM_Control/AR_AttitudeControl.h>

// AP_MOMIMAKI縺ｧ霑ｽ蜉�
#define AP_MOMIMAKI_DEFAULT_DENSITY         1.0
#define AP_MOMIMAKI_DEFAULT_RADIUS          2.0
#define AP_MOMIMAKI_DEFAULT_ANGLE           60
//#define AP_MOMIMAKI_DEFAULT_R_TO_PRM        1000
//#define AP_MOMIMAKI_DEFAULT_F_TO_PRM        1000
#define AP_MOMIMAKI_DEFAULT_FDR_MAX         100
#define AP_MOMIMAKI_DEFAULT_FED_NPR         20
#define AP_MOMIMAKI_DEFAULT_SPR_RAD_MAX     10.0



#if false
    #define AP_CAMERA_TRIGGER_TYPE_SERVO                0
    #define AP_CAMERA_TRIGGER_TYPE_RELAY                1

    #define AP_CAMERA_TRIGGER_DEFAULT_TRIGGER_TYPE  AP_CAMERA_TRIGGER_TYPE_SERVO    // default is to use servo to trigger camera

    #define AP_CAMERA_TRIGGER_DEFAULT_DURATION  10      // default duration servo or relay is held open in 10ths of a second (i.e. 10 = 1 second)

    #define AP_CAMERA_SERVO_ON_PWM              1300    // default PWM value to move servo to when shutter is activated
    #define AP_CAMERA_SERVO_OFF_PWM             1100    // default PWM value to move servo to when shutter is deactivated

    #define AP_CAMERA_FEEDBACK_DEFAULT_FEEDBACK_PIN -1  // default is to not use camera feedback pin
#endif

/// @class	Momimaki
/// @brief	Object managing momimaki unit
class AP_Momimaki {

public:
    AP_Momimaki(const AR_AttitudeControl &_atti_ctrl)
        :atti_ctrl(_atti_ctrl)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _singleton = this;

    }

    /* Do not allow copies */
    AP_Momimaki(const AP_Momimaki &other) = delete;
    AP_Momimaki &operator=(const AP_Momimaki&) = delete;

    // get singleton instance
    static AP_Momimaki *get_singleton()
    {
        return _singleton;
    }

    // handle camera control
//    void control(bool enable_spreader, bool enable_feeder, float spread_radius, float spread_density );
    void control(int8_t enable_spreader, int8_t enable_feeder, float spread_radius, float spread_density );


#if false
    // MAVLink methods
    void            control_msg(const mavlink_message_t &msg);
    void            send_feedback(mavlink_channel_t chan);

    // Command processing
    void            configure(float shooting_mode, float shutter_speed, float aperture, float ISO, float exposure_type, float cmd_id, float engine_cutoff_time);
    // handle camera control
    void            control(float session, float zoom_pos, float zoom_step, float focus_lock, float shooting_cmd, float cmd_id);

    // set camera trigger distance in a mission
    void            set_trigger_distance(uint32_t distance_m)
    {
        _trigg_dist.set(distance_m);
    }

    void take_picture();
#endif


    // Update - to be called periodically @at least 10Hz
    void update();

#if false
    // update camera trigger - 50Hz
    void update_trigger();
#endif

    static const struct AP_Param::GroupInfo        var_info[];

    // set if vehicle is in AUTO mode
    void set_is_auto_mode(bool enable)
    {
        _is_in_auto_mode = enable;
    }
#if false

    void set_mode( Mode::Number arg_new_mode )
    {
        _mode_number = arg_new_mode;
    }
#endif

#if false

    enum camera_types {
        CAMERA_TYPE_STD,
        CAMERA_TYPE_BMMCC
    };
#endif

private:

    static AP_Momimaki *_singleton;

    // AP_Momimaki霑ｽ蜉�繝代Λ繝｡繝ｼ繧ｿ
    AP_Float        _density;           // density of sowing (pcs / m^2 )
    AP_Float        _radius;            // radius of sowing area
    AP_Int8         _angle;             // angle of sowing area
    //AP_Float        _r_to_pwm;          // radius to pwm convert rate
    //AP_Float        _feed_to_pwm;       // feed_to_pwm convaert rate

    AP_Float        _feeder_max_rpm;      // max rpm of feeder gear at full throttle
    AP_Float        _feed_num_par_rotate; // feed num per gear rotation
    AP_Float        _spreader_max_radius;  // max radius of spreading at full throttle

//    AP_Int8         _feeder_pwm_ch;     // 籾送り出力CH
//    AP_Int8     _spreader_pwm_ch;   // 籾拡散出力CH



    void status_check( bool& feeder_sts, bool& spreader_sts);

    float Calc_Momiokuri_FeedRate( float tgt_num_per_sec );

    float Calc_Momiokuri_SpreadRate(void);

    void pwm_output( SRV_Channel::Aux_servo_function_t function, float value );
    
    
//    Mode::Number    _mode_number;
    bool            _is_in_auto_mode;   // true if in AUTO mode
    const AR_AttitudeControl &atti_ctrl;

#if false
    // 莉･荳九�、P_Camera縺ｮ繝代Λ繝｡繝ｼ繧ｿ
    AP_Int8         _trigger_type;      // 0:Servo,1:Relay
    AP_Int8         _trigger_duration;  // duration in 10ths of a second that the camera shutter is held open
    AP_Int8         _relay_on;          // relay value to trigger camera
    AP_Int16        _servo_on_pwm;      // PWM value to move servo to when shutter is activated
    AP_Int16        _servo_off_pwm;     // PWM value to move servo to when shutter is deactivated
    uint8_t         _trigger_counter;   // count of number of cycles shutter has been held open
    uint8_t         _trigger_counter_cam_function;   // count of number of cycles alternative camera function has been held open
    AP_Int8         _auto_mode_only;    // if 1: trigger by distance only if in AUTO mode.
    AP_Int8         _type;              // Set the type of camera in use, will open additional parameters if set
    //bool            _is_in_auto_mode;   // true if in AUTO mode

    void            servo_pic();        // Servo operated camera
    void            relay_pic();        // basic relay activation
    void            feedback_pin_timer();
    void            feedback_pin_isr(uint8_t, bool, uint32_t);
    void            setup_feedback_callback(void);

    AP_Float        _trigg_dist;        // distance between trigger points (meters)
    AP_Int16        _min_interval;      // Minimum time between shots required by camera
    AP_Int16        _max_roll;          // Maximum acceptable roll angle when trigging camera
    uint32_t        _last_photo_time;   // last time a photo was taken
    struct Location _last_location;
    uint16_t        _image_index;       // number of pictures taken since boot

    // pin number for accurate camera feedback messages
    AP_Int8         _feedback_pin;
    AP_Int8         _feedback_polarity;

    uint32_t        _camera_trigger_count;
    uint32_t        _camera_trigger_logged;
    uint32_t        _feedback_timestamp_us;
    bool            _timer_installed;
    bool            _isr_installed;
    uint8_t         _last_pin_state;

    void log_picture();

//    uint32_t log_camera_bit;
    const struct Location &current_loc;
    
    
    
    

    // entry point to trip local shutter (e.g. by relay or servo)
    void trigger_pic();

    // de-activate the trigger after some delay, but without using a delay() function
    // should be called at 50hz from main program
    void trigger_pic_cleanup();

    // return true if we are using a feedback pin
    bool using_feedback_pin(void) const
    {
        return _feedback_pin > 0;
    }
#endif
};

namespace AP {
AP_Momimaki *momimaki();
};
