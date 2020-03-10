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



/// @class	Momimaki
/// @brief	Object managing momimaki unit
class AP_Momimaki {

public:
    AP_Momimaki(const AR_AttitudeControl &_atti_ctrl)
        :atti_ctrl(_atti_ctrl)
    {
        AP_Param::setup_object_defaults(this, var_info);
        _singleton = this;

        _enable_spreader = false;
        _enable_feeder = false;

        _density = _default_density;
        _radius = _default_radius;



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
    void control(int8_t enable_spreader, int8_t enable_feeder, float spread_radius, float spread_density );


    // Update - to be called periodically @at least 10Hz
    void update();

    static const struct AP_Param::GroupInfo        var_info[];

    // set if vehicle is in AUTO mode
    void set_is_auto_mode(bool enable)
    {
        _is_in_auto_mode = enable;
    }


private:

    static AP_Momimaki *_singleton;

    // AP_Momimaki霑ｽ蜉�繝代Λ繝｡繝ｼ繧ｿ
    AP_Float        _default_density;           // density of sowing (pcs / m^2 )
    AP_Float        _default_radius;            // radius of sowing area
    AP_Int8         _angle;             // angle of sowing area
    //AP_Float        _r_to_pwm;          // radius to pwm convert rate
    //AP_Float        _feed_to_pwm;       // feed_to_pwm convaert rate

    AP_Float        _feeder_max_rpm;      // max rpm of feeder gear at full throttle
    AP_Float        _feed_num_par_rotate; // feed num per gear rotation
    AP_Float        _spreader_max_radius;  // max radius of spreading at full throttle

//    AP_Int8         _feeder_pwm_ch;     // 籾送り出力CH
//    AP_Int8     _spreader_pwm_ch;   // 籾拡散出力CH

    AP_Float        _debug_vehicle_speed;  // デバッグ用に機体速度を外部から与える。

	AP_Float		_route_ctrl;		// ルート管理用パラメータ（dronekit-pythonにて使用)
	


    void status_check( bool& feeder_sts, bool& spreader_sts);

    float Calc_Momiokuri_FeedRate( float tgt_num_per_sec );

    float Calc_Momiokuri_SpreadRate(void);

    void pwm_output( SRV_Channel::Aux_servo_function_t function, float value );

    // for test of LED override
    //void led_override_debug( float led_statrus );

    // LED overrideを解除する
    void reset_led_override(void);
    // LED overrideにて点灯する。
    void set_led_override( float led_status );

    
//    Mode::Number    _mode_number;
    bool            _is_in_auto_mode;   // true if in AUTO mode
    const AR_AttitudeControl &atti_ctrl;

    bool _enable_spreader;       // 籾拡散 ON/OFF
    bool _enable_feeder;        // 籾送り ON/OFF

    float _density;
    float _radius;



};

namespace AP {
AP_Momimaki *momimaki();
};
