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


    // @Param: DBG_SPD
    // @DisplayName: virtual vehicle_speed for debug of Momimaki
    // @Description: virtual vehicle_speed for debug of Momimaki
    // @User: Standard
    // @Units: m/s
    // @Range: -1～20
    AP_GROUPINFO("DBG_SPD",  6, AP_Momimaki, _debug_vehicle_speed, -1 ),

    // @Param: RT_CTRL
    // @DisplayName: Route_Control( use in dronekit-python )
    // @Description: Route_Control( use in dronekit-python )
    // @User: Standard
    // @Units: none
    // @Range: 1,2,3...
    AP_GROUPINFO("RT_CTRL",  7, AP_Momimaki, _route_ctrl, 0 ),

    AP_GROUPEND
};

extern const AP_HAL::HAL& hal;




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

}


/*
 * 籾送り、籾播きを動作させるかチェックする
 */
void AP_Momimaki::status_check( bool& feeder_sts, bool& spreader_sts)
{
    feeder_sts = false;
    spreader_sts = false;




#if true
    // for debug AutoMode以外でも動かす
    if( _debug_vehicle_speed > 0.0 )
    {
        feeder_sts = true;
        spreader_sts = true;

        _radius = _default_radius;
        _density = _default_density;    // パラメータ値

        return;
    }

#endif




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
    
//   gcs().send_text(MAV_SEVERITY_NOTICE, "update() was called." );


    // 籾播ききを動作させるか決める
    status_check( feeder_sts, spreader_sts );

#if false
    // 機体速度
    float forward_speed;
    if (!atti_ctrl.get_forward_speed(forward_speed) ) {
        feeder_sts      = false;
        spreader_sts    = false;
    }
#else
    // 機体速度
    float forward_speed;
    // debug用 機体速度をパラメータで与える。
    if( _debug_vehicle_speed < 0.0 )
    {
        if (!atti_ctrl.get_forward_speed(forward_speed) ) {
            feeder_sts      = false;
            spreader_sts    = false;
        }
    }
    else
    {
        forward_speed = _debug_vehicle_speed;

        // LED_実験
        // _debug_vehicle_speedにてLEDパターン変えてみる
        // for test of LED override
        static float tmp_dbg_spd = -9999;
        if( fabs( tmp_dbg_spd - _debug_vehicle_speed ) > 0.1  )
        {
            led_override_debug( _debug_vehicle_speed );
            tmp_dbg_spd = _debug_vehicle_speed;
        }
    }
#endif
    

    if( feeder_sts )
    {
        // 機体速度に応じても見送り量を決める
        // 1秒あたりの籾送り量 momi_num : [pcs / sec ]
        float momi_num = _density * _radius * forward_speed * radians(_angle) / 2.0;


        // 1秒あたりの籾送り量から籾送りモーターの出力（Rate）を決める。
        float feed_rate = Calc_Momiokuri_FeedRate( momi_num );
        // 籾送り量　PWM出力
        pwm_output( SRV_Channel::k_momimaki_feeder, feed_rate );

  //      gcs().send_text(MAV_SEVERITY_NOTICE, "SRV_Channel::k_momimaki_feeder = %f", feed_rate );

    } else {
        pwm_output( SRV_Channel::k_momimaki_feeder, 0.0 );
    }
    

    if( spreader_sts )
    {
        // 籾拡散出力（Rate）
        float spread_rate = _radius / _spreader_max_radius;
        // 籾拡散量　PWM出力
        pwm_output( SRV_Channel::k_momimaki_spreader, spread_rate );

      //  float tmpradius = _spreader_max_radius;
 //       gcs().send_text(MAV_SEVERITY_NOTICE, "momimaki_spreader = %f(%f/%f)", spread_rate, _radius , tmpradius);
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

}

#if false
// LED(override）設定を行う
void AP_Momimaki::led_override_debug( uint8_t led_color )
{
    mavlink_led_control_t pack;
    pack.target_system      = 1;    /*<  System ID.*/
    pack.target_component   = 1;    /*<  Component ID.*/
    pack.instance           = 255;  /*<  Instance (LED instance to control or 255 for all LEDs).*/
    pack.pattern            = 0;    /*<  Pattern (see LED_PATTERN_ENUM).*/
    pack.custom_len         = 4;    /*<  Custom Byte Length.*/
    pack.custom_bytes[0]    = 0;    // _led_override.r;
    pack.custom_bytes[1]    = 0;    // _led_override.g;
    pack.custom_bytes[2]    = 0;    // _led_override.b;
    pack.custom_bytes[3]    = 0;    // _led_override.rate_hz;


    // LEDパターン作成
    pack.custom_bytes[0] = ( led_color & 0x02 ) ? 255 : 0; // _led_override.r;
    pack.custom_bytes[1] = ( led_color & 0x04 ) ? 255 : 0; // _led_override.g;
    pack.custom_bytes[2] = ( led_color & 0x01 ) ? 255 : 0; // _led_override.b;

    // MAVLINKメッセージ作成
    mavlink_message_t msg;
    mavlink_msg_led_control_encode( 1, 1, &msg, &pack );

    // 設定
    AP_Notify::handle_led_control(msg);
}
#endif


// LED overrideを解除する
void AP_Momimaki::reset_led_override(void)
{
    // LED overrideを解除する
    AP_Notyfy::set_rgb_led_override(false);
}

// LED overrideにて点灯する。
void AP_Momimaki::set_led_override( float led_status )
{
    // LED overrideを設定する
    AP_Notyfy::set_rgb_led_override(true);


    // メッセージ経由で点灯パターンを設定する
    mavlink_led_control_t pack;
    pack.target_system      = 1;    /*<  System ID.*/
    pack.target_component   = 1;    /*<  Component ID.*/
    pack.instance           = 255;  /*<  Instance (LED instance to control or 255 for all LEDs).*/
    pack.pattern            = 0;    /*<  Pattern (see LED_PATTERN_ENUM).*/
    pack.custom_len         = 4;    /*<  Custom Byte Length.*/
    pack.custom_bytes[0]    = 0;    // _led_override.r;
    pack.custom_bytes[1]    = 0;    // _led_override.g;
    pack.custom_bytes[2]    = 0;    // _led_override.b;
    pack.custom_bytes[3]    = 0;    // _led_override.rate_hz;


    // LEDパターン作成
    uint8_t ledsts = (uint8_t)(led_status + 0.5);
    pack.custom_bytes[0] = ( ledsts & 0x02 ) ? 255 : 0; // _led_override.r;
    pack.custom_bytes[1] = ( ledsts & 0x04 ) ? 255 : 0; // _led_override.g;
    pack.custom_bytes[2] = ( ledsts & 0x01 ) ? 255 : 0; // _led_override.b;

    // MAVLINKメッセージ作成
    mavlink_message_t msg;
    //static inline uint16_t mavlink_msg_led_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_led_control_t* led_control)
    mavlink_msg_led_control_encode( 1, 1, &msg, &pack );

    // 設定
    AP_Notify::handle_led_control(msg);


}


// singleton instance
AP_Momimaki *AP_Momimaki::_singleton;

namespace AP {

AP_Momimaki *momimaki()
{
    return AP_Momimaki::get_singleton();
}

}
