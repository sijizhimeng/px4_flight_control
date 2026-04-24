/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <lib/perf/perf_counter.h>                                      //通用库
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <math.h>
#include <mathlib/mathlib.h>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>     //线程库

#include <uORB/uORB.h>                                                  //uorb库
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/pos_helper.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/iusl_para.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/orb_testx.h>
#include <uORB/topics/vehicle_land_detected.h>

#include <drivers/drv_hrt.h>                                          //时钟库

    class WorkItemExample : public ModuleBase<WorkItemExample>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	WorkItemExample();
	~WorkItemExample() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

    int print_status() override;

private:
	void Run() override;

    uORB::Publication<orb_testx_s>                    _orb_test_pub{ORB_ID(orb_testx)};    //发布与订阅
    uORB::SubscriptionData<vehicle_angular_velocity_s> vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::SubscriptionData<vehicle_attitude_s>         vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::SubscriptionData<pos_helper_s>               pos_helper_sub{ORB_ID(pos_helper)};
    uORB::Subscription                                 _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
    uORB::Subscription					               _actuator_controls_sub{ORB_ID(actuator_controls_0)};
    uORB::Subscription                                 para_sub{ORB_ID(iusl_para)};
    uORB::Subscription                                 para_update_sub{ORB_ID(parameter_update)};
    uORB::Subscription                                 vehicle_status_sub{ORB_ID(vehicle_status)};
    uORB::Subscription                                 _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};

    float phi_d;                                      //指令
    float theta_d;
    float phi_d_pre;
    float theta_d_pre;
    float k_q;

    float w_esti_r,w_esti_p,w_esti_y;                  //ESO
    float w_deri_esti_r,w_deri_esti_p,w_deri_esti_y;
    float delta_esti_wr,delta_esti_wp,delta_esti_wy;
    float delta_deri_esti_wr,delta_deri_esti_wp,delta_deri_esti_wy;
    float wa_1;
    float wa_2;
    float wa_3;


    hrt_abstime now, _last_time;
    float dt;
    static int i;                                     //for test
    //static int kf;

    const float _dt_min = 0.00001f;
    const float _dt_max = 0.02f;

    matrix::Vector3f s_v;                            //vectors
    matrix::Vector3f v_r;
    matrix::Vector3f v_r_prev;
    matrix::Vector3f v_r_deri;
    matrix::Vector3f w_r;
    matrix::Vector3f w_r_prev;
    matrix::Vector3f w_r_deri;
    matrix::Vector3f g;
    matrix::Vector3f tau;     //output 2
    matrix::Vector3f v_r1;
    matrix::Vector3f f_iusl;  //output 1


    matrix::Matrix<float,3, 3> lambda_p;            //matrixs
    matrix::Matrix<float,3, 3> lambda_q;
    matrix::Matrix<float,3, 3> K_v;
    matrix::Matrix<float,3, 3> I_b;
    matrix::Matrix<float,3, 3> I_b_inve;
    matrix::Matrix<float,3, 3> K_w;
    matrix::Matrix<float,3, 3> I;

    pos_helper_s pos_helper  {};
    vehicle_angular_velocity_s vehicle_angular_velocity {};   //structs
    vehicle_attitude_s vehicle_attitude {};
    actuator_controls_s actuator_controls {} ;
    manual_control_setpoint_s manual_control_setpoint {};
    vehicle_status_s vehicle_status {};
    vehicle_land_detected_s _vehicle_land_detected {};

   #define G	9.8066f
   #define Mb	3.790f
   #define SUM_mi	0.29f

   void fn_att_eso(float roll_rate,float pitch_rate,float yaw_rate,float u_r,float u_p,float u_y);  //ESO

   perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};                       //线程控制
   perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
