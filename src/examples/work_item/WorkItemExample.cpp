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

#include "WorkItemExample.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;
using namespace matrix;


WorkItemExample::WorkItemExample() :
	ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)  //test1
{
}

WorkItemExample::~WorkItemExample()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool WorkItemExample::init()
{
    ScheduleOnInterval(2000_us); // 1000 us interval, 1000 Hz rate

    lambda_q.identity();
    lambda_q(0,0)=1.5f;
    lambda_q(1,1)=1.5f;
    lambda_q(2,2)=2.1f;



  //  I_b.identity();                     //飞虎
  //  I_b(0,0)=0.2039f;
  //  I_b(1,1)=0.2039f;
  //  I_b(2,2)=0.3036f;
  //  I_b_inve.identity();
  //  I_b_inve(0,0)=4.9043f;
  //  I_b_inve(1,1)=4.9043f;
  //  I_b_inve(2,2)=3.2938f;

  //  I_b.identity();                //amov
  //  I_b(0,0)=0.0284f;
  //  I_b(1,1)=0.0284f;
  //  I_b(2,2)=0.03357f;
  //  I_b_inve.identity();
  //  I_b_inve(0,0)=35.211f;
  //  I_b_inve(1,1)=35.211f;
  //  I_b_inve(2,2)=29.788f;

    I_b.identity();              //仿真
    I_b(0,0)=0.03f;
    I_b(1,1)=0.03f;
    I_b(2,2)=0.06f;
    I_b_inve.identity();
    I_b_inve(0,0)=33.3333f;
    I_b_inve(1,1)=33.3333f;
    I_b_inve(2,2)=16.6666f;

    K_w.identity();
    K_w(0,0)=1.0f;
    K_w(1,1)=1.0f;
    K_w(2,2)=1.5f;

    g(0)=0.0f;
    g(1)=0.0f;
    g(2)=G;

    phi_d=0.0f;
    theta_d=0.0f;
    phi_d_pre=0.0f;
    theta_d_pre=0.0f;
    k_q=2.0f;

    f_iusl(0)=0.0f;
    f_iusl(1)=0.0f;
    f_iusl(2)=0.0f;

    w_esti_r=0;
    w_esti_p=0;
    w_esti_y=0;
    w_deri_esti_r=0;
    w_deri_esti_p=0;
    w_deri_esti_y=0;
    delta_esti_wr=0;
    delta_esti_wp=0;
    delta_esti_wy=0;
    delta_deri_esti_wr=0;
    delta_deri_esti_wp=0;
    delta_deri_esti_wy=0;
    wa_1=8.0f;
    wa_2=8.0f;
    wa_3=8.0f;
	return true;
}

void WorkItemExample::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);


    // DO WORK
    if (para_update_sub.updated()) {
        //  update param from qgc
        iusl_para_s iusl_param;

        para_sub.copy(&iusl_param);

        lambda_q(0,0)=iusl_param.lambda_q_x;
        lambda_q(1,1)=iusl_param.lambda_q_y;
        lambda_q(2,2)=iusl_param.lambda_q_z;

        K_w(0,0)=iusl_param.iusl_kw_x;
        K_w(1,1)=iusl_param.iusl_kw_y;
        K_w(2,2)=iusl_param.iusl_kw_z;

        k_q=iusl_param.iusl_kq;

        wa_1=iusl_param.iusl_wa1;
        wa_2=iusl_param.iusl_wa2;
        wa_3=iusl_param.iusl_wa3;


    }

                now = hrt_absolute_time();                   //系统绝对时间
                dt = math::constrain((now - _last_time) / 1e6f, _dt_min, _dt_max);
                _last_time = now;

                vehicle_angular_velocity_sub.copy(&vehicle_angular_velocity);  //订阅topic
                pos_helper_sub.copy(&pos_helper);
                vehicle_attitude_sub.copy(&vehicle_attitude);
                _actuator_controls_sub.copy(&actuator_controls);
                vehicle_status_sub.copy(&vehicle_status);

                matrix::Quatf q_att(vehicle_attitude.q);
                matrix::Dcmf _R(q_att);

                f_iusl(0)=pos_helper.thrust_sp[0];
                f_iusl(1)=pos_helper.thrust_sp[1];
                f_iusl(2)=pos_helper.thrust_sp[2];

                float f_iusl_abs=f_iusl.length();
                float temp1=sinf(pos_helper.psi_d);
                float temp2=cosf(pos_helper.psi_d);

                 phi_d_pre=phi_d;
                 theta_d_pre=theta_d;
                 phi_d=-asinf((f_iusl(0)*temp1-f_iusl(1)*temp2)/f_iusl_abs);

                 theta_d=0.0f;                                    //actan计算需要处理
                //theta_d=0.0f;
                if((f_iusl(2)<(-0.001f))||(f_iusl(2)>(0.001f)))
               {
                    theta_d=atanf((f_iusl(0)*temp2+f_iusl(1)*temp1)/f_iusl(2));//-pai/2~pai/2
               }else{
                    theta_d=0.0f;
               }

//test
                Dcmf R_d=Eulerf(phi_d,theta_d,pos_helper.psi_d);
                matrix::Dcmf matrix1_temp=R_d.transpose() * _R ;

                float t = matrix1_temp.trace();
                t = sqrt(1.0f + t);
                float q_error_r=0.5f*t;
                Vector3f q_error;
                t = (0.5f)/ t;
                q_error(0) = (matrix1_temp(2,1) - matrix1_temp(1,2)) * t;
                q_error(1) = (matrix1_temp(0,2) - matrix1_temp(2,0)) * t;
                q_error(2) = (matrix1_temp(1,0) - matrix1_temp(0,1)) * t;


                w_r_prev=w_r;
                w_r=(lambda_q*q_error)*(-2);

                float Q[9]={0,-q_error(2),q_error(1),q_error(2),0,-q_error(0),-q_error(1),q_error(0),0};
                matrix::Matrix<float,3, 3> Q_v(Q);


                matrix::Vector3f w_iusl{vehicle_angular_velocity.xyz};

                //test / log
                matrix::Eulerf euler(q_att);
                float roll_body=euler.phi();
                float theta_body=euler.theta();
                float yaw_body=euler.psi();
                //test
                matrix::Vector3f q_v_dot=(I*q_error_r+Q_v)*w_iusl*0.5f;
                w_r_deri=(lambda_q*q_v_dot)*(-2);

                //test / log
                matrix::Vector3f s_w=w_iusl-w_r;
                matrix::Vector3f tau_temp{actuator_controls.control[actuator_controls_s::INDEX_ROLL] ,actuator_controls.control[actuator_controls_s::INDEX_PITCH],actuator_controls.control[actuator_controls_s::INDEX_YAW]};
                tau_temp=tau_temp*5.0f;
                //test / log

                matrix::Vector3f u_w=(matrix::Vector3f(I_b_inve*tau_temp))-((matrix::Vector3f(I_b_inve*w_iusl))%(matrix::Vector3f(I_b*w_iusl)));

                _vehicle_land_detected_sub.copy(&_vehicle_land_detected);

                if(((vehicle_status.nav_state==vehicle_status_s::NAVIGATION_STATE_ACRO)||(vehicle_status.nav_state==vehicle_status_s::NAVIGATION_STATE_OFFBOARD))&&(vehicle_status.arming_state==vehicle_status_s::ARMING_STATE_ARMED)&&(!(_vehicle_land_detected.maybe_landed||_vehicle_land_detected.ground_contact||_vehicle_land_detected.landed)))
                {
                    fn_att_eso(w_iusl(0),w_iusl(1),w_iusl(2),u_w(0),u_w(1),u_w(2));

                }else {
                    w_esti_r=0;
                    w_esti_p=0;
                    w_esti_y=0;
                    w_deri_esti_r=0;
                    w_deri_esti_p=0;
                    w_deri_esti_y=0;
                    delta_esti_wr=0;
                    delta_esti_wp=0;
                    delta_esti_wy=0;
                    delta_deri_esti_wr=0;
                    delta_deri_esti_wp=0;
                    delta_deri_esti_wy=0;
                }

                matrix::Vector3f vector_delta_esti_w{delta_esti_wr,delta_esti_wp,delta_esti_wy};

                tau = I_b*(w_r_deri-vector_delta_esti_w)+w_iusl%(I_b*w_iusl)-(K_w*s_w)-(q_error*k_q);
                tau(1) = math::constrain(tau(1), -5.0f, 5.0f);
                tau(0) = math::constrain(tau(0), -5.0f, 5.0f);
                tau(2) = math::constrain(tau(2), -5.0f, 5.0f);

    // publish  data
    orb_testx_s data{};
	data.timestamp = hrt_absolute_time();
    //data.val = accel.device_id;
    data.torque_sp[0]=tau(0);
    data.torque_sp[1]=tau(1);
    data.torque_sp[2]=tau(2);    
    data.phi_d=phi_d;
    data.theta_d=theta_d;
    data.q_erro[0]=q_error(0);
    data.q_erro[1]=q_error(1);
    data.q_erro[2]=q_error(2);
    data.phi=roll_body;
    data.theta=theta_body;
    data.psi=yaw_body;
    data.w_esti[0]=w_esti_r;
    data.w_esti[1]=w_esti_p;
    data.w_esti[2]=w_esti_y;
    data.delta_esti_w[0]=delta_esti_wr;
    data.delta_esti_w[1]=delta_esti_wp;
    data.delta_esti_w[2]=delta_esti_wy;
    data.delta_deri_esti_wr=delta_deri_esti_wr;
    data.w_deri_esti_r=w_deri_esti_r;
    data.u_w=u_w(0);
    data.test1=k_q;
    data.test2=lambda_q(0,0);
    data.test3=K_w(0,0);
    data.test4=wa_1;
	_orb_test_pub.publish(data);

	perf_end(_loop_perf);
}

int WorkItemExample::task_spawn(int argc, char *argv[])
{
	WorkItemExample *instance = new WorkItemExample();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int WorkItemExample::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int WorkItemExample::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WorkItemExample::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Example of a simple module running out of a work queue.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
void WorkItemExample::fn_att_eso(float roll_rate,float pitch_rate,float yaw_rate,float u_r,float u_p,float u_y)
{
    w_deri_esti_r=u_r+2*wa_1*(roll_rate-w_esti_r)+delta_esti_wr;
    delta_deri_esti_wr=wa_1*wa_1*(roll_rate-w_esti_r);
    w_esti_r=w_esti_r+w_deri_esti_r*dt;
    delta_esti_wr=delta_esti_wr+delta_deri_esti_wr*dt;

    w_deri_esti_p=u_p+2*wa_2*(pitch_rate-w_esti_p)+delta_esti_wp;
    delta_deri_esti_wp=wa_2*wa_2*(pitch_rate-w_esti_p);
    w_esti_p=w_esti_p+w_deri_esti_p*dt;
    delta_esti_wp=delta_esti_wp+delta_deri_esti_wp*dt;

    w_deri_esti_y=u_y+2*wa_3*(yaw_rate-w_esti_y)+delta_esti_wy;
    delta_deri_esti_wy=wa_3*wa_3*(yaw_rate-w_esti_y);
    delta_esti_wy=delta_esti_wy+delta_deri_esti_wy*dt;
    w_esti_y=w_esti_y+w_deri_esti_y*dt;

}


extern "C" __EXPORT int work_item_example_main(int argc, char *argv[])
{
	return WorkItemExample::main(argc, argv);
}
