#include "pid.h"

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

/**
  * @brief  init pid parameter
  * @param  pid      the pid struct containing parameters
  * @param[in] kp
  * @param[in] ki
  * @param[in] kd
  * @param[in] i_max
  * @param[in] out_max
  * @param[in] deadband
  * @return none
  */
void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max,
              float deadband){

  if(pid == ((void*)0)) return;
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
  //pid->k_deadband = deadband*deadband;
  pid->k_deadband = deadband;
}

/**
  * @brief  pid reset
  * @param  pid    the pid struct containing parameters
  * @return none
  */
void pid_reset(pid_struct_t * pid){
	pid->err[1] = pid->err[0] = 0.0f;
	pid->i_out = 0.0f;
}

/**
  * @brief  pid calculation
  * @param  pid    the pid struct containing parameters
  * @param  ref    reference value
  * @param  cur    current value
  * @return pid calculation result
  * @note pid->k_deadband is invaild in this process
  */
float pid_calc(pid_struct_t *pid, float ref, float cur){
  float output;
  pid->err[1] = pid->err[0];
  pid->err[0] = ref - cur;
  
  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);
  
  output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(output, -pid->out_max, pid->out_max);
  return output;
}

/**
  * @brief  pid calculation with deadband
  * @param  pid    the pid struct containing parameters
  * @param  ref    reference value
  * @param  cur    current value
  * @return pid calculation result
  * @todo two deadband ways: asymptote or normal?
  */
float pid_calc_deadband(pid_struct_t *pid, float ref, float cur){
  float output;
  pid->err[1] = pid->err[0];

  //float err_square = err*err;
  //pid->err[0] = err*(err_square/(err_square + pid->k_deadband));
  pid->err[0] = ref - cur;
  if(pid->err[0] > pid->k_deadband) {
    pid->err[0] -= pid->k_deadband;
  }
  else if(pid->err[0] < -pid->k_deadband) {
    pid->err[0] += pid->k_deadband;
  }else{
    if(ref < pid->k_deadband && ref > -pid->k_deadband  //reference value get into deadband
		&& (pid->err[1] < -pid->k_deadband || pid->err[1] > pid->k_deadband)){ //last error out of deadband
			pid->i_out = 0.0f;
	}
	if(pid->deadband_zero_output) return 0.0f;
  }
  
  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  LIMIT_MIN_MAX(pid->i_out, -pid->i_max, pid->i_max);
  
  output = pid->p_out + pid->i_out + pid->d_out;
  LIMIT_MIN_MAX(output, -pid->out_max, pid->out_max);
  return output;
}

/**
  * @brief  pid dual loop calculation
  * @param  pid     the inner and outter loop pid struct containing parameters
  * @param  err     error value
  * @param  err_out outter loop feedback value
  * @return pid calculation result
  */
float pid_dual_loop(pid_struct_t pid[2], float err, float err_out){
  float pid_in_output;
  pid_in_output = pid_calc(&pid[ANG_LOOP], err, 0);
  return pid_calc(&pid[RPM_LOOP], pid_in_output, err_out);
}
