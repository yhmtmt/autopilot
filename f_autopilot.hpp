#ifndef F_AUTOPILOT_HPP
#define F_AUTOPILOT_HPP
// Copyright(c) 2016-2020 Yohei Matsumoto, All right reserved. 

// f_autopilot.h is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_autopilot.h is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_autopilot.h.  If not, see <http://www.gnu.org/licenses/>. 

#include "filter_base.hpp"
#include "ch_aws1_ctrl.hpp"
#include "ch_state.hpp"
#include "ch_obj.hpp"
#include "ch_wp.hpp"

// automatically controls along with the waypoints
// connects to ch_wp
class f_autopilot: public f_base
{
protected:
  bool m_verb;                 // verbose for debug

  // related channels
  ch_state * m_state;
  ch_eng_state * m_engstate;
  ch_aws1_ctrl_inst * m_ctrl_inst;
  ch_aws1_ctrl_stat * m_ctrl_stat;
  ch_aws1_ap_inst * m_ap_inst;
  ch_wp * m_wp;
  ch_ais_obj * m_ais_obj;
  
  s_aws1_ctrl_inst m_inst;      // resulting control instruction
  float m_eng;                  // resulting engine control value [0,255]
  float m_rud;                  // resulting rudder cotnrol value [0,255]
  
  // PID parameters
  float m_cdiff, m_sdiff, m_revdiff;   // course, speed and rev error to target
  float m_dcdiff, m_dsdiff, m_drevdiff;// diff of course, speed and rev error
  float m_icdiff, m_isdiff, m_irevdiff;// and their integration.
  float m_prev, m_irev, m_drev;        // PID coefficient for rev control
  float m_pc, m_ic, m_dc;              // PID coefficient for course control
  float m_ps, m_is, m_ds;              // PID coefficient for speed control
  
  // limitter 
  float m_smax, m_smin;         // speed limit (absolute value)
  float m_rev_max, m_rev_min;   // rev limit (absolute value)
  float m_eng_max, m_eng_min;   // limit for engine control value [0, 255]

  unsigned short deng, drud;    // difference of eng, rud control value
  unsigned short eng_prev;      // previous eng
  unsigned short rud_prev;      // previous rud
  
  // related to control state estimation
  float dcog, dsog, drev;       // derivative of cog, sog, rev

  float crs_flw, spd_flw;       // course and speed of flow estimated
  float alpha_flw;              // flow update factor
  bool is_rud_ltor;             // true if rudder is moving from left to right
  float roll_prev;              // previous roll
  float pitch_prev;             // previous pitch
  float yaw_prev;               // previous yaw
  float cog_prev;               // previous cog
  float sog_prev;               // previous sog
  float rev_prop_prev;          // previous propeller rev
  long long tatt_prev;          // updated time of previous attitude
  long long tcog_prev;          // updated time of previous cog
  long long tsog_prev;          // updated time of previous sog
  long long trev_prop_prev;     // updated time of previous propeller rev
  float devyaw;                 // allowable deviation of yaw to be stable
  float devcog;                 // allowable deviation of cog to be stable
  float devsog;                 // allowable deviation of sog to be stable
  float devrev;                 // allowable deviation of rev to be stable
  
  long long twindow_stability_check; // time window for stability check
  int twindow_stability_check_sec;   // twindow_stability_check(in second)
  long long tbegin_stable;           // the time yaw/cog/rev stabilized
  float yaw_stbl;                    // yaw in stable condition
  float cog_stbl;                    // cog in stable condition
  float rev_stbl;                    // rev in stable condition
  float sog_stbl;                    // sog in stable condition

  float yaw_bias;                   // estimated yaw bias of heading sensor
                                    // (estimated as the average of the
                                    //      difference between cog and yaw)
  float alpha_yaw_bias;             // yaw bias update coefficient.
  float u, v, angle_drift;          // (u, v) and arctan(v/u) from (cog, sog)
                                    // with yaw_bias correction
  float uflw, vflw, angle_flw;      // estimated flow velocity
  float ucor, vcor, angle_drift_cor;// estimated correct velocity
  float sog_cor, cog_cor;           // sog,cog version of the ucor,vcor
  float rev_prop;                   // propeller rev (backward negative)


  // if the boat is in stable condition, yaw-bias and water-flow is estimated.
  void estimate_stat(const long long tvel, const float cog,
		     const float sog,
		     const long long tatt, const float roll,
		     const float pitch, const float yaw,
		     const long long trev, const float rev,
		     const s_aws1_ctrl_stat & stat);

  // is_stable() determines whether the boat is in stable condition.
  // The function watches the stability of yaw, cog, sog, and rev,
  // whether their deviations are less than allowable deviation values
  // devyaw, devcog, devsog and dev rev within the specified time
  // twindow_stability_check
  bool is_stable(const float cog, const float sog,
		 const float yaw, const float rev);
  
  
  float tbl_stable_rpm[60];	// stable rev-eng table in forward gear 
  float tbl_stable_nrpm[60];    // stable rev-eng table in backward gear
  float alpha_tbl_stable_rpm;   // update coefficient stable rev-eng tables
  float rudmidlr, rudmidrl;     // control values of rudder is in midship.
                                // the values could change due to the
                                // moving direction of rudder: L to R or R to L.
  float alpha_rud_mid;          // update speed of midship control values
  
  void monotonize_tbl_stable_rpm(int i = 0)
  {
    float vprev = tbl_stable_rpm[i];
    for (i+=1;i < 60;i++){
      if(tbl_stable_rpm[i] < vprev){
	tbl_stable_rpm[i] = vprev;
      }else{
	vprev = tbl_stable_rpm[i];
      }
    }
  }
  void monotonize_tbl_stable_nrpm(int i = 0)
  {
    float vprev = tbl_stable_nrpm[i];
    for (i+=1;i < 60;i++){
      if(tbl_stable_nrpm[i] > vprev){
	tbl_stable_nrpm[i] = vprev;
      }else{
	vprev = tbl_stable_nrpm[i];
      }
    }   
  }

  char tbl_spd_rpm[30];
   
  // related to auto avoidance 
  float m_Lo, m_Wo;             // assumed size for my own ship
  float m_Lais, m_Wais;         // assumed size for ais target
  float m_Rav;                  // range for avoidance(multiple of ship size)
  float m_Tav;                  // time for avoidance
  float m_Cav_max;              // maximum course change in degree

  
  const float calc_course_change_for_ais_ship(const float yaw);
  void ctrl_to_sog_cog(const float sog, const float sog_tgt,
		       const float cdiff, const float smax, const float smin);
  void ctrl_to_cog(const float cdiff);
  void ctrl_to_sog(const float sog, const float sog_tgt,
		   const float smax, const float smin);
  void ctrl_to_rev(const float rev, const float rev_tgt,
		   const float rev_max, const float rev_min);
  void stb_man(const float cog, const float rev);
  void flw_tgt(const float sog, const float cog, const float yaw, bool bav = false);
  void wp(const float sog, const float cog, const float yaw, bool bav = false);
  void stay(const float sog, const float cog, const float yaw);
  void cursor(const float sog, const float cog, const float yaw, bool bav = false);
   
  char fctrl_state[1024];	
  void save_ctrl_state();
  void load_ctrl_state();
 public:
  f_autopilot(const char * name);
  virtual ~f_autopilot();
  
  virtual bool init_run();
  virtual void destroy_run();
  virtual bool proc();
};

#endif
