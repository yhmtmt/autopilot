// Copyright(c) 2020 Yohei Matsumoto, All right reserved. 

// f_autopilot.cpp is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// f_autopilot.cpp is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with f_autopilot.cpp.  If not, see <http://www.gnu.org/licenses/>. 

#include "f_autopilot.hpp"
#include "autopilot.pb.h"
#include <google/protobuf/util/json_util.h>
DEFINE_FILTER(f_autopilot)

f_autopilot::f_autopilot(const char * name) :
f_base(name), 
  m_state(NULL), m_engstate(NULL), m_ctrl_inst(NULL), m_ctrl_stat(NULL), 
  m_ap_inst(NULL), m_ais_obj(NULL), m_verb(false),
  m_wp(NULL), m_eng(127.), m_rud(127.), 
  m_smax(10), m_smin(3), m_rev_max(5500), m_rev_min(700),
  m_eng_max(200), m_eng_min(80),
  devyaw((float)(PI * 3.0f/180.f)), devcog((float)(PI * 3.0f/180.f)), devsog(1.0f), devrev(500.f),
  m_pc(0.1f), m_ic(0.1f), m_dc(0.1f), m_ps(0.1f), m_is(0.1f), m_ds(0.1f),
  m_cdiff(0.f), m_sdiff(0.f), m_revdiff(0.f),
  m_dcdiff(0.f), m_icdiff(0.f),
  m_dsdiff(0.f), m_isdiff(0.f),
  m_drevdiff(0.f), m_irevdiff(0.f),
  m_prev(0.1f), m_irev(0.1f), m_drev(0.1f),
  rudmidlr(127.0f), rudmidrl(127.0f),
  alpha_tbl_stable_rpm(0.01f),
  alpha_rud_mid(0.01f),
  alpha_flw(0.1f),
  alpha_yaw_bias(0.1f),
  twindow_stability_check_sec(3),
  m_Lo(8), m_Wo(2), m_Lais(400), m_Wais(80), m_Rav(3), m_Tav(300), m_Cav_max(45),
  yaw_bias(0.0f)
{
  register_fpar("ch_state", (ch_base**)&m_state, typeid(ch_state).name(), "State channel");
  register_fpar("ch_engstate", (ch_base**)&m_engstate, typeid(ch_eng_state).name(), "Engine State channel.");	
  register_fpar("ch_ctrl_inst", (ch_base**)&m_ctrl_inst, typeid(ch_aws1_ctrl_inst).name(), "Ctrl instruction channel");
  register_fpar("ch_ctrl_stat", (ch_base**)&m_ctrl_stat, typeid(ch_aws1_ctrl_stat).name(), "Ctrl status channel");
  register_fpar("ch_wp", (ch_base**)&m_wp, typeid(ch_wp).name(), "Waypoint channel");
  register_fpar("ch_ap_inst", (ch_base**)&m_ap_inst, typeid(ch_aws1_ap_inst).name(), "Autopilot instruction channel");
  register_fpar("ch_ais_obj", (ch_base**)&m_ais_obj, typeid(ch_ais_obj).name(), "AIS object channel.");
  register_fpar("verb", &m_verb, "Verbose for debug.");
  register_fpar("rud", &m_inst.rud_aws, "Rudder value");
  register_fpar("eng", &m_inst.eng_aws, "Main engine value");
  
  register_fpar("smax", &m_smax, "Maximum speed in knot");
  register_fpar("smin", &m_smin, "Minimum speed in knot");
  register_fpar("eng_max", &m_eng_max, "The maximum value for engine control.");
  register_fpar("eng_min", &m_eng_min, "The minimum value for engine control.");
  register_fpar("rev_max", &m_rev_max, "Maximum rev value in RPM");
  register_fpar("rev_min", &m_rev_min, "Minimum rev value in RPM");

  register_fpar("pc", &m_pc, "Coefficient P in the course control with PID.");
  register_fpar("ic", &m_ic, "Coefficient I in the course control with PID.");
  register_fpar("dc", &m_dc, "Coefficient D in the course control with PID.");

  register_fpar("ps", &m_ps, "Coefficient P in the speed control with PID.");
  register_fpar("is", &m_is, "Coefficient I in the speed control with PID.");
  register_fpar("ds", &m_ds, "Coefficient D in the speed control with PID.");

  register_fpar("prev", &m_prev, "Coefficient P in the rev control with PID.");
  register_fpar("irev", &m_irev, "Coefficient I in the rev control with PID.");
  register_fpar("drev", &m_drev, "Coefficient D in the rev control with PID.");
 
  register_fpar("lo", &m_Lo, "Length of my own ship in meter.");
  register_fpar("wo", &m_Wo, "Width of my own ship in meter.");
  register_fpar("lais", &m_Lais, "Assumed length of ais ship in meter.");
  register_fpar("wais", &m_Wais, "Assumed width of ais ship in meter.");
  register_fpar("rav", &m_Rav, "Range for avoidance (multiple of ship size.)");
  register_fpar("tav", &m_Tav, "Time for avoidance (second)");
  register_fpar("cav_max", &m_Cav_max, "Maximum course change for avoidance");

  register_fpar("devyaw", &devyaw, "Yaw Deviation in deg");
  register_fpar("devcog", &devcog, "COG Deviation in deg");
  register_fpar("devsog", &devsog, "SOG Deviation in deg");
  register_fpar("devrev", &devrev, "Rev Deviation in deg");
  
  register_fpar("alpha_tbl_stable_rpm", &alpha_tbl_stable_rpm, "Update rate of stable rpm table");
  register_fpar("alpha_rud_mid", &alpha_rud_mid, "Update rate of midship rudder position.");
  register_fpar("alpha_flw", &alpha_flw, "Update rate of local flow");
  register_fpar("alpha_yaw_bias", &alpha_yaw_bias, "Update rate of yaw bias");
  register_fpar("twindow_stability_check", &twindow_stability_check_sec, "Window stability check in second.");

  fctrl_state[0] = '\0';
  register_fpar("fctrl_state", fctrl_state, sizeof(fctrl_state), "Autopilot Control State");
  
  tbl_spd_rpm[0] = 0.f;
  tbl_spd_rpm[1] = 0.f;
  for (int i=2; i < 30; i++){
    if (i < 23)
      tbl_spd_rpm[i] = (float)(7 + (double)(i - 2) * (56.0 - 7.0) / 20.0);  
    else
      tbl_spd_rpm[i] = 56.0f;
  }
}

f_autopilot::~f_autopilot()
{
}

bool f_autopilot::init_run()
{
  if(!m_state){
    cerr << "Error in f_autopilot::init_run(). ";
    cerr << "state is not connected." << endl;
    return false;
  }

  if(!m_engstate){
    cerr << "Error in f_autopilot::init_run(). ";
    cerr << "engstate is not connected." << endl;
    return false;
  }

  if(!m_ctrl_inst){
    cerr << "Error in f_autopilot::init_run(). ";
    cerr << "ctrl_inst is not connected." << endl;
    return false;
  }

  if(!m_ctrl_stat){
    cerr << "Error in f_autopilot::init_run(). ";
    cerr << "ctrl_stat is not connected." << endl;
    return false;
  }

  if(!m_ap_inst){
    cerr << "Error in f_autopilot::init_run(). ";
    cerr << "ap_inst is not connected." << endl;
    return false;
  }

  twindow_stability_check = twindow_stability_check_sec * SEC;
  crs_flw = spd_flw = 0.0f;
  roll_prev = pitch_prev = yaw_prev
    = cog_prev = sog_prev = rev_prop_prev = 0.0f;
  tatt_prev = tcog_prev = tsog_prev = trev_prop_prev = 0;
  tbegin_stable = -1;

  if(fctrl_state[0] != '\0'){
    load_ctrl_state();
  }
  return true;
}

void f_autopilot::destroy_run()
{
  if(fctrl_state[0] != '\0'){
    save_ctrl_state();
  }
}

void f_autopilot::save_ctrl_state()
{
  ofstream file(fctrl_state);
  if(!file.is_open()){
    cerr << "f_autopilot::save_ctrl_state() failed to open file " << fctrl_state << "." << endl;
    return;
  }
  // yaw_bias
  // rudmidlr, rudmidrl
  ApControlState ctrl_state;
  ctrl_state.set_yaw_bias(yaw_bias);
  ctrl_state.set_rudmidlr(rudmidlr);
  ctrl_state.set_rudmidrl(rudmidrl);
  for (int i = 0; i < 60; i++){
    ctrl_state.add_tbl_stable_rpm(tbl_stable_rpm[i]);
    ctrl_state.add_tbl_stable_nrpm(tbl_stable_nrpm[i]); 
  }
  for (int i = 0; i < 30; i++){
    ctrl_state.add_tbl_spd_rpm(tbl_spd_rpm[i]);
    ctrl_state.add_tbl_nspd_nrpm(tbl_nspd_nrpm[i]);
  }
  
  string str_fctrl_state(fctrl_state);
  if(str_fctrl_state.substr(str_fctrl_state.find_last_of(".") + 1) == "json"){
    // for json file.
    string json_string;
    google::protobuf::util::JsonPrintOptions options;
    options.add_whitespace = true;
    options.always_print_primitive_fields = true;
    options.preserve_proto_field_names = true;
    if(!google::protobuf::util::MessageToJsonString(ctrl_state, &json_string, options).ok()){
      spdlog::error("[{}] Failed to save {}.", get_name(), fctrl_state);
      return;
    }
    file << json_string;
  }else{ // for binary file
    if(!ctrl_state.SerializeToOstream(&file)){
      spdlog::error("[{}] Failed to save {}.", get_name(), fctrl_state);
    }
  }
}

void f_autopilot::load_ctrl_state()
{
  ifstream file(fctrl_state);
  if(!file.is_open()){
    spdlog::error("[{}] load_ctrl_state() failed to open file {}.",
		  get_name(), fctrl_state);
    return;
  }
  
  ApControlState ctrl_state;
  string str_fctrl_state(fctrl_state);
  if(str_fctrl_state.substr(str_fctrl_state.find_last_of(".") + 1) == "json"){
    // for json file.
    stringstream strstream;
    strstream << file.rdbuf();
    string json_string(strstream.str());
    google::protobuf::util::Status st =
      google::protobuf::util::JsonStringToMessage(json_string, &ctrl_state);
    if(!st.ok()){
      spdlog::error("[{}] Failed to load {}.", get_name(), fctrl_state);
      return;
    }else{
      spdlog::info("[{}] {} successfully loaded.", get_name(), fctrl_state);
    }
  }else{ // for binary file
    if(!ctrl_state.ParseFromIstream(&file)){
      spdlog::error("[{}] Failed to load {}.", get_name(), fctrl_state);
      return;
    }
  }
  yaw_bias = ctrl_state.yaw_bias();
  rudmidlr = ctrl_state.rudmidlr();
  rudmidrl = ctrl_state.rudmidrl();
  if(ctrl_state.tbl_stable_rpm_size() == 60 &&
     ctrl_state.tbl_stable_nrpm_size() == 60){
    for (int i = 0; i < 60; i++){
      tbl_stable_rpm[i] = ctrl_state.tbl_stable_rpm(i);
      tbl_stable_nrpm[i] = ctrl_state.tbl_stable_nrpm(i);
    }
  }

  if(ctrl_state.tbl_spd_rpm_size() == 30 &&
     ctrl_state.tbl_nspd_nrpm_size() == 30){
    for(int i = 0; i< 30; i++){
      tbl_spd_rpm[i] = ctrl_state.tbl_spd_rpm(i);
      tbl_nspd_nrpm[i] = ctrl_state.tbl_nspd_nrpm(i);
    }
    monotonize_tbl_stable_rpm();
    monotonize_tbl_stable_nrpm();
  }
  
}

bool f_autopilot::is_stable(const float cog, const float sog,
			    const float yaw, const float rev)
{   
  if(abs(normalize_angle_rad(yaw-yaw_stbl)) < devyaw &&
     abs(normalize_angle_rad(cog-cog_stbl)) < devcog &&
     abs(rev-rev_stbl) < devrev &&
     abs(sog-sog_stbl) < devsog){
    if(tbegin_stable < 0){
      tbegin_stable = get_time();      
    }else
      return (get_time() - tbegin_stable) > twindow_stability_check;
  }else{
    yaw_stbl = yaw;
    cog_stbl = cog;
    rev_stbl = rev;
    sog_stbl = sog;
    tbegin_stable = -1;
  }
  return false;
}

void f_autopilot::estimate_stat(const long long tvel, const float cog,
				const float sog,
				const long long tatt, const float roll,
				const float pitch, const float yaw,
				const long long trev, const float rev,
				const s_aws1_ctrl_stat & stat)
{
  unsigned short eng, rud;
  eng = stat.eng_aws;
  rud = stat.rud_aws;

  if(eng <= stat.eng_nuf && eng >= stat.eng_nub)
    rev_prop = 0;
  else if(eng < stat.eng_nub){
    rev_prop = -rev;
  }else{
    rev_prop = rev;
  }
   
  angle_drift = (float) normalize_angle_rad(cog - (yaw + yaw_bias));  
  u = sog * cos(angle_drift);
  v = sog * sin(angle_drift);
  
  angle_flw = (float) normalize_angle_rad(crs_flw - (yaw + yaw_bias)); 
  uflw = spd_flw * cos(angle_flw);
  vflw = spd_flw * sin(angle_flw);  

  ucor = u - uflw;
  vcor = v - vflw;
  angle_drift_cor = atan2(vcor, ucor);
  cog_cor = angle_drift_cor + (yaw + yaw_bias);
  sog_cor = sqrt(ucor * ucor + vcor * vcor);
  
  if(m_verb){
    cout << "u,v="
	 << u << "," << v << endl;
    cout << "uflw,vflw="
	 << uflw << "," << vflw << endl;
    cout << "ucor,vcor="
	 << ucor << "," << vcor << endl;
  }
  
  if(eng_prev != eng){
    deng = (int)eng - (int)eng_prev;
  }else
    deng = 0;
  eng_prev = eng;

  if(rud_prev != rud){
    drud = (int)rud - (int)rud_prev;
    if(drud < 0)
      is_rud_ltor = false;
    else
      is_rud_ltor = true;
  }else
    drud = 0;
  
  rud_prev = rud;  
  
  // Assumption: 
  // v=vboat+vflow
  // dcog=0 (straight motion) -> byaw + yaw = cog 
  // calculate parameters below
  // dcog: cog rate deg/sec
  // byaw: yaw bias deg (average cog-yaw, where  bias+yaw=cog)
  // drev: rev rate rpm/sec
  // rudmidlr, rudmidrl: midship instruction value (left to right, right to left)
  // local_flow_dir, local_flow_spd: local flow (estimated during engine cutoff)
  // tbl_stable_rpm[60]: enging control table (rpm vs instruction value)
  
  if(tvel > tcog_prev)
    dcog = (double) SEC * normalize_angle_rad(cog - cog_prev)
      / (double) (tvel - tcog_prev);

  if(tvel > tsog_prev)
    dsog = (double) SEC * (sog - sog_prev)
      / (double) (tvel - tsog_prev);

  if(trev > trev_prop_prev)
    drev = (double) SEC * (rev_prop - rev_prop_prev)
      / (double)(trev - trev_prop_prev);

  cog_prev = cog;
  tcog_prev = tvel;
  tsog_prev = tvel;
  roll_prev = roll;
  pitch_prev = pitch;
  yaw_prev = yaw;
  tatt_prev = tatt;
  rev_prop_prev = rev_prop;
  trev_prop_prev = trev;

  if (is_stable(cog, sog, yaw, rev_prop)){
    if(rev_prop != 0){
      // updating stable rev-eng table.
      int irev =  (int)(rev_prop * 0.01 + 0.5);
      irev = min(irev, 59);
      irev = max(irev, -59);
      float ialpha = (float)(1.0 - alpha_tbl_stable_rpm);
      if(irev >= 0){
	tbl_stable_rpm[irev] = (float)(tbl_stable_rpm[irev] * ialpha
				       +alpha_tbl_stable_rpm * m_eng);
	monotonize_tbl_stable_rpm(irev);
	if(m_verb)
	  cout << "eng_rpm[" << irev << "]<-"
	       << tbl_stable_rpm[irev] << endl;
      }else{
	tbl_stable_nrpm[-irev] = (float)(float)(tbl_stable_nrpm[irev] * ialpha
						+alpha_tbl_stable_rpm * m_eng);
	monotonize_tbl_stable_nrpm(-irev);
	if(m_verb)
	  cout << "eng_nrpm[" << irev << "]<-"
	       << tbl_stable_nrpm[-irev] << endl;
      }

      // updating sog-rev table
      int isog = (int)(sog_cor+0.5);
      if(ucor > 0){ // ahead
	tbl_spd_rpm[isog] = tbl_spd_rpm[isog] * ialpha
	  + alpha_tbl_stable_rpm * irev;
	monotonize_tbl_spd_rpm(isog);
	if(m_verb)
	  cout << "spd_rev[" << isog << "]<-" << tbl_spd_rpm[isog] << endl;
      }else if(ucor < 0){ // astern
	tbl_nspd_nrpm[isog] = tbl_nspd_nrpm[isog] * ialpha
	  + alpha_tbl_stable_rpm * irev;
	monotonize_tbl_nspd_nrpm(isog);
	if(m_verb)
	  cout << "nspd_nrev[" << isog << "]<-" << tbl_spd_rpm[isog] << endl;
      }
    
      ialpha = (float)(1.0 - alpha_rud_mid);
      if (is_rud_ltor){
	rudmidlr = (float)(rudmidlr * ialpha + alpha_rud_mid * m_rud);      
	if(m_verb)
	  cout << "rudmidlr<-" << rudmidlr << endl;
      }else{
	rudmidrl = (float)(rudmidrl * ialpha + alpha_rud_mid * m_rud);
	if(m_verb)
	  cout << "rudmidrl<-" << rudmidrl << endl;
      }
      // in the stable motion, drift angle is assumed as the yaw bias.
      ialpha = (float)(1.0 - alpha_yaw_bias);
      yaw_bias = yaw_bias * ialpha + alpha_yaw_bias * angle_drift;      

    }else{
      // update flow velocity if not propelling. otherwise, update yaw_bias.
      
      float ialpha = (float)(1.0 - alpha_flw);
      crs_flw = crs_flw * ialpha + alpha_flw * cog;
      spd_flw = spd_flw * ialpha + alpha_flw * sog;
      if(m_verb)
	cout << "local flow updated to C" << crs_flw
	     << ",S" << spd_flw << endl;
    }
  }
}

bool f_autopilot::proc()
{
  float cog, sog, rpm, roll, pitch, yaw;  
  s_aws1_ctrl_stat stat;
  double Rorg[9];
  unsigned char trim;
  long long t = 0;
  long long teng = 0;
  long long tvel = 0;
  long long tatt = 0;
  m_state->get_corrected_velocity(tvel, cog, sog);
  m_state->get_enu_rotation(t, Rorg);
  m_engstate->get_rapid(teng, rpm, trim);
  m_state->get_attitude(tatt, roll, pitch, yaw);
  m_ctrl_stat->get(stat);
  roll *= PI/180.f;
  pitch *= PI/180.f;
  yaw *= PI/180.f;
  cog *= PI/180.f;
  
  estimate_stat(tvel, cog, sog, tatt, roll, pitch, yaw, teng, rpm, stat);

  if(stat.ctrl_src == ControlSource_AP){	
    if (!m_ap_inst){
      wp(sog, cog, yaw);
    }
    else{
      AutopilotMode mode = m_ap_inst->get_mode();
      switch (mode){
      case AutopilotMode_STB_MAN: // stabilized manual mode
	stb_man(cog, rev_prop_prev);
	break;	  
      case AutopilotMode_CURSOR:
	cursor(sog, cog, yaw, false);
	break;
      case AutopilotMode_FLW_TGT:
	flw_tgt(sog_cor, cog, yaw, false);
	break;
      case AutopilotMode_STAY:
	stay(sog_cor, cog, yaw);
	break;
      case AutopilotMode_WP:
	wp(sog_cor, cog, yaw, false);
	break;
      case AutopilotMode_WPAV:
	wp(sog_cor, cog, yaw, true);
	break;
      }
    }
  }else{
    m_rud = stat.rud_aws;
    m_eng = stat.eng_aws;
    m_icdiff = m_isdiff = m_irevdiff = 0.;
  }
  
  if(m_verb){
    cout << "(eng, rud)=(" << m_eng << "," << m_rud << ")" << endl;
  }
  
  m_inst.tcur = get_time();
  m_inst.eng_aws = (unsigned char) min(max(m_eng, m_eng_min), m_eng_max);
  m_inst.rud_aws = (unsigned char) min(max(m_rud, 0.f), 255.f);
  m_ctrl_inst->set(m_inst);
  
  return true;
}


const float f_autopilot::calc_course_change_for_ais_ship(const float crs)
{
  float cc = 0.; // course change
  
  if (m_ais_obj){
    float thcrs = (float)(crs);
    float iwo2 = (float)(1.0 / (m_Wo * m_Wo));
    float ilo2 = (float)(1.0 / (m_Lo * m_Lo));
    
    m_ais_obj->lock();
    for (m_ais_obj->begin(); !m_ais_obj->is_end(); m_ais_obj->next())
      {
	double x, y, z;
	float vx, vy, vz, yw;
	float bear, dist, tcpa, dcpa;
	if (!m_ais_obj->get_cur_state(x, y, z, vx, vy, vz, yw) ||
	    !m_ais_obj->get_tdcpa(tcpa, dcpa) ||
	    !m_ais_obj->get_pos_bd(bear, dist))
	  continue;
	
	if (tcpa < 0 || tcpa > m_Tav){
	  continue;
	}
	
	float bro = (float)(bear - thcrs);
	float cro = (float)cos(bro);
	float sro = (float)sin(bro);
	
	float thy = (float)(yw * (PI / 180.0f));
	float br = (float)(-bear - thy);
	float cr = (float)cos(br);
	float sr = (float)sin(br);
	
	float iwais2 = (float)(1.0 / (m_Wais * m_Wais));
	float ilais2 = (float)(1.0 / (m_Lais * m_Lais));
	
	float ro2 = (float)(1.0 / (iwo2 * cro * cro + ilo2 * sro * sro));
	float r2 = (float)(1.0 / (iwais2 * cr * ilais2 * sr));
	float ro = (float)sqrt(ro2);
	float r = (float)sqrt(r2);
	float rcol = (float)(r + ro);
	float rav = (float)(rcol * m_Rav);
	
	if (dcpa < rav){ // calculate avoidance
	  cc += (float)((1.0 - (dcpa / rav)) * m_Cav_max);
	}
      }
    m_ais_obj->unlock();
  }
  
  cc = min(m_Cav_max, cc);
  
  return cc;
}

void f_autopilot::ctrl_to_cog(const float cdiff)
{
  // If flow corrected sog is less than 1kts, don't control any more to avoid rudder chattering.
  if(sog_cor < 1) 
    return;

  float rudmid = (is_rud_ltor ? rudmidlr : rudmidrl);
  float _cdiff = cdiff;
  if(rev_prop < 0)
    _cdiff = -_cdiff;
  
  // cdiff is normalized to [-PI,PI]
  _cdiff = normalize_angle_rad(_cdiff);
  _cdiff *= (float)(1.0f/PI); // normalize PI rad to 1
  m_dcdiff = (float)(_cdiff - m_cdiff);
  if((_cdiff < 0 && m_rud > 0.f) || (_cdiff > 0 && m_rud < 255.f))
    m_icdiff += _cdiff;
  
  m_cdiff = _cdiff;

  m_rud = (float)((m_pc * m_cdiff + m_ic * m_icdiff + m_dc * m_dcdiff) * 255.);
  
  m_rud += rudmid;  
  m_rud = max(m_rud, 0.f);
  m_rud = min(m_rud, 255.f);
  if (m_verb)
    printf("ap rud=%3.1f c=%2.2f dc=%2.2f ic=%2.2f\n", m_rud, m_cdiff, m_dcdiff, m_icdiff);

}


void f_autopilot::ctrl_to_rev(const float rev, const float rev_tgt,
			      const float rev_max, const float rev_min)
{
  float _rev_tgt;
  if(abs(rev_tgt) < rev_min){
    _rev_tgt = 0;
    m_eng = 127.0f;
    return;
  }else
    _rev_tgt = rev_tgt;
  
  int irev = (int)(_rev_tgt * 0.01 + 0.5);
  if(irev < 0){ // for negative _rev_tgt
    m_eng = tbl_stable_nrpm[-irev];
    float revdiff = max(min(rev_max, -_rev_tgt), rev_min) - rev;
    revdiff *= (float)(-1. / (rev_max - rev_min));
    m_drevdiff = (float)(revdiff - m_revdiff);
    float irevdiff = m_irevdiff + revdiff;
    m_revdiff = revdiff;
    float delta_eng = (float)((m_prev * m_revdiff + m_irev * irevdiff
			  + m_drev * m_drevdiff) * 255.);
    if(delta_eng > 0 || m_eng_min != m_eng){
      m_eng += delta_eng;
      m_irevdiff = irevdiff;
    }
    m_eng = min(127.0f, m_eng);
    m_eng = max(m_eng_min, m_eng);
  }else {
    m_eng = tbl_stable_rpm[irev];
    float revdiff = max(min(rev_max, _rev_tgt), rev_min) - rev;  
    revdiff *= (float)(1. / (rev_max - rev_min));
    m_drevdiff = (float)(revdiff - m_revdiff);
    float irevdiff = m_irevdiff + revdiff;
    m_revdiff = revdiff;
    float delta_eng = (float)((m_prev * m_revdiff + m_irev * irevdiff + m_drev * m_drevdiff) * 255.);
    if(delta_eng < 0 || m_eng_max != m_eng){
      m_eng += delta_eng;
      m_irevdiff = irevdiff;
    }
    m_eng = max(127.0f, m_eng);
    m_eng = min(m_eng_max, m_eng);
  }
  if(m_verb){
    cout << "rev ctrl (p,i,d)=" << m_revdiff << "," << m_irevdiff << "," << m_drevdiff << " vtbl[" << irev << "]=" << (irev < 0 ? tbl_stable_nrpm[-irev] : tbl_stable_rpm[irev]) << endl;
  }
}

void f_autopilot::ctrl_to_sog(const float sog, const float sog_tgt,
			      const float smax, const float smin)
{
  if (abs(sog_tgt) < smin){
    m_eng = 127.f;
    return;
  }
  
  float rudmid = (is_rud_ltor ? rudmidlr : rudmidrl);
  float srange = (float)(smax - smin);
  
  float stgt = (float)(srange * (1.0 - max(0.f, min(1.f, abs(m_rud - rudmid) * (1.0f / rudmid)))) + smin);
  stgt = min(sog_tgt, stgt);
  
  float sdiff = (float)(stgt - sog);
  sdiff *= (float)(1. / smax);
  
  m_dsdiff = (float)(sdiff - m_sdiff);
  m_isdiff += sdiff;
  m_sdiff = sdiff;
 
  m_eng = tbl_stable_rpm[(int)(tbl_spd_rpm[(int)(stgt + 0.5)]+0.5)];
  m_eng += (float)((m_ps * m_sdiff + m_is * m_isdiff + m_ds * m_dsdiff) * 255.);
  m_eng = (float)min(m_eng, m_eng_max);
  m_eng = (float)max(m_eng, 127.f);  
  if(m_verb){
    printf("ap tbl[%d]=%2.1f eng=%3.1f stgt=%2.1f sog=%2.1f s=%2.2f ds=%2.2f is=%2.2f \n", (int)stgt, tbl_spd_rpm[(int)(stgt+0.5)], m_eng, stgt, sog, 
	   m_sdiff, m_dsdiff, m_isdiff);
  }
}

void f_autopilot::ctrl_to_sog_cog(const float sog, const float sog_tgt,
				  const float cdiff,
				  const float smax, const float smin)
{
  ctrl_to_cog(cdiff);
  ctrl_to_sog(sog, sog_tgt, smax, smin);  
}

void f_autopilot::wp(const float sog, const float cog, const float yaw, bool bav)
{
  float cc = 0;
  float sog_tgt = 0.0;
  m_ap_inst->get_tgt_sog(sog_tgt);
  
  if (bav)
    cc = calc_course_change_for_ais_ship(yaw);
  
  m_wp->lock();
  if (m_wp->is_finished()){
    m_rud = 127.;
    m_eng = 127.;
    m_icdiff = m_isdiff = 0.;
  }
  else{
    s_wp & wp = m_wp->get_next_wp();
    float d = 0.;
    float cdiff = 0;
    float xdiff = 0;
    if(wp.v > 0)
      sog_tgt = min(sog_tgt, wp.v);
    
    m_wp->get_diff(d, cdiff, xdiff);
    cdiff += cc;
   
    ctrl_to_sog_cog(sog, sog_tgt, (float)(cdiff * PI / 180.0f), m_smax, m_smin);
  }
  
  m_wp->unlock();
}

void f_autopilot::cursor(const float sog, const float cog, const float yaw, bool bav)
{
  float xr, yr, d, dir;
  m_ap_inst->get_csr_pos_rel(xr, yr, d, dir);
  float cdiff = (float)(dir - cog);
  float sog_tgt = 0.0;
  m_ap_inst->get_tgt_sog(sog_tgt);
  
  ctrl_to_sog_cog(sog, sog_tgt, cdiff, m_smax, m_smin);
}

void f_autopilot::flw_tgt(const float sog, const float cog, const float yaw, bool bav)
{
  float xr, yr, d, dir;
  m_ap_inst->get_tgt_pos_rel(xr, yr, d, dir);

  float cdiff = (float)normalize_angle_deg(dir - cog);
  
  float sog_tgt = 0.0;
  m_ap_inst->get_tgt_sog(sog_tgt);
  
  ctrl_to_sog_cog(sog, sog_tgt, cdiff, m_smax, m_smin);
}

void f_autopilot::stay(const float sog, const float cog, const float yaw)
{
  { // updating relative position of the stay point.
    long long t;
    double Rorg[9];
    double xorg, yorg, zorg;
    m_state->get_enu_rotation(t, Rorg);
    m_state->get_position_ecef(t, xorg, yorg, zorg);
    m_ap_inst->update_pos_rel(Rorg, xorg, yorg, zorg);
  }

  float rx, ry, d, dir;
  m_ap_inst->get_stay_pos_rel(rx, ry, d, dir);
  float cdiff = (float)(dir - cog);
  if(m_verb){
    printf("ap stay d=%02.1f\n", d);
  }
  float sog_tgt = 0.0;
  m_ap_inst->get_tgt_sog(sog_tgt);

  if(d > 10.0)
    ctrl_to_sog_cog(sog, sog_tgt, cdiff, 2.0f, 1.0f);  
  else
    m_eng = 127.f;
}


void f_autopilot::stb_man(const float cog, const float rev)
{
  float cog_tgt, rev_tgt;
  m_ap_inst->get_tgt_cog_and_rev(cog_tgt, rev_tgt);
  ctrl_to_cog((float)(cog_tgt * PI / 180.0 - cog));
  ctrl_to_rev(rev, rev_tgt, m_rev_max, m_rev_min);  
}
