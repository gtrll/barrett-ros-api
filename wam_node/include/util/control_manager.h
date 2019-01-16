#pragma once

#include <string>

#include <barrett/os.h>
#include <barrett/units.h>
#include <barrett/math.h>
#include <barrett/detail/stl_utils.h>

#include "wam_config.h"

using namespace barrett;

template<size_t DOF>  
class ControlManager {

  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
  
  public:

    // Default constructor
    ControlManager();

    ControlManager(systems::Wam<DOF>& wam, ros::NodeHandle nh, WamConfig& wam_c) ;

    void printGains() ;
    void setGains() ;

  protected:

    void setJpGains() ;
    void setJvGains() ;

    systems::Wam<DOF>& wam_;
    ros::NodeHandle nh_;
    WamConfig& wam_c_;

    struct Gains {
      struct Pos {
        v_type Kp, Kd, Ki, iLim, uLim;
      };
      struct Vel {
        v_type Kp, Kd, Ki, iLim, uLim;
      };
      Pos position;
      Vel velocity;
    }  Ks_;
};


template<size_t DOF>
ControlManager<DOF>::ControlManager() {};

template<size_t DOF>
ControlManager<DOF>::ControlManager(systems::Wam<DOF>& wam, ros::NodeHandle nh, WamConfig& wam_c) :
                                        wam_(wam), nh_(nh), wam_c_(wam_c) {};

template<size_t DOF>
void ControlManager<DOF>::setGains() {

  if (!wam_c_.use_default_JpGains) 
  { 
    setJpGains();
  }
  if (!wam_c_.use_default_JvGains)  
  { 
    setJvGains();
  }
}

template<size_t DOF>
void ControlManager<DOF>::setJpGains() {

  for (size_t j=0; j<DOF; j++) {
    Ks_.position.Kp[j]=wam_c_.Jp_Kp[j];
    Ks_.position.Kd[j]=wam_c_.Jp_Ki[j];
    Ks_.position.Ki[j]=wam_c_.Jp_Kp[j];
    Ks_.position.iLim[j]=wam_c_.Jp_iLim[j];
    Ks_.position.uLim[j]=wam_c_.Jp_uLim[j];
  }  

  wam_.jpController.setKp(Ks_.position.Kp);
  wam_.jpController.setKd(Ks_.position.Kd);
  wam_.jpController.setKi(Ks_.position.Ki);
  wam_.jpController.setIntegratorLimit(Ks_.position.iLim);
  wam_.jpController.setControlSignalLimit(Ks_.position.uLim);

}

template<size_t DOF>
void ControlManager<DOF>::setJvGains() {
 
  for (size_t j=0; j<DOF; j++) {
    Ks_.velocity.Kp[j]=wam_c_.Jv_Kp[j];
    Ks_.velocity.Kd[j]=wam_c_.Jv_Ki[j];
    Ks_.velocity.Ki[j]=wam_c_.Jv_Kp[j];
    Ks_.velocity.iLim[j]=wam_c_.Jv_iLim[j];
    Ks_.velocity.uLim[j]=wam_c_.Jv_uLim[j];
  }  

  wam_.jpController.setKp(Ks_.velocity.Kp);
  wam_.jpController.setKd(Ks_.velocity.Kd);
  wam_.jpController.setKi(Ks_.velocity.Ki);
  wam_.jpController.setIntegratorLimit(Ks_.velocity.iLim);
  wam_.jpController.setControlSignalLimit(Ks_.velocity.uLim);

}

template<size_t DOF>
void ControlManager<DOF>::printGains() {
    
    {
    std::stringstream kp;
    kp << wam_.jvController1.getKp();
    std::stringstream kd;
    kd << wam_.jvController1.getKd();
    std::stringstream ki;
    ki << wam_.jvController1.getKi();
    std::stringstream i_lim;
    i_lim << wam_.jvController1.getIntegratorLimit();
    std::stringstream u_lim;
    u_lim << wam_.jvController1.getControlSignalLimit();

    printf("velocity controller gains:\n"
           "    kp: %s \n"
           "    kd: %s \n"
           "    ki: %s \n"
           "    i_lim: %s \n"
           "    u_lim: %s \n",
           kp.str().c_str(),
           kd.str().c_str(),
           ki.str().c_str(),
           i_lim.str().c_str(),
           u_lim.str().c_str()
        );
    
    }

    {
    std::stringstream kp;
    kp << wam_.jpController.getKp();
    std::stringstream kd;
    kd << wam_.jpController.getKd();
    std::stringstream ki;
    ki << wam_.jpController.getKi();
    std::stringstream i_lim;
    i_lim << wam_.jpController.getIntegratorLimit();
    std::stringstream u_lim;
    u_lim << wam_.jpController.getControlSignalLimit();

    printf("position controller gains:\n"
           "    kp: %s \n"
           "    kd: %s \n"
           "    ki: %s \n"
           "    i_lim: %s \n"
           "    u_lim: %s \n",
           kp.str().c_str(),
           kd.str().c_str(),
           ki.str().c_str(),
           i_lim.str().c_str(),
           u_lim.str().c_str()
        );
    }
}
