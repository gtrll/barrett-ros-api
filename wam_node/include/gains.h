#include <string>

#include <barrett/os.h>
#include <barrett/units.h>
#include <barrett/math.h>
#include <barrett/detail/stl_utils.h>

using namespace barrett;

template<size_t DOF>  
void printGains(systems::Wam<DOF>& wam) {
    {
    std::stringstream kp;
    kp << wam.jvController1.getKp();
    std::stringstream kd;
    kd << wam.jvController1.getKd();
    std::stringstream ki;
    ki << wam.jvController1.getKi();
    std::stringstream i_lim;
    i_lim << wam.jvController1.getIntegratorLimit();
    std::stringstream u_lim;
    u_lim << wam.jvController1.getControlSignalLimit();

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
    kp << wam.jpController.getKp();
    std::stringstream kd;
    kd << wam.jpController.getKd();
    std::stringstream ki;
    ki << wam.jpController.getKi();
    std::stringstream i_lim;
    i_lim << wam.jpController.getIntegratorLimit();
    std::stringstream u_lim;
    u_lim << wam.jpController.getControlSignalLimit();

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

template<size_t DOF>  
void setJpGains(systems::Wam<DOF>& wam, double Kp[DOF], double Kd[DOF], double Ki[DOF]) {

  BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

  v_type v_kp, v_kd, v_ki;

  for (size_t j=0; j<DOF; j++) {
    v_kp[j]=Kp[j];
    v_kd[j]=Kd[j];
    v_ki[j]=Ki[j];
  }  

  wam.jpController.setKp(v_kp);
  wam.jpController.setKd(v_kd);
  wam.jpController.setKi(v_ki);

}