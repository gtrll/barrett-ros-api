#pragma once

#include <barrett/os.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>

namespace barrett {
namespace systems {

// System class providing realtime for xenomai timestamp
class Time : public System,
             public SingleInput<double>,
             public SingleOutput<double> {
    
    public:

        double xenomai_timestamp;

        Time(const std::string& sys_name = "Time")
            : System(sys_name),
              SingleInput<double>(this),
              SingleOutput<double>(this) {
              xenomai_timestamp = 0;
        }

        double getTime() {
            return *this->outputValue->getData();
        }

    protected:
        virtual void operate() {
            xenomai_timestamp = barrett::highResolutionSystemTime();
            this->outputValue->setData(&xenomai_timestamp);
        }
    };


// System class providing realtime delta for xenomai timestamp
class dTime : public System,
             public SingleInput<double>,
             public SingleOutput<double> {
    
    public:

        double d_xenomai_timestamp;
        double xenomai_time_start;

        dTime(const std::string& sys_name = "dTime")
            : System(sys_name),
              SingleInput<double>(this),
              SingleOutput<double>(this) {
              d_xenomai_timestamp = 0;
              xenomai_time_start = barrett::highResolutionSystemTime();
        }

        double getdTime() {
            return *this->outputValue->getData();
        }

    protected:
        virtual void operate() {
            d_xenomai_timestamp = barrett::highResolutionSystemTime()
            					  - xenomai_time_start;
            this->outputValue->setData(&d_xenomai_timestamp);
        }
    };

}
}