#ifndef GUSDELTASOLUTION_H
#define GUSDELTASOLUTION_H
#include "libs/Module.h"
#include "BaseSolution.h"

class Config;

class GUSDeltaSolution : public BaseSolution {
    public:
        GUSDeltaSolution(Config*);
        void cartesian_to_actuator( const float millimeters[], ActuatorCoordinates &steps ) override;
        void actuator_to_cartesian( const ActuatorCoordinates &steps, float millimeters[] ) override;
    private:
        float base_length;
        float vertical_offset;
};

#endif // GUSDELTASOLUTION_H
