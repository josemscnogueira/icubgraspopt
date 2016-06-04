/**************************************************************************************************
 *  File:    iCubOptParameters.hpp                                                                *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/
#ifndef __ICUBOPTPARAMATERS_HPP__
#define __ICUBOPTPARAMATERS_HPP__

#include <string>
#include <vector>

#include <json/json.h>
#include <specialtypes.hpp>


/**************************************************************************************************
 *  Enumerations                                                                                  *
 **************************************************************************************************/
enum
{
    TRANS_X   = 0,
    TRANS_Y   = 1,
    TRANS_Z   = 2,
    ROT_ROLL  = 3,
    ROT_PITCH = 4,
    ROT_YAW   = 5,
    FIRST_SIN = 6,
};

namespace bayesopt
{

class iCubOptParameters
{
public:
    std::string       object;
    uint              grasp;
    uint              position;

    uint              n_grasp_trials;
    double            trial_stddev;
    double            grasp_threshold;

    vectord           default_query;
    std::vector<uint> active_variables;

    // Constructors
    iCubOptParameters(void);
    iCubOptParameters(Json::Value config);

    // Methods
    Json::Value getJson (void);
    void        loadJson(Json::Value config);

    // Static Methods
    static std::string convertGraspToString           (uint        grasp);
    static uint        convertStringToGrasp           (std::string grasp);
    static std::string convertPositionToString        (uint        position);
    static uint        convertStringToPosition        (std::string position);
    static std::string convertActiveComponentToString (uint        active_component);
    static uint        convertStringToActiveComponent (std::string active_component);

    static std::string       convertActiveComponentsToString(std::vector<uint> active_components);
    static std::vector<uint> convertStringToActiveComponents(std::string       active_components);

private:
    static const char string_seperator = ',';
};

} // End of namespace bayesopt


#endif // __ICUBOPTPARAMATERS_HPP__
