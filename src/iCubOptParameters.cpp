/**************************************************************************************************
 *  File:    iCubOptParameters.cpp                                                                *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/

/**
 * Includes
 */
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

#include <ublas_string.hpp>

#include <iCubHand.hpp>
#include <ApproachMovementSpace.hpp>

#include <iCubOptParameters.hpp>

/**
 * Namespace: bayesopt
 */
namespace bayesopt
{


/**
 * Constructor
 */
iCubOptParameters::iCubOptParameters(void)
{
    grasp           = 0;
    position        = 0;
    n_grasp_trials  = 1;
    trial_stddev    = 0.002;
    grasp_threshold = 0.0001;
};

/**
 * Constructor
 */
iCubOptParameters::iCubOptParameters(Json::Value config)
{
    loadJson(config);
};


/**
 * Generates metadata in a Json structure
 *
 * @return  Json metadata
 */
Json::Value iCubOptParameters::getJson(void)
{
    Json::Value output;
                output["object"          ] = Json::Value(object);
                output["grasp"           ] = Json::Value(convertGraspToString(grasp));
                output["position"        ] = Json::Value(convertPositionToString(position));
                output["n_grasp_trials"  ] = Json::Value(n_grasp_trials);
                output["trial_stddev"    ] = Json::Value(trial_stddev);
                output["grasp_threshold" ] = Json::Value(grasp_threshold);
                output["default_query"   ] = Json::Value(bayesopt::utils::ublas_toString(default_query));
                output["active_variables"] = Json::Value(convertActiveComponentsToString(active_variables));
    return output;
};


/**
 * Load JSON metadata
 *
 * @param config metada
 */
void iCubOptParameters::loadJson(Json::Value config)
{
    if (config["object"].isNull() != true)
        object = config["object"].asString();
    if (config["grasp"].isNull() != true)
        grasp = convertStringToGrasp(config["grasp"].asString());
    if (config["position"].isNull() != true)
        position = convertStringToPosition(config["position"].asString());
    if (config["n_grasp_trials"].isNull() != true)
        n_grasp_trials = config["n_grasp_trials"].asUInt64();
    if (config["trial_stddev"].isNull() != true)
        trial_stddev = config["trial_stddev"].asDouble();
    if (config["grasp_threshold"].isNull() != true)
        grasp_threshold = config["grasp_threshold"].asDouble();
    if (config["default_query"].isNull() != true)
        default_query = bayesopt::utils::string_toUblasVectord(config["default_query"].asString());
    if (config["active_variables"].isNull() != true)
        active_variables = convertStringToActiveComponents(config["active_variables"].asString());
};


/**
 * Converts grasp from uint into string
 *
 * @param  grasp uint
 * @return       grasp - string
 */
std::string iCubOptParameters::convertGraspToString(uint grasp)
{
    switch(grasp)
    {
        case iCub::JOINTS_MODE       : return std::string("JOINTS_MODE"       );
        case iCub::TRYPOD            : return std::string("TRYPOD"            );
        case iCub::PALMAR_PINCH      : return std::string("PALMAR_PINCH"      );
        case iCub::LATERAL           : return std::string("LATERAL"           );
        case iCub::WRITING_TRIPOD    : return std::string("WRITING_TRIPOD"    );
        case iCub::PARALLEL_EXTENSION: return std::string("PARALLEL_EXTENSION");
        case iCub::ADDUCTION_GRIP    : return std::string("ADDUCTION_GRIP"    );
        case iCub::TIP_PINCH         : return std::string("TIP_PINCH"         );
        case iCub::LATERAL_TRIPOD    : return std::string("LATERAL_TRIPOD"    );
        case iCub::POWER_GRIP        : return std::string("POWER_GRIP"        );
        default                      : return std::string("UNKNOWN_GRASP"     );
    }
};

/**
 * Converts grasp from string to uint
 *
 * @param   grasp string
 * @return        uint
 */
uint iCubOptParameters::convertStringToGrasp(std::string grasp)
{
    if      (grasp.compare("JOINTS_MODE"       ) == 0) return iCub::JOINTS_MODE;
    else if (grasp.compare("TRYPOD"            ) == 0) return iCub::TRYPOD;
    else if (grasp.compare("PALMAR_PINCH"      ) == 0) return iCub::PALMAR_PINCH;
    else if (grasp.compare("LATERAL"           ) == 0) return iCub::LATERAL;
    else if (grasp.compare("WRITING_TRIPOD"    ) == 0) return iCub::WRITING_TRIPOD;
    else if (grasp.compare("PARALLEL_EXTENSION") == 0) return iCub::PARALLEL_EXTENSION;
    else if (grasp.compare("ADDUCTION_GRIP"    ) == 0) return iCub::ADDUCTION_GRIP;
    else if (grasp.compare("TIP_PINCH"         ) == 0) return iCub::TIP_PINCH;
    else if (grasp.compare("LATERAL_TRIPOD"    ) == 0) return iCub::LATERAL_TRIPOD;
    else if (grasp.compare("POWER_GRIP"        ) == 0) return iCub::POWER_GRIP;
    else                                               return (uint) -1;
};


/**
 * Converts position from uint to string
 *
 * @param  position uint
 * @return          string
 */
std::string iCubOptParameters::convertPositionToString(uint position)
{
    switch(position)
    {
        case GraspStudio::RIFR : { return std::string("RIFR"); }
        case GraspStudio::RIUP : { return std::string("RIUP"); }
        case GraspStudio::RIBA : { return std::string("RIBA"); }
        case GraspStudio::RIDO : { return std::string("RIDO"); }
        case GraspStudio::FRLE : { return std::string("FRLE"); }
        case GraspStudio::FRUP : { return std::string("FRUP"); }
        case GraspStudio::FRRI : { return std::string("FRRI"); }
        case GraspStudio::FRDO : { return std::string("FRDO"); }
        case GraspStudio::LEBA : { return std::string("LEBA"); }
        case GraspStudio::LEUP : { return std::string("LEUP"); }
        case GraspStudio::LEFR : { return std::string("LEFR"); }
        case GraspStudio::LEDO : { return std::string("LEDO"); }
        case GraspStudio::BARI : { return std::string("BARI"); }
        case GraspStudio::BAUP : { return std::string("BAUP"); }
        case GraspStudio::BALE : { return std::string("BALE"); }
        case GraspStudio::BADO : { return std::string("BADO"); }
        case GraspStudio::TOFR : { return std::string("TOFR"); }
        case GraspStudio::TOLE : { return std::string("TOLE"); }
        case GraspStudio::TOBA : { return std::string("TOBA"); }
        case GraspStudio::TORI : { return std::string("TORI"); }
        case GraspStudio::BOFR : { return std::string("BOFR"); }
        case GraspStudio::BORI : { return std::string("BORI"); }
        case GraspStudio::BOBA : { return std::string("BOBA"); }
        case GraspStudio::BOLE : { return std::string("BOLE"); }
        case GraspStudio::ESPH : { return std::string("ESPH"); }
    }
    return std::string("UNKNOWN_POSITION");
};


/**
 * Converts position from string into uint
 *
 * @param  position string
 * @return          uint
 */
uint iCubOptParameters::convertStringToPosition(std::string position)
{
    if      (position.compare("RIFR") == 0) return GraspStudio::RIFR;
    else if (position.compare("RIUP") == 0) return GraspStudio::RIUP;
    else if (position.compare("RIBA") == 0) return GraspStudio::RIBA;
    else if (position.compare("RIDO") == 0) return GraspStudio::RIDO;
    else if (position.compare("FRLE") == 0) return GraspStudio::FRLE;
    else if (position.compare("FRUP") == 0) return GraspStudio::FRUP;
    else if (position.compare("FRRI") == 0) return GraspStudio::FRRI;
    else if (position.compare("FRDO") == 0) return GraspStudio::FRDO;
    else if (position.compare("LEBA") == 0) return GraspStudio::LEBA;
    else if (position.compare("LEUP") == 0) return GraspStudio::LEUP;
    else if (position.compare("LEFR") == 0) return GraspStudio::LEFR;
    else if (position.compare("LEDO") == 0) return GraspStudio::LEDO;
    else if (position.compare("BARI") == 0) return GraspStudio::BARI;
    else if (position.compare("BAUP") == 0) return GraspStudio::BAUP;
    else if (position.compare("BALE") == 0) return GraspStudio::BALE;
    else if (position.compare("BADO") == 0) return GraspStudio::BADO;
    else if (position.compare("TOFR") == 0) return GraspStudio::TOFR;
    else if (position.compare("TOLE") == 0) return GraspStudio::TOLE;
    else if (position.compare("TOBA") == 0) return GraspStudio::TOBA;
    else if (position.compare("TORI") == 0) return GraspStudio::TORI;
    else if (position.compare("BOFR") == 0) return GraspStudio::BOFR;
    else if (position.compare("BORI") == 0) return GraspStudio::BORI;
    else if (position.compare("BOBA") == 0) return GraspStudio::BOBA;
    else if (position.compare("BOLE") == 0) return GraspStudio::BOLE;
    else if (position.compare("ESPH") == 0) return GraspStudio::ESPH;
    else                                    return (uint) -1;
};


/**
 * Converts active component from uint into string
 *
 * @param  active_component uint
 * @return                  string
 */
std::string iCubOptParameters::convertActiveComponentToString(uint active_component)
{
    switch(active_component)
    {
        case TRANS_X   : { return std::string("TRANS_X");   }
        case TRANS_Y   : { return std::string("TRANS_Y");   }
        case TRANS_Z   : { return std::string("TRANS_Z");   }
        case ROT_ROLL  : { return std::string("ROT_ROLL");  }
        case ROT_PITCH : { return std::string("ROT_PITCH"); }
        case ROT_YAW   : { return std::string("ROT_YAW");   }
        default        :
        {
            std::string result("SINGERY_");
                        result += boost::lexical_cast<std::string>(active_component - 5);
            return result;
        }
    }
};

/**
 * Converts active component from string into uint
 *
 * @param  active_component string
 * @return                  uint
 */
uint iCubOptParameters::convertStringToActiveComponent(std::string active_component)
{
    if      (active_component.compare("TRANS_X"  ) == 0) return TRANS_X;
    else if (active_component.compare("TRANS_Y"  ) == 0) return TRANS_Y;
    else if (active_component.compare("TRANS_Z"  ) == 0) return TRANS_Z;
    else if (active_component.compare("ROT_ROLL ") == 0) return ROT_ROLL ;
    else if (active_component.compare("ROT_PITCH") == 0) return ROT_PITCH;
    else if (active_component.compare("ROT_YAW  ") == 0) return ROT_YAW  ;
    else if (active_component.substr(0,8).compare("SINGERY_") == 0)
    {
        return (atoi(active_component.substr(8,1).c_str()) + 5);
    }
    return (uint) -1;
};


/**
 * Converts active componenents from std::vector into std::string
 *
 * @param  active_components (std::vector)
 * @return                   (std::string)
 */
std::string iCubOptParameters::convertActiveComponentsToString(std::vector<uint> active_components)
{
    std::string result = "";

    if (active_components.size() > 0)
    {
        for (uint index = 0; index < active_components.size(); index += 1)
        {
            // Push active component string
            result += convertActiveComponentToString(active_components[index]);

            // Except for the last element, introduce a seperator
            if(index != (active_components.size() - 1))
                result += string_seperator;
        }
    }

    return result;
}


/**
 * Converts active componenents from std::string into std::vector
 *
 * @param  active_components (std::string)
 * @return                   (std::vector)
 */
std::vector<uint> iCubOptParameters::convertStringToActiveComponents(std::string active_components)
{
    std::vector<uint>        result;
    std::vector<std::string> string_splits;
    std::string              delimiter;
                             delimiter += string_seperator;

    // Perform string split using the string seperator
    boost::split(string_splits, active_components, boost::is_any_of(delimiter));

    if (string_splits.size() > 0)
    {
        for (uint index = 0; index < string_splits.size(); index += 1)
        {
            uint read_component = convertStringToActiveComponent(string_splits[index]);

            // Push back read active component
            if (read_component != ((uint) -1))
                result.push_back(read_component);
        }
    }

    return result;
}

} // End of namespace bayesopt
