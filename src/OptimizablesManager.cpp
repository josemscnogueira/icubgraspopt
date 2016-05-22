/**************************************************************************************************
 *  File:    OptimizablesManager.h                                                                *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/

/**
 * Includes
 */
#include <OptimizablesManager.hpp>
#include <RKHS.hpp>

#include <OptFuncs.hpp>


namespace bayesopt
{
/**
 * Retrieves a pointer for TGPOptimizable from a Json::Value
 * Allocs a pointer
 *
 * @param  config Json config
 * @return        TGPOptimizable pointer
 *                NULL pointer if name is invalid
 */
OptimizablesManager::TGPOptimizablePtr OptimizablesManager::getOptimizable(Json::Value config)
{
    if (config["name"].isNull() != true)
    {
        return getOptimizable(config["name"].asString());
    }
    else
    {
        std::cout << std::endl << "Error: OptimizablesManager::getOptimizable Json Value is incorrect";

        return TGPOptimizablePtr();
    }
}


/**
 * Retrieves a pointer for TGPOptimizable from a Json::Value
 * Allocs a pointer
 *
 * @param  config Json config
 * @return        TGPOptimizable pointer
 *                NULL pointer if name is invalid
 */
OptimizablesManager::TGPOptimizablePtr OptimizablesManager::getOptimizable(std::string name)
{
    if ( (name.compare("RKHS") == 0) || (name.compare("RKHS_1D") == 0) )
    {
        return TGPOptimizablePtr(new RKHS());
    }
    else if ((name.compare("GramacyExponential") == 0) || (name.compare("GramacyExponential_2D") == 0))
    {
        return TGPOptimizablePtr(new GramacyExponential());
    }
    else if (name.compare("Martinez") == 0)
    {
        return TGPOptimizablePtr(new Martinez());
    }
    else
    {
        std::cout << std::endl << "Error: OptimizablesManager::getOptimizable couldn't load" << name;
    }

    // else everything
    return TGPOptimizablePtr();
}

} // End of namespace bayesopt
