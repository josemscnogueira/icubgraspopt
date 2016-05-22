/**************************************************************************************************
 *  File:    OptimizablesManager.h                                                                *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/
#ifndef __OPTIMIZABLESMANAGER_HPP__
#define __OPTIMIZABLESMANAGER_HPP__

#include <boost/shared_ptr.hpp>
#include <string>

#include <json/json.h>
#include <tgpoptimizable.hpp>


namespace bayesopt
{

/**************************************************************************************************
 *  Class: OptimizablesManager                                                                    *
 **************************************************************************************************/
class OptimizablesManager
{
public:
    typedef boost::shared_ptr<TGPOptimizable > TGPOptimizablePtr;

    // Constructor
    OptimizablesManager(void) { };

    // Static methods
    static TGPOptimizablePtr  getOptimizable    (Json::Value config);
    static TGPOptimizablePtr  getOptimizable    (std::string name  );
};

} // End of namespace bayesopt


#endif // __OPTIMIZABLESMANAGER_HPP__
