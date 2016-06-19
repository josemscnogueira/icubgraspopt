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
    typedef boost::shared_ptr<TGPOptimizable> TGPOptimizablePtr;

    namespace TGPOptimizableUtils
    {
        TGPOptimizablePtr  getOptimizable(Json::Value config);
        TGPOptimizablePtr  getOptimizable(std::string name  );
        TGPOptimizablePtr  getOptimizable(const char* name);

    } // End of namespace TGPOptimizableUtils
} // End of namespace bayesopt


#endif // __OPTIMIZABLESMANAGER_HPP__
