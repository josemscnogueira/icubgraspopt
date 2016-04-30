/**************************************************************************************************
 *  File:    iCubOptParameters.h                                                                  *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/
#ifndef __ICUBOPTPARAMATERS_H__
#define __ICUBOPTPARAMATERS_H__

#include <specialtypes.hpp>

typedef struct
{
    std::string       object;
    uint              grasp;
    uint              position;

    uint              n_grasp_trials;
    double            trial_stddev;
    double            grasp_threshold;

    vectord           default_query;
    std::vector<uint> active_variables;
}
iCubOptParameters;


#endif // __ICUBOPTPARAMATERS_H__
