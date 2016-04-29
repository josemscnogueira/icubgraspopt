/**************************************************************************************************
 *  File:    RKHS.h                                                                               *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/
#ifndef __RKHS_H__
#define __RKHS_H__


/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <string>           // Global
#include <chrono>
#include <stdlib.h>
#include <Python.h>
#include <boost/random.hpp>

#include <specialtypes.hpp> // Bayesopt
#include <tgpoptimizable.hpp>


/**************************************************************************************************
 *  Typedefs                                                                                      *
 **************************************************************************************************/
typedef boost::mt19937                                            RandomEngine;
typedef boost::uniform_real<>                                     RealDistribution;
typedef boost::variate_generator<RandomEngine&, RealDistribution> Generator;


/**************************************************************************************************
 *  Class: RKHS                                                                                   *
 **************************************************************************************************/
class RKHS : public TGPOptimizable
{
private:
    PyObject*      _module;
    PyObject*      _function;

    RandomEngine   _randEngine;

public:
    // Constructor
    RKHS(void);

    // Destructor
    ~RKHS(void);

    // Methods
    double evaluate      (double          x);
    double evaluate      (vectord         x);
    void   initSamples   (vecOfvec&       xx        , vectord&     yy);
    void   evaluateRandom(vecOfvec&       xx        , vectord&     yy, uint number, double min = 0.0, double max = 1.0);
    void   getOptParams  (tgp_parameters& tgp_params, bopt_params& opt_params);
};

#endif // __RKHS_H__