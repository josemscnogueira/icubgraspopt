/**************************************************************************************************
 *  File:    RKHS.h                                                                               *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/
#ifndef __RKHS_HPP__
#define __RKHS_HPP__


/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <string>           // Global
#include <stdlib.h>

#include <json/json.h>

#include <specialtypes.hpp> // Bayesopt
#include <tgpoptimizable.hpp>
// #include <kernels/kernel_gaussian.hpp>


namespace bayesopt {

/**************************************************************************************************
 *  Class: RKHS                                                                                   *
 **************************************************************************************************/
class RKHS : public TGPOptimizable
{
private:
    vectord _support_1;
    vectord _support_2;
    vectord _vals_1;
    vectord _vals_2;
    vectord _hyp_1;
    vectord _hyp_2;

    // SEArd   kernel;

public:
    // Constructor
    RKHS(void);

    // Destructor
    ~RKHS(void) { };

    // Methods
    double covSEard      (const vectord&  hyp,  const vectord&     x1, const vectord& x2);
    double evaluate      (vectord         x);
    void   initSamples   (vecOfvec&       xx        , vectord&     yy);
    void   getOptParams  (TgpParameters& tgp_params, Parameters& opt_params);

    void   loadJson(Json::Value config) { }; // Cannot be loaded
};

} // End of namespace bayesopt


#endif // __RKHS_H__
