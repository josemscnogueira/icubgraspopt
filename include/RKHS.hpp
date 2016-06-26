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

namespace bayesopt {

/**************************************************************************************************
 *  Class: RKHS                                                                                   *
 **************************************************************************************************/
class RKHS : public TGPOptimizable
{
private:
    vectord             _support_1;
    vectord             _support_2;
    vectord             _vals_1;
    vectord             _vals_2;
    double              _hyp_1;
    double              _hyp_2;

    std::vector<double> a_support_1;
    std::vector<double> a_support_2;
    std::vector<double> a_vals_1;
    std::vector<double> a_vals_2;

public:
    // Constructor
    RKHS(void);

    // Destructor
    ~RKHS(void) { };

    // Methods
    double covSEard      (const double   hyp, double x1, double x2);
    double evaluate      (vectord        x);
    void   initSamples   (vecOfvec&      xx        , vectord&     yy);
    void   getOptParams  (TgpParameters& tgp_params, Parameters& opt_params);

    void   loadJson(Json::Value config) { }; // Cannot be loaded
};

} // End of namespace bayesopt


#endif // __RKHS_H__
