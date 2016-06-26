/**************************************************************************************************
 *  File:    RKHS.cpp                                                                             *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/

/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <RKHS.hpp>
#include <boost/assign/list_of.hpp>


namespace bayesopt
{


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : RKHS                                                                             *
 **************************************************************************************************/
RKHS::RKHS(void)
{
    std::vector<double> a_support_1 = boost::assign::list_of(0.1)(0.15)(0.08)(0.3)(0.4);
    std::vector<double> a_support_2 = boost::assign::list_of(0.8)(0.85)(0.9)(0.95)(0.92)(0.74)(0.91)(0.89)(0.79)(0.88)(0.86)(0.96)(0.99)(0.82);
    std::vector<double> a_vals_1    = boost::assign::list_of(4)(-1)(2.)(-2.)(1.);
    std::vector<double> a_vals_2    = boost::assign::list_of(3)(4)(2)(1)(-1)(2)(2)(3)(3)(2.)(-1.)(-2.)(4.)(-3.);


    ymax        =  7;
    ymin        = -4;
    lower_bound = vectord(1, 0);
    upper_bound = vectord(1, 1);

    // Update name
    name = std::string("RKHS_1D");
    dim  = 1;

    // Define Support Vectors
    _support_1 = vectord( 5);
    _support_2 = vectord(14);
    _vals_1    = vectord( 5);
    _vals_2    = vectord(14);

    std::copy(a_support_1.begin(), a_support_1.end(), _support_1.begin());
    std::copy(a_support_2.begin(), a_support_2.end(), _support_2.begin());
    std::copy(a_vals_1   .begin(), a_vals_1   .end(), _vals_1   .begin());
    std::copy(a_vals_2   .begin(), a_vals_2   .end(), _vals_2   .begin());

    // Initialize Gaussian Kernels
    _hyp_1 = 0.10;
    _hyp_2 = 0.01;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: covSEard                                                                         *
 *  Class      : RKHS                                                                             *
 **************************************************************************************************/
double RKHS::covSEard(const double hyp, double x1, double x2)
{
    x1 /= hyp;
    x2 /= hyp;

    double dist = x1 - x2;

    return std::exp(-(dist*dist)/ 2);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: evaluate                                                                         *
 *  Class      : RKHS                                                                             *
 **************************************************************************************************/
double RKHS::evaluate(vectord x)
{
    // Assert x size
    assert(x.size() == 1);

    double result = 0.0;

    // First kernel length
    for (uint index = 0; index < _support_2.size(); ++index)
    {
        result += _vals_2[index] * covSEard(_hyp_2, _support_2[index], x[0]);
    }

    // Second kernel length
    for (uint index = 0; index < _support_1.size(); ++index)
    {
        result += _vals_1[index] * covSEard(_hyp_1, _support_1[index], x[0]);
    }

    return -result;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: initSamples                                                                      *
 *  Class      : RKHS                                                                             *
 **************************************************************************************************/
void RKHS::initSamples(vecOfvec& xx, vectord& yy)
{
    // Set XX
    xx.clear();
    xx.push_back(vectord(1, 0.1));
    xx.push_back(vectord(1, 0.5));
    xx.push_back(vectord(1, 0.9));

    yy = vectord(xx.size(), 0);

    for (uint sample = 0; sample < xx.size(); sample += 1)
    {
        yy[sample] = evaluate(xx[sample]);
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getOptParams                                                                     *
 *  Class      : RKHS                                                                             *
 **************************************************************************************************/
void RKHS::getOptParams(TgpParameters& tgp_params, Parameters& opt_params)
{
    TGPOptimizable::getOptParams(tgp_params, opt_params);

    opt_params.n_iterations      = 45;
    opt_params.n_init_samples    =  5;
    opt_params.crit_params[0]    =  1;    // exp
    opt_params.crit_params[1]    =  0.01; // bias
    opt_params.noise             =  1e-6;
    opt_params.sigma_s           =  3.93;

    tgp_params.min_data_per_leaf =  8;
}

} // End of namespace bayesopt
