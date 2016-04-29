/**************************************************************************************************
 *  File:    OptFuncs.h                                                                           *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/
#ifndef __OPTFUNCS_H__
#define __OPTFUNCS_H__


/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <string>
#include <cmath>

#include <tgpoptimizable.hpp>


/**************************************************************************************************
 *  Class: GramacyExponential                                                                     *
 **************************************************************************************************/
class GramacyExponential : public TGPOptimizable
{
public:
    // Constructor
    GramacyExponential(void) {name = std::string("GramacyExponential_2D"); ymax = 0.4; ymin = -0.4; dim = 2;
                                                                           lower_bound = vectord(dim, 0)   ;
                                                                           upper_bound = vectord(dim, 1)   ; };

    // Methods
    double evaluate(vectord x)
    {
        if (x.size() != 2) return 0.0;

        x(0) = (x(0) * 8) - 2;
        x(1) = (x(1) * 8) - 2;

        return (x(0) * std::exp(-sqr(x(0))-sqr(x(1))));
    };


    void getOptParams(tgp_parameters& tgp_params, bopt_params& opt_params)
    {
        TGPOptimizable::getOptParams(tgp_params, opt_params);

        opt_params.n_init_samples    =  20;
        opt_params.n_iterations      =  80 - opt_params.n_init_samples;
        opt_params.crit_params[0]    =   1;    // exp
        opt_params.crit_params[1]    =   0.01; // bias
        opt_params.noise             =   1e-6;
        opt_params.sigma_s           =   0.40;

        tgp_params.min_data_per_leaf  =  15;
    };

    void initSamples(vecOfvec& xx, vectord& yy) { };

private:
    double sqr(double x) {return x*x;};
};


/**************************************************************************************************
 *  Class: Branin                                                                                 *
 **************************************************************************************************/
class Branin : public TGPOptimizable
{
public:
    // Constructor
    Branin(void) {name = std::string("Branin_2D"); ymax = 300; ymin = 0; dim = 2;
                                                   lower_bound    = vectord(dim);
                                                   upper_bound    = vectord(dim);
                                                   lower_bound[0] = -5;
                                                   lower_bound[1] =  0;
                                                   upper_bound[0] = 10;
                                                   upper_bound[1] = 15;};

    // Methods
    double evaluate(vectord x)
    {
        if (x.size() != 2) return ymax;

        double xx = x(0);
        double yy = x(1);

        return (sqr(yy - (5.1 / (4 * sqr(M_PI))) * sqr(xx) + 5 * (xx/M_PI) - 6) + 10 * (1 - 1/(8 * M_PI)) * cos(xx) + 10);
    };


    void getOptParams(tgp_parameters& tgp_params, bopt_params& opt_params)
    {
        TGPOptimizable::getOptParams(tgp_params, opt_params);

        opt_params.n_iterations      = 100;
        opt_params.n_init_samples    =   6;

        tgp_params.min_data_per_leaf  =  15;
    };

    void initSamples(vecOfvec& xx, vectord& yy) { };

protected:
    double sqr(double x) {return x*x;};
};


/**************************************************************************************************
 *  Class: Easom                                                                                  *
 **************************************************************************************************/
class Easom : public TGPOptimizable
{
public:
    // Constructor
    Easom(void) {name = std::string("Easom_2D"); ymax = 0; ymin = -1; dim = 2;
                                                 lower_bound    = vectord(dim);
                                                 upper_bound    = vectord(dim);
                                                 lower_bound[0] = -5;
                                                 lower_bound[1] =  0;
                                                 upper_bound[0] = 10;
                                                 upper_bound[1] = 15;};

    // Methods
    double evaluate(vectord x)
    {
        if (x.size() != 2) return ymax;

        return ( -cos(x[0]) * cos(x[1]) * exp(-pow(x[0] - M_PI, 2) - pow(x[1] - M_PI, 2)) );
    };


    void getOptParams(tgp_parameters& tgp_params, bopt_params& opt_params)
    {
        TGPOptimizable::getOptParams(tgp_params, opt_params);

        opt_params.n_iterations      = 100;
        opt_params.n_init_samples    =   6;

        tgp_params.min_data_per_leaf  =  15;
    };

    void initSamples(vecOfvec& xx, vectord& yy) { };
};


/**************************************************************************************************
 *  Class: BraninEasom                                                                                  *
 **************************************************************************************************/
class BraninEasom : public Branin
{
public:
    // Constructor
    BraninEasom(void) {name = std::string("BraninEasom_2D"); ymax = 300; ymin = -1; dim = 2;
                                                             lower_bound    = vectord(dim);
                                                             upper_bound    = vectord(dim);
                                                             lower_bound[0] = -5;
                                                             lower_bound[1] =  0;
                                                             upper_bound[0] = 10;
                                                             upper_bound[1] = 15;};

    // Methods
    double evaluate(vectord x)
    {
        if (x.size() != 2) return ymax;

        Branin func1;
        Easom  func2;

        vectord center    = vectord(2);
                center[0] = M_PI - M_PI;
                center[1] = M_PI - 2.275;

        return ((func1.evaluate(x) + (2 * func2.evaluate(x+center)))-10);
    };


    void getOptParams(tgp_parameters& tgp_params, bopt_params& opt_params)
    {
        TGPOptimizable::getOptParams(tgp_params, opt_params);

        opt_params.n_init_samples    =   6;
        opt_params.n_iterations      = 150 - opt_params.n_init_samples;
        opt_params.crit_params[0]    =   1;   // exp
        opt_params.crit_params[1]    =   0.1; // bias

        tgp_params.min_data_per_leaf  =  15;
    };

    void initSamples(vecOfvec& xx, vectord& yy) { };
};

/**************************************************************************************************
 *  Class: BraninEasomNormalized                                                                  *
 **************************************************************************************************/
class BraninEasomNormalized : public Branin
{
public:
    // Constructor
    BraninEasomNormalized(void) {name = std::string("BraninEasomNormalized_2D"); ymax = 300; ymin = -1; dim = 2;
                                                                                 lower_bound    = vectord(dim);
                                                                                 upper_bound    = vectord(dim);
                                                                                 lower_bound[0] =  0;
                                                                                 lower_bound[1] =  0;
                                                                                 upper_bound[0] =  1;
                                                                                 upper_bound[1] =  1;};

    // Methods
    double evaluate(vectord x)
    {
        if (x.size() != 2) return ymax;

        double xx = x(0) * 15 - 5;
        double yy = x(1) * 15;

        return branineasom(xx,yy);
    };

    double branineasom(double x, double y)
    {
      double x1p    = x - (2 * M_PI);
      double x2p    = y - (2 * M_PI);
      double easom  = - x * y * std::exp(- (x1p * x1p) - (x2p * x2p));
      double branin = sqr(y - (5.1 / (4*sqr(M_PI))) * sqr(x) + (5 * x / M_PI) - 6) + 10 * (1 - (1 / (8 * M_PI))) * cos(x) + 10;

      return branin + (5 * easom);
    };

    void getOptParams(tgp_parameters& tgp_params, bopt_params& opt_params)
    {
        TGPOptimizable::getOptParams(tgp_params, opt_params);

        opt_params.n_init_samples    =   6;
        opt_params.n_iterations      = 150 - opt_params.n_init_samples;
        opt_params.crit_params[0]    =   1;   // exp
        opt_params.crit_params[1]    =   0.1; // bias

        tgp_params.min_data_per_leaf  =  15;
    };

    void initSamples(vecOfvec& xx, vectord& yy) { };
};



/**************************************************************************************************
 *  Class: Michalewicz                                                                            *
 **************************************************************************************************/
class Michalewicz : public TGPOptimizable
{
private:
    // Attributes
    double _exp = 10;


public:
    // Constructor
    Michalewicz(size_t dim) { name = std::string("Michalewicz"); ymax = 1; ymin = -1; this -> dim = dim;
                                                                 lower_bound = vectord(dim, 0);
                                                                 upper_bound = vectord(dim, 1);         };

    // Methods
    double evaluate(vectord x)
    {

        if (dim != x.size()) return 0.0;

        double result  = 0.0;
                    x *= M_PI;

        for(uint index = 0; index < dim; index += 1)
        {
            double frac  = x(index) * x(index) * (index + 1);
                   frac /= M_PI;

            result      += std::sin(x(index)) * std::pow(std::sin(frac), 2*_exp);
        }

        return -result;
    };

    void getOptParams(tgp_parameters& tgp_params, bopt_params& opt_params)
    {
        TGPOptimizable::getOptParams(tgp_params, opt_params);

        opt_params.n_iterations      = 200;
        opt_params.n_init_samples    =  30;
        opt_params.crit_params[0]    =  1;   // exp
        opt_params.crit_params[1]    =  0.01; // bias

        tgp_params.min_data_per_leaf =  30;
    };

    void initSamples(vecOfvec& xx, vectord& yy) { };
};

/**************************************************************************************************
 *  Class: Michalewicz                                                                            *
 **************************************************************************************************/
class Martinez : public TGPOptimizable
{
public:
    // Constructor
    Martinez(void) { name = std::string("Martinez"); ymax = 1; ymin = -1;  dim = 2;
                                                     lower_bound = vectord(dim, 0);
                                                     upper_bound = vectord(dim, 1); };

    // Methods
    double evaluate(vectord x);

    void getOptParams(tgp_parameters& tgp_params, bopt_params& opt_params)
    {
        TGPOptimizable::getOptParams(tgp_params, opt_params);

        opt_params.n_init_samples    =  30;
        opt_params.n_iterations      = 150 - opt_params.n_init_samples;
        opt_params.crit_params[0]    =   1;   // exp
        opt_params.crit_params[1]    =   0.01; // bias
        opt_params.noise             =  1e-6;
        opt_params.sigma_s           =   0.0642;

        tgp_params.min_data_per_leaf =  15;
    };

    void initSamples(vecOfvec& xx, vectord& yy) { };
};


#endif // __OPTFUNCS_H__
