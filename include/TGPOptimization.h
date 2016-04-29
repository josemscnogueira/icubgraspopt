/**************************************************************************************************
 *  File:    TGPOptimization.h                                                                    *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/
#ifndef __TGPOPTIMIZATION_H__
#define __TGPOPTIMIZATION_H__


/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <bayesopt.hpp>
#include <tgpoptimizable.hpp>     // Bayesopt
#include <learningqueue.hpp>
#include <treedgaussianprocess.hpp>
#include <LogManager.h>


/**************************************************************************************************
 *  Namespace: bayesopt                                                                           *
 **************************************************************************************************/
namespace bayesopt
{


/**************************************************************************************************
 *  Class: TGPOptimization                                                                        *
 **************************************************************************************************/
class TGPOptimization : public ContinuousModel
{
public:
    // Constructor
    TGPOptimization(tgp_parameters  tgp_params,
                    bopt_params     params,
                    TGPOptimizable& func,
                    uint            printmode,
                    bool            isTGP  = false,
                    uint            index  = 0);

    // Destructor
    virtual ~TGPOptimization(void) { delete _log; };

    // Methods
    void                 initializeOptimization(void);
    void                 optimize              (      vectord& best );
    void                 stepOptimization      (void);
    double               evaluateSample        (const vectord& query);
    bool                 checkReachability     (const vectord& query);
    void                 printLogFooter        (void) { _log -> printLogFooter(mCurrentIter); };
    LearningQueueWrapper getBestResults        (void);

    uint                 getOptimumIndex       (uint maximum_index);

    // Static methods
    static bopt_params   initializeOptParams   (void);

protected:
    // Methods
    void    findOptimal             (      vectord&              xOpt );
    void    getUnscentedSigmaPoints (const vectord x,       std::vector<vectord>& xx,       std::vector<double>& w);
    double  getUnscentedOptimumValue(                 const std::vector<vectord>& xx, const std::vector<double>& w);
    double  getUnscentedOptimumStd  (                 const std::vector<vectord>& xx, const std::vector<double>& w);
    double  getUnscentedOptimumValue(const vectord x);
    double  getUnscentedOptimumStd  (const vectord x);

private:
    // Attributes
    TGPOptimizable&          _func;
    tgp_parameters           _tgpparams;
    bool                     _isTGP;

    uint                     _index;
    uint                     _printmode;
    LogManager*              _log;

    LearningQueueWrapper     _results;
};

} // End: namespace bayesopt

#endif // _TGPOPTIMIZATION_H_
