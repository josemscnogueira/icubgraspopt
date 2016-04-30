/**************************************************************************************************
 *  File:    TGPOptimization.cpp                                                                  *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/

/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <TGPOptimization.hpp>
#include <locale>
#include <vector>
#include <string>
#include <inneroptimization.hpp>


/**************************************************************************************************
 *  Namespace: bayesopt                                                                           *
 **************************************************************************************************/
namespace bayesopt
{

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : TGPOptimization                                                                  *
 **************************************************************************************************/
TGPOptimization::TGPOptimization(TgpParameters tgp_params, Parameters params, TGPOptimizable& func, uint printmode, bool isTGP, uint index)
:
ContinuousModel(tgp_params.dimensions, params),
_func          (func)
{
    // Attribute Initialization
    _tgpparams     = tgp_params;
    _index         = index;
    _isTGP         = isTGP;
    _printmode     = printmode;
    mCurrentIter   = 0;

    // Create Log Manager
    _log = new LogManager(_printmode, _isTGP, _tgpparams, mParameters, _func);

    // Set tgp_model if it's the case
    if (isTGP) mModel.reset(new TreedGaussianProcess(_tgpparams, mParameters, mEngine));

    // If this is the first Optimization, clean results folder
    if (_index == 0)
    {
        _log -> cleanFolderContents();
    }

    // Create Log header for batch of Optimizations
    _log -> printLogHeader();

    // Create Log header for Optimization
    _log -> printTestHeader(_index);
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: findOptimal                                                                         *
 *  Class      : TGPOptimization                                                                  *
 **************************************************************************************************/
void TGPOptimization::findOptimal(vectord& xOpt)
{
    if (_isTGP)
    {
        double                minf      = DBL_MAX;
        TreedGaussianProcess* tgp_model = dynamic_cast<TreedGaussianProcess*>(mModel.get());
        std::vector<TGPNode*> leafs     = tgp_model -> getLeafsPtrs();

        for (uint index = 0; index < leafs.size(); index += 1)
        {
            double  minf_leaf;
            vectord xOpt_leaf(mDims);
            vectord lower_bound, upper_bound;

            // Get leaf bounds
            leafs[index] -> getBounds(lower_bound, upper_bound);

            // Set NLOPT Optimizarion
            cOptimizer -> setLimits(lower_bound, upper_bound);

            // Run NLOPT Optimization
            try
            {
                minf_leaf = cOptimizer -> run(xOpt_leaf);
            }
            catch (std::runtime_error &e)
            {
                FILE_LOG(logERROR) << "NLOPT Error: Runtime Error, General Failure in TGPOptimization::findOptimal.";

                mModel -> updateHyperParameters();
                mModel -> fitSurrogateModel();

                findOptimal(xOpt); return;
            }

            // Verify if xOpt_leaf is a minimum
            if (minf_leaf < minf)
            {
                minf            = minf_leaf;
                xOpt            = xOpt_leaf;
            }
        }

        // Reset Optimizer like for all exploration space
        cOptimizer -> setLimits(zvectord(mDims), vectord(mDims, 1.0));

        // Let's try some local exploration like spearmint
        randNFloat drawSample(mEngine,normalDist(0,0.001));

        for(size_t ii = 0; ii < 5; ++ii)
        {
            vectord pert = getPointAtMinimum();
            for(size_t j = 0; j < xOpt.size(); ++j)
            {
                pert(j) += drawSample();
            }
            try
            {
                double minf2 = cOptimizer -> localTrialAround(pert);
                if (minf2 < minf)
                {
                    minf = minf2;
                    xOpt = pert;
                    FILE_LOG(logDEBUG) << "Local beats Global";
                }
            }
            catch(std::invalid_argument& e)
            {
                //We ignore this one
            }
        }
    }
    else
    {
        ContinuousModel::findOptimal(xOpt);
    }
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: initializeOptimization                                                           *
 *  Class      : TGPOptimization                                                                  *
 **************************************************************************************************/
void TGPOptimization::initializeOptimization(void)
{
    this -> ContinuousModel::initializeOptimization();

    for (uint index = 0; index < mModel -> getData() -> getNSamples(); index += 1)
    {
        // Log current iteration
        if      ( (_printmode & MONTE_CARLO_OPT) != 0)
        {
            vectord lastx   = mModel -> getData() -> getSampleX(index);
            double  lasty   = mModel -> getData() -> getSampleY(index);
            uint    optimum = getOptimumIndex                  (index);

            _func.unnormalizeVector(lastx);

            _log -> printCurrIterMonteCarlo(index+1, lastx, lasty, optimum+1, MC_PTS, MC_STD);
        }
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: optimize                                                                         *
 *  Class      : TGPOptimization                                                                  *
 **************************************************************************************************/
void TGPOptimization::optimize(vectord& best)
{
    // Run Optimization
    this -> ContinuousModel::optimize(best);

    if ( (_printmode & UNSCENTED_OPT) != 0)
    {
        for (uint index = 0; index < mModel -> getData() -> getNSamples(); index += 1)
        {
            vectord lastx   = mModel -> getData() -> getSampleX(index);
            double  lasty   = mModel -> getData() -> getSampleY(index);
            uint    optimum = getOptimumIndex                  (index);
            double  uns_mu  = getUnscentedOptimumValue(lastx);
            double  uns_std = getUnscentedOptimumStd  (lastx);

            _func.unnormalizeVector(lastx);

            _log -> printCurrIterUBO(index+1, lastx, lasty, optimum+1, uns_mu, uns_std);
        }
    }

    // Write Log footer information
    _log -> printTestFooter(_index,       _results, mModel.get());
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: stepOptimization                                                                 *
 *  Class      : TGPOptimization                                                                  *
 **************************************************************************************************/
void TGPOptimization::stepOptimization(void)
{
    this -> ContinuousModel::stepOptimization();

    // Log current iteration
    _log -> printCurrIter(mCurrentIter, _results, mModel.get(), mYPrev);

    if ( (_printmode & MONTE_CARLO_OPT) != 0)
    {
        vectord lastx   = mModel -> getData() -> getSampleX(mModel -> getData() -> getNSamples() - 1);
        double  lasty   = mModel -> getData() -> getSampleY(mModel -> getData() -> getNSamples() - 1);
        uint    optimum = getOptimumIndex                  (mModel -> getData() -> getNSamples() - 1);

        _func.unnormalizeVector(lastx);

        _log -> printCurrIterMonteCarlo(mModel -> getData() -> getNSamples(), lastx, lasty, optimum+1, MC_PTS, MC_STD);
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: optimize                                                                         *
 *  Class      : TGPOptimization                                                                  *
 **************************************************************************************************/
double TGPOptimization::evaluateSample(const vectord& query)
{
    // Evaluate function
    double result = _func.evaluate(query);

    // Push for best results
    _results.push(query, result);

    // Return evaluation result
    return result;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: checkReachability                                                                *
 *  Class      : TGPOptimization                                                                  *
 **************************************************************************************************/
bool TGPOptimization::checkReachability(const vectord& query)
{
    return true;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getBestResults                                                                   *
 *  Class      : TGPOptimization                                                                  *
 **************************************************************************************************/
LearningQueueWrapper TGPOptimization::getBestResults(void)
{
    return _results;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getUnscentedSigmaPoints                                                          *
 *  Class      : TGPOptimization                                                                  *
 **************************************************************************************************/
void TGPOptimization::getUnscentedSigmaPoints(const vectord x, std::vector<vectord>& xx, std::vector<double>& w)
{
    UnscentedExpectedImprovement* uei = new UnscentedExpectedImprovement(_func.dim);
                                  uei -> setParameters(mParameters.crit_params);

    uei -> getSamples(x, xx, w);

    delete uei;
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getOptimumIndex                                                                  *
 *  Class      : TGPOptimization                                                                  *
 **************************************************************************************************/
uint TGPOptimization::getOptimumIndex(uint maximum_index)
{
    const Dataset* data   = mModel -> getData();
    uint           n_data = data -> getNSamples();

    // If there's no data
    if (n_data == 0)             return ((uint)(-1));
    if (maximum_index >= n_data) return ((uint)(-1));

    n_data = maximum_index + 1;

    // Searching Variables
    uint           result = 0;
    double         best   = data -> getSampleY(0);

    // If it is not the Uscented Expected Improvement Criteria
    if ( mParameters.crit_name.compare("cUEI") != 0 )
    {
        for (uint index = 1; index < n_data; index += 1)
        {
            if (best > data -> getSampleY(index))
            {
                best   = data -> getSampleY(index);
                result = index;
            }
        }
    }
    else
    {
        std::vector<vectord>          xx;
        std::vector<double >          w;

        getUnscentedSigmaPoints(data -> getSampleX(0), xx, w);

        best = getUnscentedOptimumValue(xx, w);

        for (uint index = 1; index < n_data; index += 1)
        {
            getUnscentedSigmaPoints(data -> getSampleX(index), xx, w);

            double unscented_value = getUnscentedOptimumValue(xx, w);

            if (best > unscented_value)
            {
                best   = unscented_value;
                result = index;
            }
        }
    }

    return result;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getUnscentedOptimumValue                                                         *
 *  Class      : TGPOptimization                                                                  *
 **************************************************************************************************/
double TGPOptimization::getUnscentedOptimumValue(const std::vector<vectord>& xx, const std::vector<double>& w)
{
    double  result = 0.0;
    double  w_sum  = 0.0;

    for (uint index = 0; index < xx.size(); index += 1)
    {
        ProbabilityDistribution* prediction  = mModel -> getPrediction(xx[index]);
                                 result     += (prediction -> getMean()) * w[index];
                                 w_sum      += w[index];
    }



    return  (result / w_sum);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getUnscentedOptimumValue                                                         *
 *  Class      : TGPOptimization                                                                  *
 **************************************************************************************************/
double TGPOptimization::getUnscentedOptimumValue(const vectord x)
{
    std::vector<vectord> xx;
    std::vector<double > w;

    getUnscentedSigmaPoints(x, xx, w);

    return getUnscentedOptimumValue(xx, w);
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getUnscentedOptimumStd                                                           *
 *  Class      : TGPOptimization                                                                  *
 **************************************************************************************************/
double TGPOptimization::getUnscentedOptimumStd(const std::vector<vectord>& xx, const std::vector<double>& w)
{
    double   result = 0.0;
    double   w_sum  = 0.0;
    double   mu     = getUnscentedOptimumValue(xx, w);

    for (uint index = 0; index < xx.size(); index += 1)
    {
        ProbabilityDistribution* prediction  = mModel -> getPrediction(xx[index]);
                                 result     += std::pow(prediction -> getMean() - mu, 2) * w[index];
                                 w_sum      += w[index];
    }

    return std::sqrt( result / w_sum );
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getUnscentedOptimumStd                                                         *
 *  Class      : TGPOptimization                                                                  *
 **************************************************************************************************/
double TGPOptimization::getUnscentedOptimumStd(const vectord x)
{
    std::vector<vectord> xx;
    std::vector<double > w;

    getUnscentedSigmaPoints(x, xx, w);

    return getUnscentedOptimumStd(xx, w);
}

} // End namespace bayesopt
