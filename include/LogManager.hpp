/**************************************************************************************************
 *  File:    LogManager.h                                                                         *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/
#ifndef __LOGMANAGER_HPP__
#define __LOGMANAGER_HPP__


/**************************************************************************************************
 *  Include Files                                                                                 *
 **************************************************************************************************/
#include <fstream>
#include <iostream>
#include <vector>
#include <string>
#include <boost/filesystem.hpp>

#include <bayesopt/bayesopt.hpp>
#include <tgpoptimizable.hpp>     // Bayesopt
#include <learningqueue.hpp>

#define MC_STD  0.01
#define MC_PTS  100

/**************************************************************************************************
 *  Enumerations                                                                                  *
 **************************************************************************************************/
enum
{
    LOG_LEARNING       = 0b00000001,
    LOG_POSTERIOR      = 0b00000010,
    LOG_OBJ_FUNCTION   = 0b00000100,
};

enum
{
    METRIC_OPT         = 0b0000000100000000,
    SAMPLES            = 0b0000001000000000,
    CREATE_MATLAB_FIGS = 0b0000010000000000,
    MONTE_CARLO_OPT    = 0b0000100000000000,
    UNSCENTED_OPT      = 0b0001000000000000,
};


/**************************************************************************************************
 *  Class: LogManager                                                                             *
 **************************************************************************************************/
class LogManager
{
public:
    // Constructor
    LogManager(void);
    LogManager(uint type);
    LogManager(uint type, std::string dir_name);
    LogManager(uint type, bool& isTGP, bayesopt::TgpParameters& tgp_params, bayesopt::Parameters& opt_params, bayesopt::TGPOptimizable& func);
    LogManager(bayesopt::TGPOptimizable& func);
    LogManager(bayesopt::TGPOptimizable& func, bayesopt::TgpParameters& tgp_params);

    // Methods
    void cleanFolderContents    (void);
    void printLogHeader         (void);
    void printTestHeader        (uint test_index);
    void printTestFooter        (uint test_index, LearningQueueWrapper& results, bayesopt::PosteriorModel* model);
    void printCurrIter          (uint iteration,  LearningQueueWrapper& results, bayesopt::PosteriorModel* model, double y_prev);
    void printCurrIterMonteCarlo(const uint iteration, const vectord query, const double y, const uint optimum, const uint   points, const double std_dev);
    void printCurrIterUBO       (const uint iteration, const vectord query, const double y, const uint optimum, const double uns_mu, const double uns_std);
    void printLogFooter         (uint iteration);
    void printFuncProfile       (uint points = 100);
    void printMonteCarlo        (uint points, const vectord query, double              std_dev);
    void printMonteCarlo        (uint points, const vectord query, std::vector<double> std_dev);

    // Static methods
    static std::string optimizationToString(bool                      isTGP,
                                            bayesopt::TgpParameters   tgp_params,
                                            bayesopt::Parameters      opt_params,
                                            bayesopt::TGPOptimizable& func);

private:
    // File system Attributes
    boost::filesystem::path  _path_binary;
    boost::filesystem::path  _path_folder;
    std::ofstream            _output;

    // Optimization References
    bayesopt::Parameters*      _opt_params;
    bayesopt::TgpParameters*   _tgp_params;
    bayesopt::TGPOptimizable*  _func;
    bool*                      _isTGP;

    uint             _type;
    uint             _mode;

    // Methods
    bool checkInit          (void);
    void callMatlab         (uint iteration);
    void getMonteCarloPoints(uint points, const vectord query, std::vector<double> std_dev, std::vector<vectord>& xx, std::vector<double>& yy);
    void getMonteCarloPoints(uint points, const vectord query,             double  std_dev, std::vector<vectord>& xx, std::vector<double>& yy);
};

 #endif // __LOGMANAGER_H__
