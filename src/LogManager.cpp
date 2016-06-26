/**************************************************************************************************
 *  File:    LogManager.cpp                                                                       *
 *  Author:  Jose Miguel Nogueira, josemscnogueira@gmail.com                                      *
 *                                                                                                *
 *  History:                                                                                      *
 **************************************************************************************************/

 /**************************************************************************************************
  *  Include Files                                                                                 *
  **************************************************************************************************/
#include <LogManager.hpp>
#include <ctime>
#include <numeric>
#include <boost/lexical_cast.hpp>
#include <boost/random.hpp>
#include <boost/algorithm/string.hpp>
#include <learningqueue.hpp>
#include <treedgaussianprocess.hpp>

using namespace bayesopt;

/**
 * Init Function for Constructor overloading
 */
void LogManager::init(void)
{
    _path_binary =                boost::filesystem::current_path();
    _path_folder = _path_binary / boost::filesystem::path("results");

    _opt_params = NULL;
    _tgp_params = NULL;
    _func       = NULL;
    _isTGP      = NULL;

    // If directory doens't exist, create it
    if (!boost::filesystem::exists(_path_folder)) boost::filesystem::create_directory(_path_folder);
}

void LogManager::init(uint type)
{
    init();

    _mode = type & 0b1111111100000000;
    _type = type & 0b0000000011111111;

    switch(_type)
    {
        case(LOG_LEARNING    ) : _path_folder /= boost::filesystem::path("learning" ); break;
        case(LOG_POSTERIOR   ) : _path_folder /= boost::filesystem::path("posterior"); break;
        case(LOG_OBJ_FUNCTION) : _path_folder /= boost::filesystem::path("function" ); break;
        default                : return;
    }

    // If directory doens't exist, create it
    if (!boost::filesystem::exists(_path_folder)) boost::filesystem::create_directory(_path_folder);
}


void LogManager::init(TGPOptimizable& func)
{
    init(LOG_OBJ_FUNCTION);

    std::string dir_name = func.name + "_" + boost::lexical_cast<std::string>(func.dim) + "D";

    _func       = &func;

    // Update Directory Path
    _path_folder /= boost::filesystem::path(dir_name.c_str());

    // If directory doens't exist, create it
    if (!boost::filesystem::exists(_path_folder)) boost::filesystem::create_directory(_path_folder);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
LogManager::LogManager(void)
{
    init();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
LogManager::LogManager(uint type)
{
    init(type);
}




/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
LogManager::LogManager(uint type, std::string dir_name)
{
    init(type);

    // Update File Path
    _path_folder /= boost::filesystem::path(dir_name.c_str());

    // If directory doens't exist, create it
    if (!boost::filesystem::exists(_path_folder)) boost::filesystem::create_directory(_path_folder);
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
LogManager::LogManager(uint type, bool& isTGP, TgpParameters& tgp_params, Parameters& opt_params, TGPOptimizable& func)
{
    init(type);

    std::string dir_name = optimizationToString(isTGP, tgp_params, opt_params, func);

    _opt_params = &opt_params;
    _tgp_params = &tgp_params;
    _func       = &func;
    _isTGP      = &isTGP;

    // Update Directory Path
    _path_folder /= boost::filesystem::path(dir_name.c_str());

    // If directory doens't exist, create it
    if (!boost::filesystem::exists(_path_folder)) boost::filesystem::create_directory(_path_folder);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
LogManager::LogManager(TGPOptimizable& func)
{
    init(func);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: Constructor                                                                      *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
LogManager::LogManager(TGPOptimizable& func, TgpParameters& tgp_params)
{
    init(func);

    _tgp_params = &tgp_params;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: cleanFolderContents                                                              *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
void LogManager::cleanFolderContents(void)
{
    if (boost::filesystem::exists(_path_folder)) boost::filesystem::remove_all(_path_folder);

    boost::filesystem::create_directory(_path_folder);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: checkInit                                                                        *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
bool LogManager::checkInit(void)
{
    if ( _opt_params == NULL ) return false;
    if ( _tgp_params == NULL ) return false;
    if ( _func       == NULL ) return false;
    if ( _isTGP      == NULL ) return false;

    return true;
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: printLogHeader                                                                   *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
void LogManager::printLogHeader(void)
{
    std::cout << std::endl << _type;
    std::cout << std::endl << _mode;

    if ( ((_type & LOG_LEARNING   ) != 0) &&
         ((_mode & METRIC_OPT     ) != 0)   )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/learning.m";

        if ( boost::filesystem::exists(filename.c_str()) ) return;

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);

        // Write to file
        _output << "clear;"                                                        << std::endl;
        _output << "logtype            = 'Learning Toolbox';"                     << std::endl;

        if ((*_isTGP)) _output << "surrogate          = 'tgp';"                      << std::endl;
        else           _output << "surrogate          = 'gp';"                       << std::endl;

        _output << "optimizable        = '" << _func -> name                      << "';" << std::endl;
        _output << "max_iter           = "  << _opt_params -> n_iterations        <<  ";" << std::endl;
        _output << "rand_iter          = "  << _opt_params -> n_init_samples      <<  ";" << std::endl;
        _output << "learning_criteria  = '" << _opt_params -> crit_name           << "';" << std::endl;
        _output << "dims               = "  << _func -> dim                       <<  ";" << std::endl;
        _output << "sigma_s            = "  << _opt_params -> sigma_s             <<  ";" << std::endl;
        _output << "epsilon            = "  << _opt_params -> epsilon             <<  ";" << std::endl;
        _output << "force_jump         = "  << _opt_params -> force_jump          <<  ";" << std::endl;
        _output << "noise              = "  << _opt_params -> noise               <<  ";" << std::endl << std::endl;

        if ((*_isTGP))
        {
            _output << "mcmc_particles     = " << _tgp_params -> mcmc_particles    <<  ";" << std::endl;
            _output << "min_data_per_leaf  = " << _tgp_params -> min_data_per_leaf <<  ";" << std::endl;
            _output << "wheight_power      = " << _tgp_params -> wheight_power     <<  ";" << std::endl;
            _output << "wheight_threshold  = " << _tgp_params -> wheight_threshold <<  ";" << std::endl << std::endl;
        }

        if (_tgp_params -> others.size() > 0)
        {
            for (uint index = 0; index < _tgp_params -> others.size(); index += 1)
            {
                _output << _tgp_params -> others[index] << ";" << std::endl;
            }
        }

        _output << std::endl;

        // Close file
        _output.close();
    }

    if ( ((_type & LOG_LEARNING   ) != 0) &&
         ((_mode & MONTE_CARLO_OPT) != 0)   )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/mc_learning.m";

        if ( boost::filesystem::exists(filename.c_str()) ) return;

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);

        // Write to file
        _output << "clear;"                                                        << std::endl;
        _output << "logtype            = 'Learning Toolbox MC';"                    << std::endl;

        if ((*_isTGP)) _output << "surrogate          = 'tgp';"                      << std::endl;
        else           _output << "surrogate          = 'gp';"                       << std::endl;

        _output << "optimizable        = '" << _func -> name                   << "';" << std::endl;
        _output << "max_iter           = "  << _opt_params -> n_iterations        <<  ";" << std::endl;
        _output << "rand_iter          = "  << _opt_params -> n_init_samples      <<  ";" << std::endl;
        _output << "learning_criteria  = '" << _opt_params -> crit_name           << "';" << std::endl;
        _output << "dims               = "  << _func -> dim                       <<  ";" << std::endl;
        _output << "sigma_s            = "  << _opt_params -> sigma_s             <<  ";" << std::endl;
        _output << "epsilon            = "  << _opt_params -> epsilon             <<  ";" << std::endl;
        _output << "force_jump         = "  << _opt_params -> force_jump          <<  ";" << std::endl;
        _output << "noise              = "  << _opt_params -> noise               <<  ";" << std::endl << std::endl;

        if ((*_isTGP))
        {
            _output << "mcmc_particles     = " << _tgp_params -> mcmc_particles    <<  ";" << std::endl;
            _output << "min_data_per_leaf  = " << _tgp_params -> min_data_per_leaf <<  ";" << std::endl;
            _output << "wheight_power      = " << _tgp_params -> wheight_power     <<  ";" << std::endl;
            _output << "wheight_threshold  = " << _tgp_params -> wheight_threshold <<  ";" << std::endl << std::endl;
        }

        if (_tgp_params -> others.size() > 0)
        {
            for (uint index = 0; index < _tgp_params -> others.size(); index += 1)
            {
                _output << _tgp_params -> others[index] << ";" << std::endl;
            }
        }

        _output << std::endl;

        _output << "mc_pts             = " << MC_PTS << ";" << std::endl;
        _output << "mc_std             = " << MC_STD << ";" << std::endl;
        _output << "column_headers     = ['Iteration', 'Optimum Iteration', 'y(iteration)', 'ymean_{mc}(iteration)', 'ystd_{mc}(iteration)', 'yworst_{mc}(iteration)'];";
        _output << std::endl << std::endl;

        // Close file
        _output.close();
    }

    if ( ((_type & LOG_LEARNING   ) != 0) &&
         ((_mode & UNSCENTED_OPT) != 0)   )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/uns_learning.m";

        if ( boost::filesystem::exists(filename.c_str()) ) return;

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);

        // Write to file
        _output << "clear;"                                                        << std::endl;
        _output << "logtype            = 'Learning Toolbox Unscented';"            << std::endl;

        if ((*_isTGP)) _output << "surrogate          = 'tgp';"                      << std::endl;
        else           _output << "surrogate          = 'gp';"                       << std::endl;

        _output << "optimizable        = '" << _func -> name                   << "';" << std::endl;
        _output << "max_iter           = "  << _opt_params -> n_iterations        <<  ";" << std::endl;
        _output << "rand_iter          = "  << _opt_params -> n_init_samples      <<  ";" << std::endl;
        _output << "learning_criteria  = '" << _opt_params -> crit_name           << "';" << std::endl;
        _output << "dims               = "  << _func -> dim                       <<  ";" << std::endl;
        _output << "sigma_s            = "  << _opt_params -> sigma_s             <<  ";" << std::endl;
        _output << "epsilon            = "  << _opt_params -> epsilon             <<  ";" << std::endl;
        _output << "force_jump         = "  << _opt_params -> force_jump          <<  ";" << std::endl;
        _output << "noise              = "  << _opt_params -> noise               <<  ";" << std::endl << std::endl;

        if ((*_isTGP))
        {
            _output << "mcmc_particles     = " << _tgp_params -> mcmc_particles    <<  ";" << std::endl;
            _output << "min_data_per_leaf  = " << _tgp_params -> min_data_per_leaf <<  ";" << std::endl;
            _output << "wheight_power      = " << _tgp_params -> wheight_power     <<  ";" << std::endl;
            _output << "wheight_threshold  = " << _tgp_params -> wheight_threshold <<  ";" << std::endl << std::endl;
        }

        if (_tgp_params -> others.size() > 0)
        {
            for (uint index = 0; index < _tgp_params -> others.size(); index += 1)
            {
                _output << _tgp_params -> others[index] << ";" << std::endl;
            }
        }

        _output << std::endl;

        _output << "mc_pts             = " << MC_PTS << ";" << std::endl;
        _output << "mc_std             = " << MC_STD << ";" << std::endl;
        _output << "column_headers     = ['Iteration', 'Optimum Iteration', 'y(iteration)', 'ubo_ymean(iteration)', 'ubo_ystd_(iteration)'];";
        _output << std::endl << std::endl;

        // Close file
        _output.close();
    }


    if ( ((_type & LOG_LEARNING) != 0) &&
         ((_mode & SAMPLES     ) != 0)    )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/samples.m";

        if ( boost::filesystem::exists(filename.c_str()) ) return;

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);

        // Write to file
        _output << "clear;"                                                    << std::endl;
        _output << "logtype           = 'Learning Samples';"                   << std::endl;
        _output << "n_samples         = " << _tgp_params -> samples_to_save << ";" << std::endl;

        if ((*_isTGP)) _output << "surrogate         = 'tgp';"                    << std::endl;
        else           _output << "surrogate         = 'gp';"                     << std::endl;

        _output << "optimizable       = '" << _func -> name                    << "';" << std::endl;
        _output << "max_iter          = "  << _opt_params -> n_iterations         <<  ";" << std::endl;
        _output << "rand_iter         = "  << _opt_params -> n_init_samples       <<  ";" << std::endl;
        _output << "learning_criteria = '" << _opt_params -> crit_name            << "';" << std::endl;
        _output << "dims              = "  << _func -> dim                        <<  ";" << std::endl;
        _output << "sigma_s           = "  << _opt_params -> sigma_s              <<  ";" << std::endl;
        _output << "epsilon           = "  << _opt_params -> epsilon              <<  ";" << std::endl;
        _output << "force_jump        = "  << _opt_params -> force_jump           <<  ";" << std::endl;
        _output << "noise             = "  << _opt_params -> noise                <<  ";" << std::endl << std::endl;

        if ((*_isTGP))
        {
            _output << "mcmc_particles    = " << _tgp_params -> mcmc_particles    <<  ";" << std::endl;
            _output << "min_data_per_leaf = " << _tgp_params -> min_data_per_leaf <<  ";" << std::endl;
            _output << "wheight_power     = " << _tgp_params -> wheight_power     <<  ";" << std::endl;
            _output << "wheight_threshold = " << _tgp_params -> wheight_threshold <<  ";" << std::endl << std::endl;
        }

        if (_tgp_params -> others.size() > 0)
        {
            for (uint index = 0; index < _tgp_params -> others.size(); index += 1)
            {
                _output << _tgp_params -> others[index] << ";" << std::endl;
            }
        }

        _output << std::endl;

        // Close file
        _output.close();
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: printTestHeader                                                                  *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
void LogManager::printTestHeader(uint test_index)
{
    if ( ((_type & LOG_LEARNING) != 0) &&
         ((_mode & METRIC_OPT  ) != 0)    )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/learning.m";

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

        // Write to file
        _output << "Test(:,:," << test_index + 1 << ") = interpolateMatrix( [" << std::endl;

        // Close file
        _output.close();
    }

    if ( ((_type & LOG_LEARNING   ) != 0) &&
         ((_mode & MONTE_CARLO_OPT) != 0)    )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/mc_learning.m";

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

        // Write to file
        _output << "Test(:,:," << test_index + 1 << ") = interpolateMatrix( [" << std::endl;

        // Close file
        _output.close();
    }

    if ( ((_type & LOG_LEARNING   ) != 0) &&
         ((_mode & UNSCENTED_OPT  ) != 0)    )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/uns_learning.m";

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

        // Write to file
        _output << "Test(:,:," << test_index + 1 << ") = interpolateMatrix( [" << std::endl;

        // Close file
        _output.close();
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: printTestHeader                                                                  *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
void LogManager::printTestFooter(uint test_index, LearningQueueWrapper& results, bayesopt::PosteriorModel* model)
{
    if ( ((_type & LOG_LEARNING) != 0) &&
         ((_mode & METRIC_OPT  ) != 0)    )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/learning.m";

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

        // Write to file
        _output << "           ], max_iter);" << std::endl << std::endl;

        _output << "best(" << test_index + 1 << ",:) = [";

        for(uint index = 0; index < _func -> dim; index += 1)
        {
            _output << results[results.size() - 1].query(index) << "  ";
        }

        _output << "];" << std::endl << std::endl;

        // Close file
        _output.close();
    }

    if ( ((_type & LOG_LEARNING   ) != 0) &&
         ((_mode & MONTE_CARLO_OPT) != 0)    )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/mc_learning.m";

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

        // Write to file
        _output << "           ], max_iter+rand_iter);" << std::endl << std::endl;

        // Close file
        _output.close();
    }

    if ( ((_type & LOG_LEARNING   ) != 0) &&
         ((_mode & UNSCENTED_OPT  ) != 0)    )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/uns_learning.m";

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

        // Write to file
        _output << "           ], max_iter+rand_iter);" << std::endl << std::endl;

        // Close file
        _output.close();
    }

    if ( ((_type & LOG_LEARNING) != 0) &&
         ((_mode & SAMPLES     ) != 0)    )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/samples.m";

        // Prepare data
        LearningQueueWrapper     samples = LearningQueueWrapper(_tgp_params -> samples_to_save);
        const bayesopt::Dataset* data    = model -> getData();

        for (uint index = 0; index < data -> getNSamples(); index += 1)
        {
            samples.push(data -> getSampleX(index), data -> getSampleY(index));
        }

        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
        _output << "Samples(:,:," << test_index + 1 << ") = [" << std::endl;
        _output.close();

        // Write samples
        for (uint index = 0; index < samples.size(); index += 1)
        {
            _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
            _output << std::setw(11) << -samples[index].quality;

            for(uint dim_index = 0; dim_index < _func -> dim; dim_index += 1)
            {
                _output << ", " << std::setw(11) << samples[index].query(dim_index);
            }

            _output << ";" << std::endl;
            _output.close();
        }

        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
        _output << "];" << std::endl << std::endl;
        _output.close();
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: printTestHeader                                                                  *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
void LogManager::printCurrIter(uint iteration, LearningQueueWrapper& results, bayesopt::PosteriorModel* model, double y_prev)
{
    if (iteration == 0) return;

    if ( ((_type & LOG_POSTERIOR) != 0 ) && (_func -> dim == 1) )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/posterior_";

        // Create axis
        std::vector<double>      axis;
                                 axis.push_back( 0.0);
                                 axis.push_back( 1.0);
                                 axis.push_back(_func -> ymin);
                                 axis.push_back(_func -> ymax);
        const bayesopt::Dataset* data = model -> getData();
        std::string              title;

        // Update file path
        if((*_isTGP))
        {
            filename  += std::string("tgp_");
            title      = std::string("TGP");
        }
        else
        {
            filename  += std::string("gp_");
            title      = std::string("GP");
        }

        filename += boost::lexical_cast<std::string>(iteration);
        filename += std::string(".m");

        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);
        _output << "clear;"           << std::endl;
        _output << "opengl software;" << std::endl;
        _output << "distribution = [" << std::endl;
        _output.close();

        for (vectord query = vectord(1, 0.0); query(0) < 1; query(0) += 0.001)
        {
            bayesopt::ProbabilityDistribution* distribution = model -> getPrediction(query);

            _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

            _output << std::endl
                    << std::setw(11) <<   query(0)                  << ","
                    << std::setw(11) << - distribution -> getMean() << ","
                    << std::setw(11) <<   distribution -> getStd () << ";";
            _output.close();
        }

        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
        _output << std::endl << "];" << std::endl;
        _output << "uei = [" << std::endl;
        _output.close();

        for (vectord query = vectord(1, 0.0); query(0) < 1; query(0) += 0.001)
        {
            bayesopt::UnscentedExpectedImprovement* uei = new bayesopt::UnscentedExpectedImprovement(1);
                                                    uei -> setParameters(_opt_params -> crit_params);

            std::vector<vectord>          xx;
            std::vector<double >          w;
            double                        uei_y   = 0;
            double                        uei_std = 0;

            uei -> getSamples(query, xx, w);

            for (uint uei_sample = 0; uei_sample < xx.size(); uei_sample += 1)
            {
                bayesopt::ProbabilityDistribution* distribution = model -> getPrediction(xx[uei_sample]);

                uei_y   += w[uei_sample] * distribution -> getMean();
                uei_std += w[uei_sample] * distribution -> getStd();
            }

            _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
            _output << std::endl
                    << std::setw(11) <<  query(0) << ","
                    << std::setw(11) <<  uei_y    << ","
                    << std::setw(11) <<  uei_std  << ";";
            _output.close();
        }

        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
        _output << std::endl << "];" << std::endl;
        _output << "data = [\n";
        _output.close();

        for (uint index = 0; index < data -> getNSamples(); index += 1)
        {
            _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
            _output << std::endl
                    << std::setw(11) <<  data -> getSampleX(index)(0) << ","
                    << std::setw(11) << -data -> getSampleY(index)    << ";";
            _output.close();
        }

        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
        _output << std::endl << "];" << std::endl;
        _output << std::endl;
        _output << std::endl << "real_func = [";
        _output.close();

        for (double query = _func -> lower_bound[0]; query <= _func -> upper_bound[0]; query += ((_func -> upper_bound[0] - _func -> lower_bound[0]) * 0.001))
        {
            _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
            _output << std::endl
                    << std::setw(11) <<   query                                << ","
                    << std::setw(11) << - _func -> evaluate(vectord(1, query)) << ";";
            _output.close();
        }

        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
        _output << std::endl << "];" << std::endl;
        _output << std::endl << "criteria = [";
        _output.close();

        for (double query = _func -> lower_bound[0]; query <= _func -> upper_bound[0]; query += ((_func -> upper_bound[0] - _func -> lower_bound[0]) * 0.001))
        {
            _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
            _output << std::endl
                    << std::setw(11) <<   query                                         << ","
                    << std::setw(11) << - model -> evaluateCriteria(vectord(1, query)) << ";";
            _output.close();
        }

        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
        _output << std::endl << "];" << std::endl;

        _output << std::endl << "distribution(:,4) = distribution(:,2) - 2 * distribution(:,3);";
        _output << std::endl << "distribution(:,5) = distribution(:,3) * 4;";
        _output << std::endl;
        _output << std::endl << "h1 = figure(size(data,1));";
        _output << std::endl << "hold on";
        _output << std::endl;
        _output << std::endl << "h2 = area(distribution(:,1),distribution(:,4:5));";
        _output << std::endl << "set(h2(1),'FaceColor',[1.0 1.0 1.0]);";
        _output << std::endl << "set(h2(2),'FaceColor',[0.2 0.2 0.2]);";
        _output << std::endl << "alpha(0.1);";
        _output << std::endl;
        _output << std::endl << "plot(distribution(:,1),distribution(:,2),        'LineWidth', 1.3)";
        _output << std::endl << "plot(   real_func(:,1),   real_func(:,2), 'm-.', 'LineWidth', 1.3)";
        _output << std::endl << "axis([" << axis[0] << ", " << axis[1] << ", " << axis[2] << ", " << axis[3] << "]);";
        _output << std::endl << "title('" << title.c_str() << "');";
        _output << std::endl << "xlabel('x')";
        _output << std::endl << "ylabel('y')";
        _output << std::endl;
        _output << std::endl << "scatter(data(:,1), data(:,2), 'filled', 'SizeData', 50)";
        _output << std::endl << "max_index = find(data(:,2) == max(data(:,2)));";
        _output << std::endl << "scatter(data(end,1), data(end,2), 'MarkerEdgeColor', [1 0 0], 'MarkerFaceColor',[1 0 0], 'LineWidth',3);";

        if (dynamic_cast<bayesopt::TreedGaussianProcess*>(model) != NULL)
        {
            std::vector<double> splits = dynamic_cast<bayesopt::TreedGaussianProcess*>(model) -> getSplits();

            if (splits.size() > 0)
            {
                _output << std::endl;

                for (uint index = 0; index < splits.size(); index += 1)
                {
                    _output << std::endl << "line([" << splits[index] << " " << splits[index] << "], [" << axis[2] << " " << axis[3] << "], 'LineStyle', '--', 'Color', [1.0 0.0 0.0]);";
                }
            }
        }

        _output << std::endl << "hold off";
        _output << std::endl << "h2 = figure(10*size(data,1));";
        _output << std::endl << "plot(criteria(:,1), criteria(:,2), 'LineWidth', 1.3)";

        _output << std::endl << "imagename = '" << title.c_str() << "_';";
        _output << std::endl << "imagename = strcat(imagename, num2str(size(data,1)));";
        _output << std::endl << "print(h1, imagename,'-dpng');";
        _output << std::endl << "close(h1)";
        _output << std::endl << "print(h2, strcat(imagename,'_criteria'),'-dpng');";
        _output << std::endl << "close(h2)";
        _output.close();
    }

    if ( ((_type & LOG_LEARNING) != 0) &&
         ((_mode & METRIC_OPT  ) != 0)    )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/learning.m";

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

        // Write to file
        _output << "            ";
        _output << iteration << ", ";
        _output << results.toString();
        _output << ", " << -y_prev;

        if ((*_isTGP))
        {
            bayesopt::TreedGaussianProcess* tgp_model = dynamic_cast<bayesopt::TreedGaussianProcess*>(model);

            if (tgp_model != NULL)
            {
                _output << ", " << tgp_model -> getNumberOfLeafs();
            }

        }
        _output << ";" << std::endl;

        // Close file
        _output.close();
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: printTestHeader                                                                  *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
void LogManager::printCurrIterMonteCarlo(const uint iteration, const vectord query, const double y, const uint optimum, const uint points, const double std_dev)
{
    if (iteration == 0) return;

    if ( ((_type & LOG_LEARNING   ) != 0) &&
         ((_mode & MONTE_CARLO_OPT) != 0)    )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/mc_learning.m";

        std::vector<vectord>           xx;
        std::vector<double>            yy;
        double                         y_mean;
        double                         y_std;
        std::vector<double>::iterator  y_worst;

        if (points > 1)
        {
            getMonteCarloPoints(points, query, std_dev, xx, yy);

            std::vector<double>  yy_diff(yy.size());

            // Calculate y_mean
            y_mean = std::accumulate(yy.begin(), yy.end(), 0.0) / yy.size();

            // Calculate y_std
            std::transform(yy.begin(), yy.end(), yy_diff.begin(), std::bind2nd(std::minus<double>(), y_mean));

            y_std   = std::sqrt(std::inner_product(yy_diff.begin(), yy_diff.end(), yy_diff.begin(), 0.0) / (yy.size() - 1));

            // Calculate y_min
            y_worst = std::max_element(yy.begin(), yy.end());
        }
        else
        {
            xx.push_back(query);
            yy.push_back(y);

            y_mean  = y;
            y_std   = 0;
            y_worst = (std::vector<double>::iterator) &yy[0];
        }

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

        // Write to file
        _output << std::setw(11) << iteration  << ", ";
        _output << std::setw(11) << optimum    << ", ";
        _output << std::setw(11) << y          << ", ";
        _output << std::setw(11) << y_mean     << ", ";
        _output << std::setw(11) << y_std      << ", ";
        _output << std::setw(11) << (*y_worst) << "; " << std::endl;

        // for (uint sample = 0; sample < yy.size(); sample += 1)
        // {
        //     if (sample == (yy.size() - 1)) _output << std::setw(11) << yy[sample] << ";" << std::endl;
        //     else                           _output << std::setw(11) << yy[sample] << ", ";
        // }

        // Close file
        _output.close();
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: printCurrIterUBO                                                                 *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
void LogManager::printCurrIterUBO(const uint iteration, const vectord query, const double y, const uint optimum, const double uns_mu, const double uns_std)
{
    if (iteration == 0) return;

    if ( ((_type & LOG_LEARNING ) != 0) &&
         ((_mode & UNSCENTED_OPT) != 0)    )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/uns_learning.m";

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

        // Write to file
        _output << std::setw(11) << iteration  << ", ";
        _output << std::setw(11) << optimum    << ", ";
        _output << std::setw(11) << y          << ", ";
        _output << std::setw(11) << uns_mu     << ", ";
        _output << std::setw(11) << uns_std    << ", " << std::endl;

        // Close file
        _output.close();
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: printLogFooter                                                                   *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
void LogManager::printLogFooter(uint iteration)
{
    if ( ((_type & LOG_POSTERIOR     ) != 0) &&
         ((_mode & CREATE_MATLAB_FIGS) != 0)    ) callMatlab(iteration);

    if ( ((_type & LOG_LEARNING)       != 0) &&
         ((_mode & METRIC_OPT  )       != 0)    )
    {
        std::string filename = std::string(_path_folder.c_str()) + "/learning.m";

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

        // Write to file
        _output << "Test = removeOutliers(Test);"                                                              << std::endl;
        _output <<                                                                                                std::endl;
        _output << "Result.name       = strcat(surrogate, '_', optimizable, '_', learning_criteria);"          << std::endl;
        _output <<                                                                                                std::endl;
        _output << "if strcmp(surrogate, 'tgp')"                                                               << std::endl;
        _output << "      Result.name = strcat(Result.name, '_l', num2str(min_data_per_leaf), '_w', num2str(wheight_power), '_t', num2str(wheight_threshold));" << std::endl;
        _output << "end"                                                                                       << std::endl;
        _output <<                                                                                                std::endl;
        _output << "Result.Avg = zeros(size(Test(:,:,1)));"                                                    << std::endl;
        _output << "for index = 1:size(Test,3)"                                                                << std::endl;
        _output << "    Result.Avg = Result.Avg + Test(:,:,index);"                                            << std::endl;
        _output << "end"                                                                                       << std::endl;
        _output << "Result.Avg = Result.Avg / size(Test,3);"                                                   << std::endl;
        _output <<                                                                                                std::endl;
        _output << "Result.Dev = zeros(size(Test(:,:,1)));"                                                    << std::endl;
        _output << "for index1 = 1:size(Test,1)"                                                               << std::endl;
        _output << "    for index2 = 2:size(Test,2)"                                                           << std::endl;
        _output << "        for index3 = 1:size(Test,3)"                                                       << std::endl;
        _output << "            Result.Dev(index1,index2) = Result.Dev(index1,index2) + ( Test(index1,index2,index3) - Result.Avg(index1,index2) )^2;" << std::endl;
        _output << "        end"                                                                               << std::endl << std::endl;
        _output << "        Result.Dev(index1,index2) = sqrt(Result.Dev(index1,index2) / (size(Test,3) - 1));" << std::endl;
        _output << "    end"                                                                                   << std::endl;
        _output << "end"                                                                                       << std::endl;
        _output                                                                                                << std::endl;
        _output << "if size(Test,3) > 1"                                                                       << std::endl;
        _output << "    yy_min = min(Result.Avg(:,2) - 2*Result.Dev(:,2));"                                    << std::endl;
        _output << "    yy_max = max(Result.Avg(:,2) + 2*Result.Dev(:,2));"                                    << std::endl;
        _output << "else"                                                                                      << std::endl;
        _output << "    yy_min = min(Result.Avg(:,2));"                                                        << std::endl;
        _output << "    yy_max = max(Result.Avg(:,2));"                                                        << std::endl;
        _output << "end"                                                                                       << std::endl;
        _output <<                                                                                                std::endl;
        _output << "y_min  = (yy_min - yy_max) * 0.10 + yy_min;"                                               << std::endl;
        _output << "y_max  = (yy_max - yy_min) * 0.10 + yy_max;"                                               << std::endl;
        _output <<                                                                                                std::endl;
        _output << "opengl software;"                                                                          << std::endl;
        _output << "%% PLOTTING - 1st Best"                                                                    << std::endl;
        _output << "h11 = figure(100);"                                                                        << std::endl;
        _output << "hold on"                                                                                   << std::endl;
        _output << "grid on"                                                                                   << std::endl;
        _output << "h2 = area(Result.Avg(:,1), [Result.Avg(:,2) - 2*Result.Dev(:,2) 4*Result.Dev(:,2)]);"      << std::endl;
        _output << "set(h2(1),'FaceColor',[1.0 1.0 1.0]);"                                                     << std::endl;
        _output << "set(h2(2),'FaceColor',[0.2 0.2 0.2]);"                                                     << std::endl;
        _output << "alpha(0.1);"                                                                               << std::endl;
        _output << "plot(Result.Avg(:,1), Result.Avg(:,2), 'LineWidth', 1.3);"                                 << std::endl;
        _output << "xlabel('Iterations');"                                                                     << std::endl;
        _output << "ylabel('Output');"                                                                         << std::endl;
        _output << "title (strcat('1st Best Grasp'));"                                                         << std::endl;
        _output << "axis([1,max_iter,y_min, y_max]);"                                                          << std::endl;
        _output << "hold off"                                                                                  << std::endl;
        _output <<                                                                                                std::endl;
        _output << "%% PLOTTING - 2nd Best"                                                                    << std::endl;
        _output << "h12 = figure(101);"                                                                        << std::endl;
        _output << "hold on"                                                                                   << std::endl;
        _output << "grid on"                                                                                   << std::endl;
        _output << "h2 = area(Result.Avg(:,1), [Result.Avg(:,3) - 2*Result.Dev(:,3) 4*Result.Dev(:,3)]);"      << std::endl;
        _output << "set(h2(1),'FaceColor',[1.0 1.0 1.0]);"                                                     << std::endl;
        _output << "set(h2(2),'FaceColor',[0.2 0.2 0.2]);"                                                     << std::endl;
        _output << "alpha(0.1);"                                                                               << std::endl;
        _output << "plot(Result.Avg(:,1), Result.Avg(:,3), 'LineWidth', 1.3);"                                 << std::endl;
        _output << "xlabel('Iterations');"                                                                     << std::endl;
        _output << "ylabel('Output');"                                                                         << std::endl;
        _output << "title (strcat('2nd Best Grasp'));"                                                         << std::endl;
        _output << "axis([1,max_iter,y_min, y_max]);"                                                          << std::endl;
        _output << "hold off"                                                                                  << std::endl;
        _output <<                                                                                                std::endl;
        _output << "%% PLOTTING - 3rd Best"                                                                    << std::endl;
        _output << "h13 = figure(102);"                                                                        << std::endl;
        _output << "hold on"                                                                                   << std::endl;
        _output << "grid on"                                                                                   << std::endl;
        _output << "h2 = area(Result.Avg(:,1), [Result.Avg(:,4) - 2*Result.Dev(:,4) 4*Result.Dev(:,4)]);"      << std::endl;
        _output << "set(h2(1),'FaceColor',[1.0 1.0 1.0]);"                                                     << std::endl;
        _output << "set(h2(2),'FaceColor',[0.2 0.2 0.2]);"                                                     << std::endl;
        _output << "alpha(0.1);"                                                                               << std::endl;
        _output << "plot(Result.Avg(:,1), Result.Avg(:,4), 'LineWidth', 1.3);"                                 << std::endl;
        _output << "xlabel('Iterations');"                                                                     << std::endl;
        _output << "ylabel('Output');"                                                                         << std::endl;
        _output << "title (strcat('3rd Best Grasp'));"                                                         << std::endl;
        _output << "axis([1,max_iter,y_min, y_max]);"                                                          << std::endl;
        _output << "hold off"                                                                                  << std::endl;
        _output <<                                                                                                std::endl;
        _output << "%% PLOTTING - 4th Best"                                                                    << std::endl;
        _output << "h14 = figure(103);"                                                                        << std::endl;
        _output << "hold on"                                                                                   << std::endl;
        _output << "grid on"                                                                                   << std::endl;
        _output << "h2 = area(Result.Avg(:,1), [Result.Avg(:,5) - 2*Result.Dev(:,5) 4*Result.Dev(:,5)]);"      << std::endl;
        _output << "set(h2(1),'FaceColor',[1.0 1.0 1.0]);"                                                     << std::endl;
        _output << "set(h2(2),'FaceColor',[0.2 0.2 0.2]);"                                                     << std::endl;
        _output << "alpha(0.1);"                                                                               << std::endl;
        _output << "plot(Result.Avg(:,1), Result.Avg(:,5), 'LineWidth', 1.3);"                                 << std::endl;
        _output << "xlabel('Iterations');"                                                                     << std::endl;
        _output << "ylabel('Output');"                                                                         << std::endl;
        _output << "title (strcat('4th Best Grasp'));"                                                         << std::endl;
        _output << "axis([1,max_iter,y_min, y_max]);"                                                          << std::endl;
        _output << "hold off"                                                                                  << std::endl;
        _output <<                                                                                                std::endl;
        _output << "%% PLOTTING - 5th Best"                                                                    << std::endl;
        _output << "h15 = figure(104);"                                                                        << std::endl;
        _output << "hold on"                                                                                   << std::endl;
        _output << "grid on"                                                                                   << std::endl;
        _output << "h2 = area(Result.Avg(:,1), [Result.Avg(:,6) - 2*Result.Dev(:,6) 4*Result.Dev(:,6)]);"      << std::endl;
        _output << "set(h2(1),'FaceColor',[1.0 1.0 1.0]);"                                                     << std::endl;
        _output << "set(h2(2),'FaceColor',[0.2 0.2 0.2]);"                                                     << std::endl;
        _output << "alpha(0.1);"                                                                               << std::endl;
        _output << "plot(Result.Avg(:,1), Result.Avg(:,6), 'LineWidth', 1.3);"                                 << std::endl;
        _output << "xlabel('Iterations');"                                                                     << std::endl;
        _output << "ylabel('Output');"                                                                         << std::endl;
        _output << "title (strcat('5h Best Grasp'));"                                                          << std::endl;
        _output << "axis([1,max_iter,y_min, y_max]);"                                                          << std::endl;
        _output << "hold off"                                                                                  << std::endl;
        _output <<                                                                                                std::endl;
        _output << "%% PLOTTING - PrevIter"                                                                    << std::endl;
        _output << "h16 = figure(105);"                                                                        << std::endl;
        _output << "hold on"                                                                                   << std::endl;
        _output << "grid on"                                                                                   << std::endl;
        _output << "h2 = area(Result.Avg(:,1), [Result.Avg(:,7) - 2*Result.Dev(:,7) 4*Result.Dev(:,7)]);"      << std::endl;
        _output << "set(h2(1),'FaceColor',[1.0 1.0 1.0]);"                                                     << std::endl;
        _output << "set(h2(2),'FaceColor',[0.2 0.2 0.2]);"                                                     << std::endl;
        _output << "alpha(0.1);"                                                                               << std::endl;
        _output << "plot(Result.Avg(:,1), Result.Avg(:,7), 'LineWidth', 1.3);"                                 << std::endl;
        _output << "xlabel('Iterations');"                                                                     << std::endl;
        _output << "ylabel('Output');"                                                                         << std::endl;
        _output << "title (strcat('Sample'));"                                                                 << std::endl;
        _output << "axis([1,max_iter,min(Result.Avg(:,7)), max(Result.Avg(:,7))]);"                            << std::endl;
        _output << "hold off"                                                                                  << std::endl;
        _output << "print(h16, 'yprev','-dpng');"                                                              << std::endl;
        _output << "close(h16)"                                                                                << std::endl;
        _output <<                                                                                                std::endl;
        if ((*_isTGP))
        {
            _output << "%% PLOTTING - Number of Leafs"                                                             << std::endl;
            _output << "h17 = figure(106);"                                                                        << std::endl;
            _output << "hold on"                                                                                   << std::endl;
            _output << "grid on"                                                                                   << std::endl;
            _output << "h2 = area(Result.Avg(:,1), [Result.Avg(:,8) - 2*Result.Dev(:,8) 4*Result.Dev(:,8)]);"      << std::endl;
            _output << "set(h2(1),'FaceColor',[1.0 1.0 1.0]);"                                                     << std::endl;
            _output << "set(h2(2),'FaceColor',[0.2 0.2 0.2]);"                                                     << std::endl;
            _output << "alpha(0.1);"                                                                               << std::endl;
            _output << "plot(Result.Avg(:,1), Result.Avg(:,8), 'LineWidth', 1.3);"                                 << std::endl;
            _output << "xlabel('Iterations');"                                                                     << std::endl;
            _output << "ylabel('Number of Leafs');"                                                                << std::endl;
            _output << "title (strcat('TGP - Number of Leafs'));"                                                  << std::endl;
            _output << "axis([1,max_iter,1, max(Result.Avg(:,8))]);"                                               << std::endl;
            _output << "hold off"                                                                                  << std::endl;
            _output <<                                                                                                std::endl;
            _output << "print(h17, 'tgp_numberofleafs','-dpng');"                                                  << std::endl;
            _output << "close(h17)"                                                                                << std::endl;

        }
        _output << "print(h11, '1stbest','-dpng');"                                                            << std::endl;
        _output << "close(h11)"                                                                                << std::endl;
        _output << "print(h12, '2ndbest','-dpng');"                                                            << std::endl;
        _output << "close(h12)"                                                                                << std::endl;
        _output << "print(h13, '3rdbest','-dpng');"                                                            << std::endl;
        _output << "close(h13)"                                                                                << std::endl;
        _output << "print(h14, '4thbest','-dpng');"                                                            << std::endl;
        _output << "close(h14)"                                                                                << std::endl;
        _output << "print(h15, '5thbest','-dpng');"                                                            << std::endl;
        _output << "close(h15)"                                                                                << std::endl;
        _output <<                                                                                                std::endl;
        _output << "filename = char(strcat(surrogate, '_', optimizable, '.mat'));"                             << std::endl;
        _output << "save(char(filename), 'Result');"                                                           << std::endl;

        // Close file
        _output.close();
    }
    if ( ((_type & LOG_LEARNING) != 0) &&
         ((_mode & SAMPLES     ) != 0)    )
    {
        std::string filename = std::string(_path_folder.c_str()) + + "/samples.m";

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);
        _output << "Result = [];"                                                                   << std::endl;
        _output << "for index = 1:size(Samples,3)"                                                  << std::endl;
        _output << "    Result         = [ Result; Samples(size(Samples,1)-n_samples+1:end,:,1) ];" << std::endl;
        _output << "    Samples(:,:,1) = [];"                                                       << std::endl;
        _output << "end"                                                                            << std::endl << std::endl;
        _output << "Result( find(Result(:,1) < -10), :) = [];"                                      << std::endl << std::endl;
        _output << "bin_step  = 0.1;"                                                               << std::endl;
        _output << "binranges = (min(Result(:,1))-bin_step):bin_step:(max(Result(:,1))+bin_step);"  << std::endl;
        _output << "bincounts = histc(Result(:,1),binranges);"                                      << std::endl;
        _output << "bincounts = bincounts / sum(bincounts);"                                        << std::endl << std::endl;
        _output << "figure(1)"                                                                      << std::endl;
        _output << "hist1 = bar(binranges,bincounts,'histc');"                                      << std::endl;
        _output << "set(hist1,'FaceColor',[0.0 0.5 0.9])"                                           << std::endl;
        _output << "set(hist1,'EdgeColor',[0.0 0.0 0.0])"                                           << std::endl;
        _output << "set(hist1,'LineWidth',1.5          )"                                           << std::endl;
        _output << "xlabel('query')"                                                                << std::endl;
        _output << "ylabel('occurrence (%)')"                                                       << std::endl;
        _output << "title('Function Profile')"                                                      << std::endl;
        _output << "grid on"                                                                        << std::endl;
        _output << "%%"                                                                             << std::endl;
        _output << "number_of_clusters = 20;"                                                       << std::endl;
        _output << "number_of_tries    = 100;"                                                      << std::endl;
        _output << "cluster_plot = zeros(number_of_clusters-1,2);"                                  << std::endl;
        _output << "for tries = 1:number_of_tries"                                                  << std::endl;
        _output << "    for num_c = 2:number_of_clusters"                                           << std::endl;
        _output << "        message = strcat('Try =', num2str(tries), '; Cluter =', num2str(num_c));" << std::endl << std::endl;
        _output << "        try"                                                                    << std::endl;
        _output << "            idx                       = kmeans(Result(:,2:end), num_c);"        << std::endl;
        _output << "            s                         = silhouette(Result(:,2:end), idx);"      << std::endl;
        _output << "            cluster_plot(num_c - 1,:) = cluster_plot(num_c - 1,:) + [num_c, sum(s)/size(s,1)];" << std::endl;
        _output << "        catch ME"                                                               << std::endl;
        _output << "            cluster_plot(num_c - 1,:) = cluster_plot(num_c - 1,:) + [num_c, 0];" << std::endl;
        _output << "            message = strcat(message, '; ERROR;');"                             << std::endl;
        _output << "        end"                                                                    << std::endl << std::endl;
        _output << "        disp(message);"                                                         << std::endl;
        _output << "    end"                                                                        << std::endl;
        _output << "end"                                                                            << std::endl;
        _output << "cluster_plot = cluster_plot / number_of_tries;"                                 << std::endl << std::endl;
        _output << "figure(2)"                                                                      << std::endl;
        _output << "plot(cluster_plot(:,1), cluster_plot(:,2), 'r*-.');"                            << std::endl << std::endl;
        _output << "number_of_clusters = cluster_plot(find(cluster_plot(:,2) == max(cluster_plot(:,2))),1)"      << std::endl;
        _output << "%number_of_clusters = 4"                                                        << std::endl << std::endl;
        _output << "face_colors{1} = [0.8 0.2 0.2];"                                                << std::endl;
        _output << "face_colors{2} = [0.0 0.5 0.9];"                                                << std::endl;
        _output << "face_colors{3} = [0.0 0.9 0.5];"                                                << std::endl;
        _output << "face_colors{4} = [1.0 0.0 1.0];"                                                << std::endl;
        _output << "face_colors{5} = [0.0 1.0 1.0];"                                                << std::endl;
        _output << "face_colors{6} = [1.0 1.0 0.0];"                                                << std::endl << std::endl;
        _output << "[idx,C,sumd,D] = kmeans(Result(:,2:end), number_of_clusters);"                  << std::endl << std::endl;
        _output << "total_bincounts = [];"                                                          << std::endl;
        _output << "for index = 1:number_of_clusters"                                               << std::endl;
        _output << "    bincounts = histc(Result(find(idx == index),1),binranges);"                 << std::endl;
        _output << "    total_bincounts = [total_bincounts; bincounts'];"                           << std::endl;
        _output << "end"                                                                            << std::endl;
        _output << "total_bincounts = total_bincounts / sum(sum(total_bincounts));"                 << std::endl << std::endl;
        _output << "fig_hand = figure(3);"                                                          << std::endl;
        _output << "hist1 = bar(binranges',total_bincounts', 'stacked');"                           << std::endl;
        _output << "colormap(summer);"                                                              << std::endl;
        _output << "xlabel('learning metric')"                                                      << std::endl;
        _output << "ylabel('occurrence (%)')"                                                       << std::endl;
        _output << "%title('TGP - Waterbottle - TOLE - cBEI')"                                      << std::endl;
        _output << "%axis([0.2 0.8 0 0.25])"                                                        << std::endl;
        _output << "set(fig_hand, 'Position', [100, 100, 800, 600]);"                               << std::endl;
        _output << "grid on"                                                                        << std::endl << std::endl;
        _output << "legend_text =[];"                                                               << std::endl;
        _output << "for index = 1:size(total_bincounts,1)"                                          << std::endl;
        _output << "    legend_text{index} = horzcat(num2str(sum(total_bincounts(index,:))), ' %  >>  c = [');" << std::endl;
        _output << "    for dim = 1:size(C,2)"                                                      << std::endl;
        _output << "        legend_text{index} = horzcat(legend_text{index}, num2str(C(index, dim) ), ' , ');"    << std::endl;
        _output << "    end"                                                                        << std::endl;
        _output << "    legend_text{index} = horzcat(legend_text{index}, ']');"                      << std::endl;
        _output << "end"                                                                            << std::endl;
        _output << "legend(legend_text)"                                                            << std::endl << std::endl;

        // Close file
        _output.close();
    }
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: callMatlab                                                                       *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
void LogManager::callMatlab(uint iteration)
{
    // Create shell string
    std::string command  = std::string("sudo /usr/local/MATLAB/R2013a/bin/matlab -nosplash -nodesktop -r \"cd ");
                command += std::string(_path_folder.c_str());
                command += std::string("; ");

    for (uint index = 1; index <= iteration; index += 1)
    {
        if ((*_isTGP)) command += std::string("posterior_tgp_") + boost::lexical_cast<std::string>(index) + std::string("; ");
        else           command += std::string("posterior_gp_")  + boost::lexical_cast<std::string>(index) + std::string("; ");
    }

    command += std::string("exit; \"");

    // Call shell command
    system(command.c_str());
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: printFuncProfile                                                                 *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
void LogManager::printFuncProfile(uint points)
{
    if ( (_type & LOG_OBJ_FUNCTION) != 0 )
    {
        if (_func -> dim > 2) return;

        std::string filename = std::string(_path_folder.c_str()) + "/func_profile.m";

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::trunc);

        // Write to file
        _output << "clear;"                                                      << std::endl;
        _output << "logtype           = 'Objective Function Profile';"           << std::endl;

        _output << "optimizable       = '" << _func -> name                   << "';" << std::endl;
        _output << "dims              = "  << _func -> dim                    <<  ";" << std::endl;

        if ( (_tgp_params != NULL) && (_tgp_params -> others.size() > 0) )
        {
            for (uint index = 0; index < _tgp_params -> others.size(); index += 1)
            {
                _output << _tgp_params -> others[index] << ";" << std::endl;
            }
        }

        _output << std::endl << std::endl;

        _output << "Samples = [";

        // Close file
        _output.close();

        vectord lower_bound, upper_bound, steps;

        _func -> getBoundingBox(lower_bound, upper_bound);

        steps = (upper_bound - lower_bound) / points;

        std::cout << std::endl << "lower_bound = " << lower_bound;
        std::cout << std::endl << "upper_bound = " << upper_bound;
        std::cout << std::endl << "steps       = " << steps;

        switch (_func -> dim)
        {
            case 1 :
            {
                vectord query = lower_bound;

                while (query[0] <= upper_bound[0])
                {
                    double y  = _func -> evaluate(query);

                    std::cout << std::endl << "LogManager::printFuncProfile | y = " << y << " | query = " << query;

                    // Open file
                    _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

                    _output << std::endl
                            << std::setw(11) << y        << ","
                            << std::setw(11) << query[0] << ";";
                    _output.close();

                    query[0] += steps[0];
                }


                break;
            }

            case 2 :
            {
                vectord query = lower_bound;

                while (query[1] <= upper_bound[1])
                {
                    query[0] = lower_bound[0];

                    while (query[0] <= upper_bound[0])
                    {
                        double y  = _func -> evaluate(query);

                        std::cout << std::endl << "LogManager::printFuncProfile | y = " << y << " | query = " << query;

                        // Open file
                        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

                        _output << std::endl
                                << std::setw(11) << y        << ","
                                << std::setw(11) << query[0] << ","
                                << std::setw(11) << query[1] << ";";
                        _output.close();

                        query[0] += steps[0];
                    }

                    query[1] += steps[1];
                }

                break;
            }
        }

        // Open file
        _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

        _output << std::endl << "];";

        // Close file
        _output.close();
    }
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: printFuncProfile                                                                 *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
void LogManager::printMonteCarlo(uint points, const vectord query, double std_dev)
{
    std::vector<double> v_std_dev;

    for (uint index = 0; index < query.size(); index += 1)
    {
        v_std_dev.push_back(std_dev);
    }

    printMonteCarlo(points, query, v_std_dev);
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: printFuncProfile                                                                 *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
void LogManager::printMonteCarlo(uint points, const vectord query, std::vector<double> std_dev)
{
    // Assert Section
    if ( (_type & LOG_OBJ_FUNCTION) == 0           ) return;
    if (   query.size()             != _func -> dim) return;
    if ( std_dev.size()             == 0           ) return;
    if ( std_dev.size()             == 1           )
    {
        while (std_dev.size() != query.size())
        {
            std_dev.push_back(std_dev[0]);
        }
    }

    if (std_dev.size() != query.size()) return;

    // Get Monte Carlo Points
    std::vector<vectord> xx;
    std::vector<double>  yy;

    getMonteCarloPoints(points, query, std_dev, xx, yy);

    double y_mean   = std::accumulate(yy.begin(), yy.end(), 0.0) / yy.size();
    double y_stdev  = std::sqrt(std::inner_product(yy.begin(), yy.end(), yy.begin(), 0.0) / yy.size() - y_mean * y_mean);

    std::string filename = std::string(_path_folder.c_str()) + "/func_montecarlo_sampling.m";

    // Open file
    _output.open(filename.c_str(), std::ofstream::out | std::ofstream::app);

    // Write to file
    _output << "%%"                                                     << std::endl;
    _output << "clear;"                                                 << std::endl;
    _output << "logtype           = 'Monte Carlo Function Sampling';"   << std::endl;

    _output << "optimizable       = '" << _func -> name         << "';" << std::endl;
    _output << "dims              = "  << _func -> dim          <<  ";" << std::endl;

    if ( (_tgp_params != NULL) && (_tgp_params -> others.size() > 0) )
    {
        for (uint index = 0; index < _tgp_params -> others.size(); index += 1)
        {
            _output << _tgp_params -> others[index] << ";" << std::endl;
        }
    }

    _output << std::endl << std::endl;
    _output << "query             = [";

    for (uint index = 0; index < query.size(); index += 1)
    {
        _output << query[index];

        if (index == (query.size() - 1))  _output << "];" << std::endl;
        else                              _output << ", " << std::endl;
    }

    _output << "y_query           = " << _func -> evaluate(query) << ";" << std::endl;
    _output << "std_dev           = [";

    for (uint index = 0; index < query.size(); index += 1)
    {
        _output << std_dev[index];

        if (index == (query.size() - 1))  _output << "];" << std::endl << std::endl;
        else                              _output << ", " << std::endl;
    }

    _output << "y_mean  = " << y_mean  << ";" << std::endl;
    _output << "y_stdev = " << y_stdev << ";" << std::endl << std::endl;
    // _output << "xx      = [" << std::endl;
    //
    // for (uint samples = 0; samples < points; samples += 1)
    // {
    //     for (uint index = 0; index < query.size(); index += 1)
    //     {
    //         _output << std::setw(11) << (xx[samples])[index];
    //
    //         if (index == (query.size() - 1))  _output << ";" << std::endl;
    //         else                              _output << ", ";
    //     }
    // }
    //
    // _output << "];" << std::endl;
    // _output << "yy      = [" << std::endl;
    //
    // for (uint samples = 0; samples < points; samples += 1)
    // {
    //     _output << std::setw(11) << yy[samples] << ";" << std::endl;
    // }
    //
    // _output << "];" << std::endl;

    _output.close();
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getMonteCarloPoints                                                              *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
void LogManager::getMonteCarloPoints(uint points, const vectord query, double std_dev, std::vector<vectord>& xx, std::vector<double>& yy)
{
    std::vector<double> v_std_dev;

    for (uint index = 0; index < query.size(); index += 1)
    {
        v_std_dev.push_back(std_dev);
    }

    getMonteCarloPoints(points, query, v_std_dev, xx, yy);
}


/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: getMonteCarloPoints                                                              *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
void LogManager::getMonteCarloPoints(uint points, const vectord query, std::vector<double> std_dev, std::vector<vectord>& xx, std::vector<double>& yy)
{
    // Create Normal distribution
    std::vector<boost::normal_distribution<double> > noise_distribution;
    boost::mt19937                                   generator;
                                                     generator.seed(std::time(0));
    vectord                                          query_normalzized = query;
                          _func -> normalizeVector(query_normalzized);

    // Update noise distribution according to std_dev
    for(uint index = 0; index < query.size(); index += 1)
    {
        boost::normal_distribution<double> distribution( query_normalzized[index], std_dev[index]);

        noise_distribution.push_back(distribution);
    }

    xx.clear();
    yy.clear();

    for (uint index = 0; index < points; index += 1)
    {
        vectord x_curr(query.size());

        for (uint curr_dim = 0; curr_dim < query.size(); curr_dim += 1)
        {
            x_curr[curr_dim] = (noise_distribution[curr_dim])(generator);

            if ( (x_curr[curr_dim] < 0 ) ||
                 (x_curr[curr_dim] > 1 )   )
            {
                x_curr[curr_dim] = query_normalzized[curr_dim];
            }
        }

        _func -> unnormalizeVector(x_curr);

        xx.push_back(x_curr);
        yy.push_back(_func -> evaluate(x_curr));
    }
}

/**************************************************************************************************
 *  Procecure                                                                                     *
 *                                                                                                *
 *  Description: optimizationToString                                                             *
 *  Class      : LogManager                                                                       *
 **************************************************************************************************/
std::string LogManager::optimizationToString(bool isTGP, TgpParameters tgp_params, Parameters opt_params, TGPOptimizable& func)
{
    std::string s = std::string("");

    if (isTGP) s += "tgp_optimization_";
    else       s +=  "gp_optimization_";

    // Fuction name
    s += std::string(func.name) + "_";

    // Learning Criterion Name
    s += std::string(opt_params.crit_name);

    // TGP parameters
    if (isTGP)
    {
        uint wheight_threshold = (tgp_params.wheight_threshold * 100);

        s += "_w"   + boost::lexical_cast<std::string>((uint)tgp_params.wheight_power    );
        s += "_l"   + boost::lexical_cast<std::string>((uint)tgp_params.min_data_per_leaf);
        s += "_t"   + boost::lexical_cast<std::string>((uint)wheight_threshold           );
    }

    return s;
}

/**
 * Returns Json::Value configuration from file
 *
 * @param  config_path configuration filepath
 *
 * @return             configuration in a Json object
 */
Json::Value LogManager::readJsonConfig(std::string config_path)
{
    std::ifstream input_config(config_path.c_str());
    Json::Reader  json_parser;
    Json::Value   read_config;

    // Verify if config file is open
    if (input_config.is_open())
    {
        // Verify if Json configuration was successfully parsed
        if (!json_parser.parse(input_config, read_config))
        {
            // If not print error message
            std::cout << std::endl << json_parser.getFormattedErrorMessages();
        }
    }
    // Config file could not be opened
    else
    {
        std::cout << std::endl << "Could not open config file with path [" << config_path << "]";
    }

    input_config.close();

    return read_config;
}


/**
 * Creates JsonConfigFile
 *
 * @param config      Json object to be saved
 * @param config_path filepath of json file
 */
void LogManager::createJsonConfigFile(Json::Value config, std::string config_path)
{
    namespace fs = boost::filesystem;

    std::vector<std::string> file_tree;
    std::string              folder_path;
    std::ofstream            config_file;
    Json::StyledWriter       json_writer;

    // Perform string split using the string seperator
    boost::split(file_tree, config_path, boost::is_any_of("/"));

    // Creater path excluding the json file
    for (uint folder = 0; folder < (file_tree.size() - 1); folder += 1)
    {
        folder_path += file_tree[folder];
    }

    fs::path folderpath(folder_path);
    fs::path filepath  (config_path);

    // If folder doesn't exist, create it
    if (!fs::exists(folder_path)) fs::create_directory(folder_path);

    // Write config to file
    config_file.open(config_path.c_str(), std::ofstream::out | std::ofstream::trunc);
    config_file << json_writer.write(config);
    config_file.close();
}

/**
 * Converts logmode(uint) to string
 *
 * @param  logmode
 *
 * @return
 */
std::string LogManager::logmodeToString(uint logmode)
{
    std::vector<std::string> result;

    if ( (LOG_LEARNING       & logmode) != 0) result.push_back("LOG_LEARNING");
    if ( (LOG_POSTERIOR      & logmode) != 0) result.push_back("LOG_POSTERIOR");
    if ( (LOG_OBJ_FUNCTION   & logmode) != 0) result.push_back("LOG_OBJ_FUNCTION");

    if ( (METRIC_OPT         & logmode) != 0) result.push_back("METRIC_OPT");
    if ( (CREATE_MATLAB_FIGS & logmode) != 0) result.push_back("CREATE_MATLAB_FIGS");
    if ( (MONTE_CARLO_OPT    & logmode) != 0) result.push_back("MONTE_CARLO_OPT");
    if ( (UNSCENTED_OPT      & logmode) != 0) result.push_back("UNSCENTED_OPT");

    return boost::algorithm::join(result, ",");
};


/**
 * Converts string to logmode(uint)
 *
 * @param  logmode String that describes logmode
 *
 * @return         uint   that describes logmdoe
 */
uint LogManager::stringToLogmode(std::string logmode)
{
    uint                     result      = 0;
    std::vector<std::string> logmode_vec;

    boost::split(logmode_vec, logmode, boost::is_any_of(","));

    for (uint index = 0; index < logmode_vec.size(); index += 1)
    {
        if      (logmode_vec[index].compare("LOG_LEARNING"      ) == 0) result |= LOG_LEARNING;
        else if (logmode_vec[index].compare("LOG_POSTERIOR"     ) == 0) result |= LOG_POSTERIOR;
        else if (logmode_vec[index].compare("LOG_OBJ_FUNCTION"  ) == 0) result |= LOG_OBJ_FUNCTION;

        else if (logmode_vec[index].compare("METRIC_OPT"        ) == 0) result |= METRIC_OPT;
        else if (logmode_vec[index].compare("CREATE_MATLAB_FIGS") == 0) result |= CREATE_MATLAB_FIGS;
        else if (logmode_vec[index].compare("MONTE_CARLO_OPT"   ) == 0) result |= MONTE_CARLO_OPT;
        else if (logmode_vec[index].compare("UNSCENTED_OPT"     ) == 0) result |= UNSCENTED_OPT;
    }

    return result;
}
