#include <boost/program_options.hpp>

#include <json/json.h>

#include <tgpoptimizable.hpp>
#include <criteria/criteria_uei.hpp>

#include <TGPOptimization.hpp>
#include <OptFuncs.hpp>

#include <LogManager.hpp>
#include <OptimizablesManager.hpp>

using namespace bayesopt;

/**
 * Global Variables
 */
std::string str_config_filepath("settings/opt_cfg.json");
bool        use_tgp     = false;
uint        log_mode    = LOG_LEARNING | MONTE_CARLO_OPT;
uint        total_tests = 1;
uint        test_idx    = 0;



/**
 * Argument Parser function
 *
 * @param argc number of arguments
 * @param argv arguments
 */
void parse_options(int argc, char *argv[])
{
    try
    {
        namespace po = boost::program_options;

        po::options_description desc("Options");
                                desc.add_options()
                                                ("help,h"                                         , "Print help messages")
                                                (",i"                                             , "Test index.\nFrom 0 to n.\nDefault 0.")
                                                (",n"                                             , "Number of tests for the current batch.\nDefault 1.")
                                                ("create_default_config", po::value<std::string>(), "Creates templeate Json config file.\nIf arg is specified, arg represents the name of the function to be optimized");

        po::variables_map vm;

        try
        {
            po::store(po::parse_command_line(argc, argv, desc), vm);

            if (vm.count("help"))
            {
                std::cout << "iCubOptimization C++ Application" << std::endl
                          << desc << std::endl;

                exit(0);
            }

            if (vm.count("create_default_config"))
            {
                std::string func_name = vm["create_default_config"].as<std::string>();

                if (func_name.empty())
                {
                    func_name = std::string("GramacyExponential");
                }

                TGPOptimizablePtr func   = bayesopt::TGPOptimizableUtils::getOptimizable(func_name);
                Json::Value       config;
                Parameters        opt_param;
                TgpParameters     tgp_param;

                func -> getOptParams(tgp_param, opt_param);

                config["TGPOptimizable"] = func ->   getJson();
                config["Parameters"    ] = opt_param.getJson();
                config["TgpParameters" ] = tgp_param.getJson();
                config["LogMode"       ] = Json::Value(LogManager::logmodeToString(log_mode));
                config["useTGP"]         = Json::Value(use_tgp);

                LogManager::createJsonConfigFile(config, str_config_filepath);

                std::cout << std::endl << "Create Json Config File in: [" << str_config_filepath << "]" << std::endl;

                exit(0);
            }

            if (vm.count("n"))
            {
                total_tests = vm["n"].as<uint>();
            }

            if (vm.count("i"))
            {
                test_idx = vm["i"].as<uint>();

                if (test_idx >= total_tests)
                {
                    std::cout << std::endl << "Test index out of bounds. Must be lower than " << total_tests << std::endl;

                    exit(-1);
                }
            }

            po::notify(vm);
        }
        catch(po::error& e)
        {
            std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
            std::cerr << desc << std::endl;

            exit(-1);
        }
    }
    catch(std::exception& e)
    {
        std::cerr << "Unhandled Exception reached the top of main: "
                  << e.what() << ", application will now exit" << std::endl;

        exit(-1);
    }
}


/**
 * Main program of iCubSimulator
 *
 * @param  argc  Number of arguments
 * @param  argv  Arguments
 *
 * @return      Program exit status
 */
int main (int argc, char *argv[])
{
    parse_options(argc, argv);

    // Read Json File configuration
    Json::Value read_config = LogManager::readJsonConfig(str_config_filepath);

    // Check if it was loaded a TGPOptimizableUtils
    if (read_config["TGPOptimizable"].isNull())
    {
        std::cout << std::endl << "TGPOptimizable was not loaded from config file";
        std::cout << std::endl << "Terminating program" << std::endl;;

        exit(-1);
    }

    // Load TGPOptimizable from configuration
    TGPOptimizablePtr func = bayesopt::TGPOptimizableUtils::getOptimizable(read_config["TGPOptimizable"]);

    if (!func)
    {
        std::cout << std::endl << "TGPOptimizable was incorrect";
        std::cout << std::endl << "Terminating program" << std::endl;;

        exit(-1);
    }

    // Load Default Optimization parameters
    Parameters     opt_param;
    TgpParameters  tgp_param;

    func -> getOptParams(tgp_param, opt_param);

    // Load Parameters from json config file
    if (!read_config["Parameters"].isNull())
    {
        opt_param.loadJson(read_config["Parameters"]);
        std::cout << std::endl << "Loaded Parameters";
    }

    if (!read_config["TgpParameters"].isNull())
    {
        tgp_param.loadJson(read_config["TgpParameters"]);
        std::cout << std::endl << "Loaded TgpParameters";
    }

    // See if Optimization should use TGP or GP
    if (!read_config["useTGP"].isNull())
    {
        use_tgp = read_config["useTGP"].asBool();
    }

    // Set logging mode
    if (!read_config["logMode"].isNull())
    {
        use_tgp = LogManager::stringToLogmode(read_config["logMode"].asString());
    }

    // Init Optimization
    //   Synthetic functions Optimization
    if (func -> name.compare("iCubOptimizable") != 0)
    {
        std::cout << std::endl << "Initializing Optimization for synthetic function: " << func -> name;

        vectord lower_bound, upper_bound, best_point;

        // Get Exploration Bounding Box
        func -> getBoundingBox(lower_bound, upper_bound);

        if (total_tests != 1)
        {
            opt_param.random_seed = 1 + ( (func -> dim * opt_param.n_init_samples) * test_idx);
        }

        TGPOptimization*                     tgp_opt = new TGPOptimization(tgp_param,
                                                                           opt_param,
                                                                           (*func.get()),
                                                                           log_mode,
                                                                           use_tgp,
                                                                           test_idx);
                                             tgp_opt -> setBoundingBox(lower_bound, upper_bound);
                                             tgp_opt -> optimize(best_point);
        if (test_idx == (total_tests - 1))
        {
            tgp_opt -> printLogFooter();
            tgp_opt -> copyJsonConfig();
        }

        delete tgp_opt;
    }

    std::cout << std::endl << "Optimization Finished successfully" << std::endl;

    return 0;
}
