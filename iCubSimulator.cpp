//#include "Window.h"
#include <tgpoptimizable.hpp>
#include <criteria/criteria_uei.hpp>
#include <param_loader.hpp>
#include <TGPOptimization.hpp>
#include <OptFuncs.hpp>
#include <RKHS.hpp>
#include <LogManager.hpp>
#include <json/json.h>

using bayesopt::utils::ParamLoader;
using namespace bayesopt;

void funcProfile(int argc, char *argv[])
{
    TGPOptimizable* func       = new RKHS();
    Parameters     opt_param;
    TgpParameters  tgp_param;

    func -> getOptParams(tgp_param, opt_param);

    LogManager* logmanager = new LogManager(*func);

    logmanager -> printFuncProfile(1000);
    // logmanager -> printMonteCarlo (1000, vectord(1, 0.077754), 0.01);

    delete logmanager;
    delete func;
}


void synthBayesOpt(uint index, uint tests)
{
    vectord         bestpoint, upper, lower;
    TGPOptimizable* func       = new Martinez();

    Parameters     opt_param;
    TgpParameters  tgp_param;

    func -> getOptParams(tgp_param, opt_param);

    // opt_param.crit_name         = "cUEI";
    // opt_param.crit_params[2]    = 1.00; // scale
    // opt_param.crit_params[3]    = 0.00; // alpha
    // opt_param.n_crit_params     = 2 + 2 + (func -> dim * func -> dim);
    opt_param.n_inner_iterations = 1000;
    tgp_param.wheight_power      =    4;
    // tgp_param.samples_to_save    =   15;

    matrixd px = func -> getUncertaintyMatrix(0.02); // Must be Normalized for montecarlo sampling

    UnscentedExpectedImprovement::convertMatrixToParams(opt_param, px);


    Json::Value        read_config;
    Json::Reader       json_parser;
    Json::StyledWriter json_writer;
    Json::Value params;
                params["Parameters"   ] = opt_param.getJson();
                params["TgpParameters"] = tgp_param.getJson();
                params["func"         ] = func ->   getJson();

    std::ofstream configs;
                  configs.open("bayesopt_saved_beforeLoading.json", std::ofstream::out | std::ofstream::trunc);
                  configs << json_writer.write(params);
                  configs.close();

    std::ifstream input_config("bayesopt_load.json");

    if (json_parser.parse(input_config, read_config))
    {
        std::cout << "Json Parsing Successful" << std::endl;

        if (read_config["TgpParameters"].isNull() != true)
        {
            std::cout << "Loaded TgpParameters" << std::endl;
            tgp_param.loadJson(read_config["TgpParameters"]);
        }


        if (read_config["Parameters"   ].isNull() != true)
        {
            std::cout << "Loaded Parameters" << std::endl;
            opt_param.loadJson(read_config["Parameters"   ]);
        }
    }
    else
    {
        std::cout << json_parser.getFormattedErrorMessages();
    }
    input_config.close();

    params.clear();
    params["Parameters"   ] = opt_param.getJson();
    params["TgpParameters"] = tgp_param.getJson();

    configs.open("bayesopt_saved_afterLoading.json", std::ofstream::out | std::ofstream::trunc);
    configs << json_writer.write(params);
    configs.close();

    std::cout << "noise = " <<                     opt_param.noise  << std::endl;
    std::cout << "noise = " << Json::valueToString(opt_param.noise) << std::endl;
    std::cout << "noise = " <<      std::to_string(opt_param.noise) << std::endl;







    // matrixd px = func -> getUncertaintyMatrix(MC_STD); // Must be Normalized for montecarlo sampling
    // UnscentedExpectedImprovement::convertMatrixToParams(opt_param, px);
    //
    // // Get Exploration Bounding Box
    // func -> getBoundingBox(lower, upper);
    //
    // if (tests != 1)
    // {
    //     opt_param.random_seed = 1 + ( (func -> dim * opt_param.n_init_samples) * index);
    // }
    //
    // // TGPOptimization*            tgp_opt = new TGPOptimization(tgp_param, opt_param, (*func), LOG_POSTERIOR | CREATE_MATLAB_FIGS, false, index);
    // TGPOptimization*            tgp_opt = new TGPOptimization(tgp_param, opt_param, (*func), LOG_LEARNING  | MONTE_CARLO_OPT, true, index);
    //                             tgp_opt -> setBoundingBox (lower, upper);
    //                             tgp_opt -> optimize(bestpoint);
    // if (index == (tests - 1))   tgp_opt -> printLogFooter();
    //
    // delete tgp_opt;
}


// void simoxBayesOpt(uint index, uint tests, int argc, char *argv[])
// {
//     // Variavbles
//     ShowWindow *SimWindow;
//
//     // Initiate Qt
//     SoDB::init();
//     SoQt::init(argc, argv, "iCubSimulator");
//
//     std::cout << "Simulator Started" << std::endl;
//
//     // Simulator Window Initialization
//     // SimWindow = new ShowWindow();
//     SimWindow = new ShowWindow(index, tests);
//
//     // Simulator Main Loop
//     SimWindow -> main();
//
//     std::cout << "Simulator Ended" << std::endl;
// }



int main (int argc, char *argv[])
{
    uint            tests = 1;
    uint            index = 0;

    if (argc == 3)
    {
        index = std::atoi(argv[1]);
        tests = std::atoi(argv[2]);
    }

    // funcProfile  (argc, argv);

    synthBayesOpt(index, tests);

    // simoxBayesOpt(index, tests, argc, argv);



    return 0;
}
