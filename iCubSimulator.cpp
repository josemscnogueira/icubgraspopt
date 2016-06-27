#include <json/json.h>

#include "LogManager.hpp"
#include "Window.hpp"


/**
 * Global Variables
 */
std::string str_config_filepath("settings/opt_cfg.json");
bool        use_tgp     = false;
uint        log_mode    = LOG_LEARNING | MONTE_CARLO_OPT;
uint        total_tests = 1;
uint        test_idx    = 0;


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
    if (argc == 3)
    {
        test_idx    = std::atoi(argv[1]);
        total_tests = std::atoi(argv[2]);
    }

    // Read Json File configuration
    Json::Value read_config = LogManager::readJsonConfig(str_config_filepath);

    if (!read_config["TGPOptimizable"].isNull())
    {
        if (!read_config["TGPOptimizable"]["name"].isNull())
        {
            if (read_config["TGPOptimizable"]["name"].asString().compare("iCubOptimizable") == 0)
            {
                // Variavbles
                ShowWindow* SimWindow;

                // Initiate Qt
                SoDB::init();
                SoQt::init(argc, argv, "iCubSimulator");

                std::cout << "Simulator Started" << std::endl;

                // Simulator Window Initialization
                SimWindow = new ShowWindow(test_idx, total_tests);

                // Simulator Main Loop
                SimWindow -> main();

                std::cout << "Simulator Ended" << std::endl;

                delete SimWindow;

                return(0);
            }
        }
    }

    return (-1);
}
