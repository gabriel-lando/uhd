// Minimal bonded wideband RX example
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/usrp/bonded/sync_manager.hpp>
#include <uhd/usrp/bonded/freq_planner.hpp>
#include <uhd/usrp/bonded/bonded_rx_streamer.hpp>
#include <iostream>

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    (void)argc;
    (void)argv;
    try {
        uhd::device_addr_t args;
        args["args"] = "";
        auto usrp = uhd::usrp::multi_usrp::make(args);

        uhd::usrp::bonded::sync_manager::config scfg;
        scfg.clock_source = "internal";
        scfg.time_source = "internal";
        uhd::usrp::bonded::sync_manager sync(usrp, scfg);

        uhd::usrp::bonded::bonded_rx_streamer::config cfg;
        uhd::usrp::bonded::bonded_rx_streamer brx(usrp, sync, cfg);

        std::cout << "Bonded RX example constructed." << std::endl;
        return EXIT_SUCCESS;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }
}
