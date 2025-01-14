#include "stdafx.hpp"

#include "shared-utils.hpp"
#include "trax.hpp"
#include "dual-streaming.hpp"
#include "ric.hpp"
#include "strata.hpp"

//global verbosity flag
int arches_verbosity = 1;

int main(int argc, char* argv[])
{
	Arches::SimulationConfig sim_config(argc, argv);
	sim_config.print();


	if (sim_config.get_string("arch_name") == "TRaX")
	{
		Arches::TRaX::run_sim_trax(sim_config);
	}
	else if (sim_config.get_string("arch_name") == "Dual-Streaming")
	{
		Arches::DualStreaming::run_sim_dual_streaming(sim_config);
	}
	else if(sim_config.get_string("arch_name") == "RIC")
	{
		Arches::RIC::run_sim_ric(sim_config);
	}
	else if (global_config.simulator == 2)
	{
		Arches::STRaTA::run_sim_strata(global_config);
	}
	
	return 0;
}