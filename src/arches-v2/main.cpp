#include "stdafx.hpp"

#include "shared-utils.hpp"
#include "trax.hpp"
#include "dual-streaming.hpp"
#include "ric.hpp"
#include "strata.hpp"
#include "strata-rt.hpp"
#include "trax-sg.hpp"

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
	else if(sim_config.get_string("arch_name") == "TRaX-SG")
	{
		Arches::TRaXSG::run_sim(sim_config);
	}
	else if (sim_config.get_string("arch_name") == "STRaTA")
	{
		Arches::STRaTA::run_sim_strata(sim_config);
	}
	else if(sim_config.get_string("arch_name") == "STRaTA-RT")
	{
		Arches::STRaTART::run_sim_strata_rt(sim_config);
	}
	else if (sim_config.get_string("arch_name") == "Dual-Streaming")
	{
		Arches::DualStreaming::run_sim_dual_streaming(sim_config);
	}
	else if(sim_config.get_string("arch_name") == "RIC")
	{
		Arches::RIC::run_sim_ric(sim_config);
	}
	
	return 0;
}