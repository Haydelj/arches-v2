#include "stdafx.hpp"

#include "shared-utils.hpp"
#include "trax.hpp"
#include "dual-streaming.hpp"
#include "ric.hpp"

//global verbosity flag
int arches_verbosity = 1;

int main(int argc, char* argv[])
{
	Arches::GlobalConfig global_config(argc, argv);

	if (global_config.simulator == 0)
	{
		Arches::TRaX::run_sim_trax(global_config);
	}
	else if (global_config.simulator == 1)
	{
		Arches::DualStreaming::run_sim_dual_streaming(global_config);
	}
	else if(global_config.simulator == 2)
	{
		Arches::RIC::run_sim_ric(global_config);
	}
	
	return 0;
}