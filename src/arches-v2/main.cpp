#include "stdafx.hpp"

#include "shared-utils.hpp"
#include "trax.hpp"
#include "dual-streaming.hpp"
#include "trax-kernel/intersect.hpp"

//global verbosity flag
int arches_verbosity = 1;

int main(int argc, char* argv[])
{
	Arches::GlobalConfig global_config(argc, argv);

	if (global_config.simulator == "trax")
	{
		Arches::TRaX::run_sim_trax(global_config);
	}
	else if (global_config.simulator == "dual-streaming")
	{
		Arches::DualStreaming::run_sim_dual_streaming(global_config);
	}
	
	return 0;
}