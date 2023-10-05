#include "stdafx.hpp"

//#include "trax.hpp"
#include "dual-streaming.hpp"

//global verbosity flag
int arches_verbosity = 1;

int main(int argc, char* argv[])
{
	Arches::run_sim_dual_streaming(argc, argv);
	return 0;
}