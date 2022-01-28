
/*driver.cpp
* Solve a MAPF instance on 2D grids.
*/
#include <boost/program_options.hpp>
#include <boost/tokenizer.hpp>
#include "PBS.h"

/* Declare some static utility functions */
static void usage();

/* Main function */
int main(int argc, char** argv)
{
	namespace po = boost::program_options;
	// Declare the supported options.
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "produce help message")

		// params for the input instance and experiment settings
		("map,m", po::value<string>()->required(), "input file for map")
		("agents,a", po::value<string>()->required(), "input file for agents")
		("output,o", po::value<string>(), "output file for schedule")
		("agentNum,k", po::value<int>()->default_value(0), "number of agents")
		("cutoffTime,t", po::value<double>()->default_value(7200), "cutoff time (seconds)")
		("screen,s", po::value<int>()->default_value(1), "screen option (0: none; 1: results; 2:all)")
		("seed,d", po::value<int>()->default_value(0), "random seed")
		// params for instance generators
		("rows", po::value<int>()->default_value(0), "number of rows")
		("cols", po::value<int>()->default_value(0), "number of columns")
		("obs", po::value<int>()->default_value(0), "number of obstacles")
		("warehouseWidth", po::value<int>()->default_value(0), "width of working stations on both sides, for generating instances")
		;

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);

	if (vm.count("help"))
	{
		usage();
		cout << desc << endl;
		return 1;
	}

	po::notify(vm);
	/////////////////////////////////////////////////////////////////////////
	// check the correctness and consistence of params
	/*if (vm["sipp"].as<bool>() && vm["targetReasoning"].as<bool>())
	{
		cerr << "SIPP cannot work together with target reasoning!" << endl;
		return -1;
	}*/

  
	srand((int) time(0));

	///////////////////////////////////////////////////////////////////////////
	// load the instance
	Instance instance(vm["map"].as<string>(), vm["agents"].as<string>(),
					  vm["agentNum"].as<int>(),
					  vm["rows"].as<int>(), vm["cols"].as<int>(), vm["obs"].as<int>(), vm["warehouseWidth"].as<int>());

	srand(vm["seed"].as<int>());

	int runs = 1;
	
	//////////////////////////////////////////////////////////////////////
	// initialize the solver
	PBS pbs(instance, vm["screen"].as<int>());
	//////////////////////////////////////////////////////////////////////
	// run
	double runtime = 0;
	int min_f_val = 0;
	for (int i = 0; i < runs; i++)
	{
		pbs.clear();
		pbs.solve(vm["cutoffTime"].as<double>(), min_f_val);
		runtime += pbs.runtime;
		if (pbs.solution_found)
			break;
		min_f_val = (int) pbs.min_f_val;
		pbs.randomRoot = true;
	}
	pbs.runtime = runtime;
	if (vm.count("output"))
		pbs.saveResults(vm["output"].as<string>(), vm["agents"].as<string>());
  pbs.clearSearchEngines();

	return 0;

}


/*
Prints out usage help.
*/
static void usage()
{
	// TODO: update the following information
	fprintf(stderr, "Usage: optimize instance exp strat [options]\n");
	fprintf(stderr, "Arguments:\n");
	fprintf(stderr, "	help		-> this list\n");
	fprintf(stderr, "	screen		-> screen output on(=1) or off(=0) (default: 0)\n");
	fprintf(stderr, "	instance	-> MIP in MPS format\n");
	fprintf(stderr, "	exp		-> experiment name\n");
	fprintf(stderr, "	strat		-> branching strategy:\n");
	fprintf(stderr, "				-1: CPLEX Default\n");
	fprintf(stderr, "				-2: FSB\n");
	fprintf(stderr, "				-3: Most Infeasible\n");
	fprintf(stderr, "				-4: SB\n");
	fprintf(stderr, "				-5: PC\n");
	fprintf(stderr, "				 3: Hybrid SB/PC\n");
	fprintf(stderr, "				 6: ML\n");
	fprintf(stderr, "				 7: ML + Problem Features \n");
	fprintf(stderr, "	sbnodes		-> num. of SB nodes if strat:={3,6,7}\n");
	fprintf(stderr, "	varPerNode	-> num. of variables per SB node if strat:={3,6,7}\n");
	fprintf(stderr, "	varSorting	-> variable sorting criterion if strat:={-4,3,6,7}\n");
	fprintf(stderr, "	learningAlg	-> learning algorithm if strat={6,7}\n");
	fprintf(stderr, "				 1: SVM-Rank\n");
	fprintf(stderr, "				 2: NDCG\n");
	fprintf(stderr, "				 3: Regression\n");
	fprintf(stderr, "	loss		-> SVM loss function variant, (default: 2)\n");
	fprintf(stderr, "	c		-> SVM parameter (default: 0.1)\n");
	fprintf(stderr, "	alpha		-> Fraction of max. SB score to get label 1 (default: 0.2)\n");
	fprintf(stderr, "	diag		-> diagnostic mode (default: 0)\n");
	fprintf(stderr, "	root		-> root-only cuts and heuristics (default: 0)\n");
	fprintf(stderr, "	maxtime		-> time cutoff in sec. (default: 7200)\n");
	fprintf(stderr, "	restart		-> Restart after learning(=1) or not (=0), (default: 0)\n");
	fprintf(stderr, "	kernel		-> Add interaction features with Kernel(=1) or not(=0), (default: 1)\n");
	fprintf(stderr, "	seed		-> CPLEX random seed (default: 1)\n");
	fprintf(stderr, "	cutoff		-> Use instance's optimal value as cutoff(=1) or not(=0), (default: 0)\n");
	fprintf(stderr, "	whichpc		-> Use PC scores as search goes(=1) or after Phase 1(=0), (default: 0)\n");
	fprintf(stderr, "whichFeatures	-> which features to include, (default 0) \n");
	fprintf(stderr, "				0: All\n");
	fprintf(stderr, "				1: Static\n");
	fprintf(stderr, "				2: Active\n");
	fprintf(stderr, "				3: Compact\n");
	fprintf(stderr, "	desc		-> Optional string describing this experiment\n");
	fprintf(stderr, "Exiting...\n");
}
