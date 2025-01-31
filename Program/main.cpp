#include "Genetic.h"
#include "commandline.h"
#include "LocalSearch.h"
#include "Split.h"
#include "InstanceCVRPLIB.h"
#include "InstanceZA.h"
using namespace std;

int main(int argc, char *argv[])
{
	try
	{
		// // Reading the arguments of the program
		//CommandLine commandline(argc, argv);
		AlgorithmParameters ap = default_algorithm_parameters();
		ap.nbIter = 3000;

		// // Print all algorithm parameter values
		// if (commandline.verbose)
		// 	print_algorithm_parameters(commandline.ap);

		// // Reading the data file and initializing some data structures
		// if (commandline.verbose)
		// 	std::cout << "----- READING INSTANCE: " << commandline.pathInstance << std::endl;
		//InstanceCVRPLIB cvrp1(commandline.pathInstance, commandline.isRoundingInteger);
		//
		string path = string("/home/bvdka/HGS-CVRP/Instances/n_80-euclidean_true");
		InstanceZA cvrp(path,1);

			Params params(cvrp.x_coords,cvrp.y_coords,cvrp.dist_mtx,cvrp.service_time,cvrp.demands,
				          cvrp.vehicleCapacity,cvrp.durationLimit,20,cvrp.isDurationConstraint,1,ap,cvrp.th,cvrp.T);

			// Running HGS
			Genetic solver(params);
			solver.run();

			// // Exporting the best solution
			// if (solver.population.getBestFound() != NULL)
			// {
			// 	if (params.verbose) std::cout << "----- WRITING BEST SOLUTION IN : " << commandline.pathSolution << std::endl;
			// 	solver.population.exportCVRPLibFormat(*solver.population.getBestFound(),commandline.pathSolution);
			// 	solver.population.exportSearchProgress(commandline.pathSolution + ".PG.csv", commandline.pathInstance);
			// }
			solver.population.exportCVRPLibFormat(*solver.population.getBestFound(),"./output");
	}

	catch (const string &e)
	{
		std::cout << "EXCEPTION | " << e << std::endl;
	}
	catch (const std::exception &e)
	{
		std::cout << "EXCEPTION | " << e.what() << std::endl;
	}
	return 0;
}
