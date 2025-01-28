#include "Params.h"

// The universal constructor for both executable and shared library
// When the executable is run from the commandline,
// it will first generate an CVRPLIB instance from .vrp file, then supply necessary information.
Params::Params(
	const std::vector<double> &x_coords,
	const std::vector<double> &y_coords,
	const std::vector<std::vector<double>> &dist_mtx,
	const std::vector<double> &service_time,
	const std::vector<double> &demands,
	double vehicleCapacity,
	double durationLimit,
	int nbVeh,
	bool isDurationConstraint,
	bool verbose,
	const AlgorithmParameters &ap,
	const std::vector<double> &th,
	double Tmax)
	: ap(ap), isDurationConstraint(isDurationConstraint), nbVehicles(nbVeh), durationLimit(durationLimit),
	  vehicleCapacity(vehicleCapacity), timeCost(dist_mtx), verbose(verbose)
{
	// This marks the starting time of the algorithm
	startTime = clock();

	nbClients = (int)demands.size() - 1; // Need to substract the depot from the number of nodes
	totalDemand = 0.;
	maxDemand = 0.;
	T = Tmax;

	// Initialize RNG
	ran.seed(ap.seed);

	// check if valid coordinates are provided
	areCoordinatesProvided = (demands.size() == x_coords.size()) && (demands.size() == y_coords.size());
	bool areTHProvided = demands.size() == th.size();

	cli = std::vector<Client>(nbClients + 1);
	for (int i = 0; i <= nbClients; i++)
	{
		// If useSwapStar==false, x_coords and y_coords may be empty.
		if (ap.useSwapStar == 1 && areCoordinatesProvided)
		{
			cli[i].coordX = x_coords[i];
			cli[i].coordY = y_coords[i];
			cli[i].polarAngle = CircleSector::positive_mod(
				32768. * atan2(cli[i].coordY - cli[0].coordY, cli[i].coordX - cli[0].coordX) / PI);
		}
		else
		{
			cli[i].coordX = 0.0;
			cli[i].coordY = 0.0;
			cli[i].polarAngle = 0.0;
		}

		cli[i].serviceDuration = service_time[i];
		cli[i].demand = demands[i];
		if (areTHProvided)
			cli[i].th = th[i];
		if (cli[i].demand > maxDemand)
			maxDemand = cli[i].demand;
		totalDemand += cli[i].demand;
	}

	if (verbose && ap.useSwapStar == 1 && !areCoordinatesProvided)
		std::cout << "----- NO COORDINATES HAVE BEEN PROVIDED, SWAP* NEIGHBORHOOD WILL BE DEACTIVATED BY DEFAULT" << std::endl;

	// Default initialization if the number of vehicles has not been provided by the user
	if (nbVehicles == INT_MAX)
	{
		nbVehicles = (int)std::ceil(1.3 * totalDemand / vehicleCapacity) + 3; // Safety margin: 30% + 3 more vehicles than the trivial bin packing LB
		if (verbose)
			std::cout << "----- FLEET SIZE WAS NOT SPECIFIED: DEFAULT INITIALIZATION TO " << nbVehicles << " VEHICLES" << std::endl;
	}
	else
	{
		if (verbose)
			std::cout << "----- FLEET SIZE SPECIFIED: SET TO " << nbVehicles << " VEHICLES" << std::endl;
	}

	// Calculation of the maximum distance
	maxDist = 0.;
	for (int i = 0; i <= nbClients; i++)
		for (int j = 0; j <= nbClients; j++)
			if (timeCost[i][j] > maxDist)
				maxDist = timeCost[i][j];

	// Calculation of the correlated vertices for each customer (for the granular restriction)
	correlatedVertices = std::vector<std::vector<int>>(nbClients + 1);
	std::vector<std::set<int>> setCorrelatedVertices = std::vector<std::set<int>>(nbClients + 1);
	std::vector<std::pair<double, int>> orderProximity;
	for (int i = 1; i <= nbClients; i++)
	{
		orderProximity.clear();
		for (int j = 1; j <= nbClients; j++)
			if (i != j)
				orderProximity.emplace_back(timeCost[i][j], j);
		std::sort(orderProximity.begin(), orderProximity.end());

		for (int j = 0; j < std::min<int>(ap.nbGranular, nbClients - 1); j++)
		{
			// If i is correlated with j, then j should be correlated with i
			setCorrelatedVertices[i].insert(orderProximity[j].second);
			setCorrelatedVertices[orderProximity[j].second].insert(i);
		}
	}

	// Filling the vector of correlated vertices
	for (int i = 1; i <= nbClients; i++)
		for (int x : setCorrelatedVertices[i])
			correlatedVertices[i].push_back(x);

	// Safeguards to avoid possible numerical instability in case of instances containing arbitrarily small or large numerical values
	if (maxDist < 0.1 || maxDist > 100000)
		throw std::string(
			"The distances are of very small or large scale. This could impact numerical stability. Please rescale the dataset and run again.");
	if (maxDemand < 0.1 || maxDemand > 100000)
		throw std::string(
			"The demand quantities are of very small or large scale. This could impact numerical stability. Please rescale the dataset and run again.");
	if (nbVehicles < std::ceil(totalDemand / vehicleCapacity))
		throw std::string("Fleet size is insufficient to service the considered clients.");

	// A reasonable scale for the initial values of the penalties
	penaltyDuration = 1;
	penaltyCapacity = std::max<double>(0.1, std::min<double>(1000., maxDist / maxDemand));

	for (int i = 0; i < nbClients + 1; i++)
	{
		for (int j = 0; j < nbClients + 1; j++)
		{
			if (i != j)
			{
				sor1.emplace_back(i, j, cli[i].th + cli[j].th);
				sor2.emplace_back(i, j, cli[i].th * cli[j].th);
			}
		}
	}

	std::sort(sor1.begin(), sor1.end(), [](const auto &a, const auto &b)
			  {
				  return std::get<2>(a) > std::get<2>(b); // Compare by the weight (3rd element)
			  });
	std::sort(sor2.begin(), sor2.end(), [](const auto &a, const auto &b)
			  {
				  return std::get<2>(a) > std::get<2>(b); // Compare by the weight (3rd element)
			  });

	sor1_index.resize(nbClients + 1, std::vector<int>(nbClients + 1, 0));
	sor2_index.resize(nbClients + 1, std::vector<int>(nbClients + 1, 0));

	// Fill sor1_index with the index of each pair in sor1
	for (int i = 0; i < sor1.size(); ++i)
	{
		int client1 = std::get<0>(sor1[i]); // Get the first client in the pair
		int client2 = std::get<1>(sor1[i]); // Get the second client in the pair
		sor1_index[client1][client2] = i;	// Set the index of the pair in sor1
	}

	// Optionally, fill sor2_index in a similar way (if needed)
	for (int i = 0; i < sor2.size(); ++i)
	{
		int client1 = std::get<0>(sor2[i]); // Get the first client in the pair
		int client2 = std::get<1>(sor2[i]); // Get the second client in the pair
		sor2_index[client1][client2] = i;	// Set the index of the pair in sor2
	}

	if (verbose)
		std::cout << "----- INSTANCE SUCCESSFULLY LOADED WITH " << nbClients << " CLIENTS AND " << nbVehicles << " VEHICLES" << std::endl;
}
