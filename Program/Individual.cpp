#include "Individual.h"

void Individual::evaluateCompleteCost(const Params &params)
{

	std::vector<std::tuple<int, int, double, double>> selected_edges;

	eval = EvalIndiv();
	for (int r = 0; r < params.nbVehicles; r++)
	{
		if (!chromR[r].empty())
		{
			double distance = params.timeCost[0][chromR[r][0]];
			selected_edges.emplace_back(0, chromR[r][0], params.cli[0].th + params.cli[chromR[r][0]].th, params.cli[0].th * params.cli[chromR[r][0]].th);
			double load = params.cli[chromR[r][0]].demand;
			double service = params.cli[chromR[r][0]].serviceDuration;
			predecessors[chromR[r][0]] = 0;
			for (int i = 1; i < (int)chromR[r].size(); i++)
			{
				distance += params.timeCost[chromR[r][i - 1]][chromR[r][i]];
				selected_edges.emplace_back(chromR[r][i - 1], chromR[r][i], params.cli[chromR[r][i - 1]].th + params.cli[chromR[r][i]].th, params.cli[chromR[r][i - 1]].th * params.cli[chromR[r][i]].th);

				load += params.cli[chromR[r][i]].demand;
				service += params.cli[chromR[r][i]].serviceDuration;
				predecessors[chromR[r][i]] = chromR[r][i - 1];
				successors[chromR[r][i - 1]] = chromR[r][i];
			}
			successors[chromR[r][chromR[r].size() - 1]] = 0;
			distance += params.timeCost[chromR[r][chromR[r].size() - 1]][0];
			selected_edges.emplace_back(chromR[r][chromR[r].size() - 1], 0, params.cli[chromR[r][chromR[r].size() - 1]].th + params.cli[0].th, params.cli[chromR[r][chromR[r].size() - 1]].th * params.cli[0].th);

			eval.distance += distance;
			eval.nbRoutes++;
			if (load > params.vehicleCapacity)
				eval.capacityExcess += load - params.vehicleCapacity;
			if (distance + service > params.durationLimit)
				eval.durationExcess += distance + service - params.durationLimit;
		}
	}

	std::sort(selected_edges.begin(), selected_edges.end(), [](const auto &a, const auto &b)
			  {
				  return std::get<2>(a) > std::get<2>(b); // Compare by the weight (3rd element)
			  });

	double S1 = 0.0;
	double robust_cost_1 = 0.0;
	for (const auto &edge : selected_edges)
	{
		if (S1 + 1.0 <= params.T)
		{
			// Can take full item
			robust_cost_1 += std::get<2>(edge);
			S1 += 1.0;
		}
		else
		{
			// Take fractional part
			double remaining = params.T - S1;
			if (remaining > 0)
			{
				robust_cost_1 += remaining * std::get<2>(edge);
				S1 += remaining;
			}
			break; // We've reached params.T, no need to continue
		}
	}
		std::sort(selected_edges.begin(), selected_edges.end(), [](const auto &a, const auto &b)
			  {
				  return std::get<3>(a) > std::get<3>(b); // Compare by the weight (3rd element)
			  });

	double S2 = 0.0;
	double robust_cost_2 = 0.0;
	for (const auto &edge : selected_edges)
	{
		if (S2 + 2.0 <= params.T* params.T)
		{
			// Can take full item
			robust_cost_2 += 2.0 * std::get<3>(edge);
			S2 += 2.0;
		}
		else
		{
			// Take fractional part
			double remaining = params.T*params.T - S2;
			if (remaining > 0)
			{
				robust_cost_2 += remaining * std::get<3>(edge);
				S2 += remaining;
			}
			break; // We've reached params.T, no need to continue
		}
	}

	// std::cout<<"test";
	eval.distance += robust_cost_1 + robust_cost_2;
	eval.penalizedCost = eval.distance + eval.capacityExcess * params.penaltyCapacity + eval.durationExcess * params.penaltyDuration ;
	eval.isFeasible = (eval.capacityExcess < MY_EPSILON && eval.durationExcess < MY_EPSILON);
}

Individual::Individual(Params &params)
{
	successors = std::vector<int>(params.nbClients + 1);
	predecessors = std::vector<int>(params.nbClients + 1);
	chromR = std::vector<std::vector<int>>(params.nbVehicles);
	chromT = std::vector<int>(params.nbClients);
	for (int i = 0; i < params.nbClients; i++)
		chromT[i] = i + 1;
	std::shuffle(chromT.begin(), chromT.end(), params.ran);
	eval.penalizedCost = 1.e30;
}

Individual::Individual(Params &params, std::string fileName) : Individual(params)
{
	double readCost;
	chromT.clear();
	std::ifstream inputFile(fileName);
	if (inputFile.is_open())
	{
		std::string inputString;
		inputFile >> inputString;
		// Loops in the input file as long as the first line keyword is "Route"
		for (int r = 0; inputString == "Route"; r++)
		{
			inputFile >> inputString;
			getline(inputFile, inputString);
			std::stringstream ss(inputString);
			int inputCustomer;
			while (ss >> inputCustomer) // Loops as long as there is an integer to read in this route
			{
				chromT.push_back(inputCustomer);
				chromR[r].push_back(inputCustomer);
			}
			inputFile >> inputString;
		}
		if (inputString == "Cost")
			inputFile >> readCost;
		else
			throw std::string("Unexpected token in input solution");

		// Some safety checks and printouts
		evaluateCompleteCost(params);
		if ((int)chromT.size() != params.nbClients)
			throw std::string("Input solution does not contain the correct number of clients");
		if (!eval.isFeasible)
			throw std::string("Input solution is infeasible");
		if (eval.penalizedCost != readCost)
			throw std::string("Input solution has a different cost than announced in the file");
		if (params.verbose)
			std::cout << "----- INPUT SOLUTION HAS BEEN SUCCESSFULLY READ WITH COST " << eval.penalizedCost << std::endl;
	}
	else
		throw std::string("Impossible to open solution file provided in input in : " + fileName);
}
