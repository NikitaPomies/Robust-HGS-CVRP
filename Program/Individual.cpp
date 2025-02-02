#include "Individual.h"

void Individual::evaluateCompleteCost(const Params &params)
{

	for (auto &row : is_selected)
	{
		for (auto &val : row)
		{
			val = 0;
		}
	}
	eval = EvalIndiv();
	for (int r = 0; r < params.nbVehicles; r++)
	{
		if (!chromR[r].empty())
		{
			double distance = params.timeCost[0][chromR[r][0]];
			is_selected[0][chromR[r][0]] = 1;
			double load = params.cli[chromR[r][0]].demand;
			double service = params.cli[chromR[r][0]].serviceDuration;
			predecessors[chromR[r][0]] = 0;
			for (int i = 1; i < (int)chromR[r].size(); i++)
			{
				distance += params.timeCost[chromR[r][i - 1]][chromR[r][i]];
				is_selected[chromR[r][i - 1]][chromR[r][i]] = 1;

				load += params.cli[chromR[r][i]].demand;
				service += params.cli[chromR[r][i]].serviceDuration;
				predecessors[chromR[r][i]] = chromR[r][i - 1];
				successors[chromR[r][i - 1]] = chromR[r][i];
			}
			successors[chromR[r][chromR[r].size() - 1]] = 0;
			distance += params.timeCost[chromR[r][chromR[r].size() - 1]][0];
			is_selected[chromR[r][chromR[r].size() - 1]][0] = 1;

			eval.distance += distance;
			eval.nbRoutes++;
			if (load > params.vehicleCapacity)
				eval.capacityExcess += load - params.vehicleCapacity;
			if (distance + service > params.durationLimit)
				eval.durationExcess += distance + service - params.durationLimit;
		}
	}

	auto [robust_cost_1, p1] = computeRobustCost1(params, is_selected);

	auto [robust_cost_2, p2] = computeRobustCost2(params, is_selected);
	last_edge_type_1 = p1;
	last_edge_type_2 = p2;

	eval.robust_cost_1 = robust_cost_1;
	eval.robust_cost_2 = robust_cost_2;
	eval.robust_cost = robust_cost_1 + robust_cost_2;
	// std::cout<<"test";
	eval.distance += robust_cost_1 + robust_cost_2;
	eval.penalizedCost = eval.distance + eval.capacityExcess * params.penaltyCapacity + eval.durationExcess * params.penaltyDuration;
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
	is_selected.resize(params.nbClients + 1, std::vector<int>(params.nbClients + 1, 0));
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

void Individual::computeSelectedEdges(const Params &params)
{
	for (int r = 0; r < params.nbVehicles; r++)
	{
		if (!chromR[r].empty())
		{
			is_selected[0][chromR[r][0]] = 1;
			for (int i = 1; i < (int)chromR[r].size(); i++)
			{
				is_selected[chromR[r][i - 1]][chromR[r][i]] = 1;
				;
			}
			is_selected[chromR[r][chromR[r].size() - 1]][0] = 1;
		}
	}
}

std::tuple<double, int, int> Individual::computeRobustCost(const Params &params, std::vector<std::vector<int>> &is_selec)
{
	auto [robust_cost_1, p1] = computeRobustCost1(params, is_selec);
	auto [robust_cost_2, p2] = computeRobustCost2(params, is_selec);

	return std::make_tuple(robust_cost_1 + robust_cost_2, p1, p2);
}

std::tuple<double, int> Individual::computeRobustCost1(const Params &params, std::vector<std::vector<int>> &is_selec)
{
	// assume is_selected is correct
	double S1 = 0.0;
	double robust_cost_1 = 0.0;
	int last_idx = 0;
	for (int i = 0; i < params.sor1.size(); i++)
	{
		auto &edge = params.sor1[i];
		if (is_selec[std::get<0>(edge)][std::get<1>(edge)] == 0)
			continue;

		if (S1 + 1.0 <= params.T)
		{
			// Can take full item
			robust_cost_1 += std::get<2>(edge);
			S1 += 1.0;
			last_idx = i;
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

	return std::make_tuple(robust_cost_1, last_idx);
}

std::tuple<double, int> Individual::computeRobustCost2(const Params &params, std::vector<std::vector<int>> &is_selec)
{

	double S2 = 0.0;
	double robust_cost_2 = 0.0;
	int last_idx = 0;

	for (int i = 0; i < params.sor2.size(); i++)
	{
		auto &edge = params.sor2[i];
		if (is_selec[std::get<0>(edge)][std::get<1>(edge)] == 0)
			continue;

		if (S2 + 2.0 <= params.T * params.T)
		{
			// Can take full item
			robust_cost_2 += 2.0 * std::get<2>(edge);
			S2 += 2.0;
			last_idx = i;
		}
		else
		{
			// Take fractional part
			double remaining = params.T * params.T - S2;
			if (remaining > 0)
			{
				robust_cost_2 += remaining * std::get<2>(edge);
				S2 += remaining;
				last_idx = i;
			}
			break; // We've reached params.T, no need to continue
		}
	}

	return std::make_tuple(robust_cost_2, last_idx);
}

std::tuple<double, int> Individual::updateRobustCost1(const Params &params, std::vector<std::vector<int>> &is_selec, std::vector<std::pair<int, int>> &to_delete,
													  std::vector<std::pair<int, int>> &to_add)
{
	int c = 0;
	int d = 0;

	for (const auto &[i, j] : to_delete)
	{
		if (params.sor1_index[i][j] <= last_edge_type_1)
			d += 1;
	}

	// Add new edges
	for (const auto &[i, j] : to_add)
	{
		if (params.sor1_index[i][j] <= last_edge_type_1)
			c += 1;
	}
	if (c == 0 && d == 0)
		return std::make_tuple(eval.robust_cost_1, last_edge_type_1);
	else
		return computeRobustCost1(params, is_selec);
}



std::tuple<double, int> Individual::updateRobustCost2(const Params &params, std::vector<std::vector<int>> &is_selec, std::vector<std::pair<int, int>> &to_delete,
													  std::vector<std::pair<int, int>> &to_add)
{
	int c = 0;
	int d = 0;

	for (const auto &[i, j] : to_delete)
	{
		if (params.sor2_index[i][j] <= last_edge_type_2)
			d += 1;
	}

	// Add new edges
	for (const auto &[i, j] : to_add)
	{
		if (params.sor2_index[i][j] <= last_edge_type_2)
			c += 1;
	}
	if (c == 0 && d == 0)
		return std::make_tuple(eval.robust_cost_2, last_edge_type_2);
	else
		return computeRobustCost2(params, is_selec);
}