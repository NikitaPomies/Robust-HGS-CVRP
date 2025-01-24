//
// Created by chkwon on 3/22/22.
//
//#ifndef INSTANCEZA_H
//#define INSTANCEZA_H
#include<string>
#include<vector>

class InstanceZA
{
public:
	std::vector<double> x_coords;
	std::vector<double> y_coords;
	std::vector< std::vector<double> > dist_mtx;
	std::vector<double> service_time;
	std::vector<double> demands;
	std::vector<double> th;
	double T;
	double durationLimit = 1.e30;							// Route duration limit
	double vehicleCapacity = 1.e30;							// Capacity limit
	bool isDurationConstraint = false;						// Indicates if the problem includes duration constraints
	int nbClients ;											// Number of clients (excluding the depot)

	InstanceZA(std::string pathToInstance, bool isRoundingInteger);
};


//#endif //INSTANCEZA_H
