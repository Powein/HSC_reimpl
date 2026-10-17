#pragma once
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

template<typename T>
std::vector<T> read_csv(const std::string& log_file)
{
	std::vector<T> column_data;
	std::ifstream file(log_file);
	std::string line;

	if (!file.is_open())
	{
		std::cerr << "read_csv::unable to open file " << log_file << std::endl;
		return column_data;
	}

	while (std::getline(file, line))
	{
		std::stringstream ss(line);
		std::string cell;

		if (std::getline(ss, cell, ','))
		{
			T value = std::stod(cell);
			column_data.push_back(value);
		}
	}

	file.close();
	return column_data;
}

template<class T>
inline void export_csv(const char* log_file, unsigned int n, unsigned int m, std::vector<T> host_array)
{
	std::ofstream outfile;
	outfile.open(log_file, std::ios::out);
	//outfile << "start\n";
	for (unsigned int i = 0; i < n / m; ++i)
	{
		for (unsigned int j = 0; j < m; ++j)
		{
			outfile << std::setprecision(12) << host_array[m * i + j] << " ";
		}
		outfile << std::endl;
	}
	//outfile << "end\n";
	outfile.close();
}
