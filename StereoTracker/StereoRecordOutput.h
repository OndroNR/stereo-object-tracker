#pragma once
#include "common.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>

class StereoRecordOutput
{
private:
	std::ofstream filestream;
	std::string list_path;
	std::string frames_path;
public:
	StereoRecordOutput(std::string path, std::string frames_path, std::string list_filename);
	~StereoRecordOutput(void);
	bool WritePair(struct StereoPair &stereoPair);
	void Reset();
	int counter;
};
