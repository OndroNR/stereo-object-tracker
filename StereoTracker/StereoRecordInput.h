#pragma once
#include "common.h"
#include "stereovideoinput.h"
#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h>

class StereoRecordInput :
	public StereoVideoInput
{
private:
	std::ifstream filestream;
	std::string frames_path;
public:
	StereoRecordInput(std::string path, std::string frames_path, std::string list_filename);
	~StereoRecordInput(void);
	virtual bool GetNextPair(struct StereoPair &stereoPair);
};

