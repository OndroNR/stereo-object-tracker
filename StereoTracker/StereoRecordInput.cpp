#include "StereoRecordInput.h"

using namespace std;
using namespace cv;

StereoRecordInput::StereoRecordInput(std::string path, std::string frames_subpath, std::string list_filename)
{
	filestream.open((path + list_filename).c_str());
	if (!filestream)
	{
		std::cout << "Could not open stereo record file" << std::endl;
	}
	else
	{
		std::cout << "Stereo record file open" << std::endl;
	}

	if (frames_path.empty())
	{
		frames_path = path;
	}
	else
	{
		frames_path = path + PATH_SEPARATOR + frames_subpath;
	}
}

StereoRecordInput::~StereoRecordInput(void)
{
	filestream.close();
}

bool StereoRecordInput::GetNextPair(struct StereoPair& stereoPair)
{
	if (!filestream)
		return false;

	if (!filestream.good())
		return false;

	string filename_base;
	getline(filestream, filename_base, ';');

	string timestamp_s;
	getline(filestream, timestamp_s);

	string left_path = frames_path + PATH_SEPARATOR + filename_base + "l.tiff";
	string right_path = frames_path + PATH_SEPARATOR + filename_base + "r.tiff";
	stereoPair.frames[0] = imread(left_path);
	stereoPair.frames[1] = imread(right_path);
	stereoPair.timestamp = atof(timestamp_s.c_str());
	return true;
}

void StereoRecordInput::Reset()
{
	filestream.clear();
	filestream.seekg(0, filestream.beg);
}
