#include "StereoRecordOutput.h"

using namespace std;
using namespace cv;

StereoRecordOutput::StereoRecordOutput(std::string path, std::string frames_subpath, std::string list_filename)
{
	list_path = (path + list_filename);
	Reset();
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

StereoRecordOutput::~StereoRecordOutput(void)
{
	filestream.close();
}

bool StereoRecordOutput::WritePair(struct StereoPair& stereoPair)
{
	if (!filestream)
		return false;

	if (!filestream.good())
		return false;

	if (stereoPair.frames[0].empty())
		return false;
	if (stereoPair.frames[1].empty())
		return false;

	string filename_base = to_string(counter); // pad left 0!
	unsigned short counter_digits = 5;
	if (filename_base.length() < counter_digits)
		filename_base.insert(0, counter_digits - filename_base.length(), '0');

	string timestamp_s = to_string(stereoPair.timestamp);

	string left_path = frames_path + PATH_SEPARATOR + filename_base + "l.tiff";
	string right_path = frames_path + PATH_SEPARATOR + filename_base + "r.tiff";
	imwrite(left_path, stereoPair.frames[0]);
	imwrite(right_path, stereoPair.frames[1]);

	filestream << filename_base << ";" << timestamp_s << endl;

	counter++;

	return true;
}

void StereoRecordOutput::Reset()
{
	if (filestream)
		filestream.close();

	filestream.open(list_path.c_str());

	counter = 0;
}
