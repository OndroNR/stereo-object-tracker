#include "ConfigStore.h"
#include <iostream>

using namespace std;

void ConfigStore::parseFile(std::ifstream& inStream)
{
	if (inStream)
	{
		storedConfig.clear();

		string line;

		while (getline(inStream, line))
		{
			if (line.empty())
				continue;
			if (line.c_str()[0] == '#') // comment
				continue;

			size_t delim = line.find("=");

			if (delim == string::npos) // invalid entry
				continue;

			string key = trim_copy(line.substr(0, delim));
			string value = trim_copy(line.substr(delim+1));

			std::cout << "'" << key << "' => '" << value << "'" << endl;

			storedConfig[key] = value;
		}
	}
}

string ConfigStore::getString(string key)
{
	return storedConfig[key];
}

const char* ConfigStore::getChar(string key)
{
	return storedConfig[key].c_str();
}

float ConfigStore::getFloat(string key)
{
	return (float)getDouble(key);
}

double ConfigStore::getDouble(string key)
{
	return atof(getChar(key));
}

int ConfigStore::getInt(string key)
{
	return atoi(getChar(key));
}

long ConfigStore::getLong(string key)
{
	return atol(getChar(key));
}
