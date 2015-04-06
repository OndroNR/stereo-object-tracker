#include "common.h"
#include <fstream>
#include <string>
#include <map>

using namespace std;

class ConfigStore
{
public:
    static ConfigStore& get()
    {
        static ConfigStore instance;
        return instance;
    }
    void parseFile(std::ifstream& inStream);
	string getString(string key);
	const char* getChar(string key);
	float getFloat(string key);
	double getDouble(string key);
	int getInt(string key);
	long getLong(string key);
private:
    ConfigStore(){};
    std::map<std::string, std::string> storedConfig;
};
