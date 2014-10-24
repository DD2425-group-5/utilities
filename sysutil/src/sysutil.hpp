#include <iostream>
#include <algorithm>
#include <dirent.h>
#include <sys/stat.h>
#include <string>
#include <iostream>
#include <map>
#include <vector>
#include <cstdio>
#include <cstring>

class SysUtil {
private:
    SysUtil(){}
public:
    struct DirContents {
	std::string path;
	std::vector<std::string> dirs;
	std::vector<std::string> files;
    };

    static bool isType(std::string path, mode_t mode);
    static bool isDir(std::string path);
    static bool isFile(std::string path);
    static std::string cleanDirPath(std::string path);
    static std::string removeBaseName(std::string path);
    static std::string insertSuffix(std::string fileName, std::string suffix);
    static DirContents listDir(std::string path);
    static std::vector<DirContents> getDirContents(std::string dirName);
};

