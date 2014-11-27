#include <iostream>
#include <algorithm>
#include <dirent.h>
#include <sys/stat.h>
#include <string>
#include <iostream>
#include <map>
#include <vector>
#include <cstdio>
#include <cerrno>
#include <cstring>

namespace SysUtil {
    struct DirContents {
	std::string path;
	std::vector<std::string> dirs;
	std::vector<std::string> files;
	// bit of a hack for use in imageExtractor. Need to know whether the
	// original path was used, or if this struct is a subdirectory on the
	// path
	bool isTop;
    };

    bool isType(std::string path, mode_t mode);
    bool isDir(std::string path);
    bool isFile(std::string path);
    std::string cleanDirPath(std::string path);
    std::string fullDirPath(std::string path);
    std::string removeBaseName(std::string path);
    std::string insertSuffix(std::string fileName, std::string suffix);
    DirContents listDir(std::string path);
    std::vector<DirContents> getDirContents(std::string dirName);
    bool queryUserYN(std::string question);
    std::string getDateTimeString();
    bool makeDir(std::string path);
};

