#include "sysutil.hpp"

using namespace std;

bool SysUtil::isType(std::string path, mode_t mode) {
    struct stat s;
    if (stat(path.c_str(), &s) == 0) {
	if (s.st_mode & mode) {
	    return true;
	} else {
	    return false;
	}
    } else {
	return false;
    }
}

bool SysUtil::isDir(std::string path) {
    return isType(path, S_IFDIR);
}

bool SysUtil::isFile(std::string path) {
    return isType(path, S_IFREG);
}

/**
 * Removes the trailing slash from a directory path if there is one.
 */
std::string SysUtil::cleanDirPath(std::string path) {
    std::string dir = path;
    if (path[path.size() - 1] == '/'){
	dir = path.substr(0, dir.size() - 1);
    }
    return dir;
}

/**
 * Removes the base name from a path, leaving either the directory or the 
 * file name.
 */
std::string SysUtil::removeBaseName(std::string path) {
    if (isFile(path)){
	int lastDir = path.find_last_of("/");
	return path.substr(lastDir);
    } else if (isDir(path)){
	std::string cleanPath = cleanDirPath(path);
	int lastDir = cleanPath.find_last_of("/");
	return cleanPath.substr(lastDir + 1);
    }
    return path;
}

/**
 * Insert a suffix at the end of a filename which has an extension. The suffix
 * will be placed before the extension.
 */
std::string SysUtil::insertSuffix(std::string fileName, std::string suffix) {
    int dotInd = fileName.find_last_of(".");
    std::string raw = fileName.substr(0, dotInd);
    std::string ext = fileName.substr(dotInd);
    
    return raw + suffix + ext;
}


SysUtil::DirContents SysUtil::listDir(std::string path) {
    SysUtil::DirContents c;
    c.path = cleanDirPath(path);
    DIR* dir;
    struct dirent* ent;
    if ((dir = opendir(path.c_str())) != NULL) {
	while ((ent = readdir(dir)) != NULL) {
	    // ignore current and parent dirs
	    if (strcmp(ent->d_name, ".") == 0 
		|| strcmp(ent->d_name, "..") == 0){
		continue;
	    }
	    std::string fullPath = c.path + "/" + std::string(ent->d_name);
	    if (isFile(fullPath)) {
		c.files.push_back(fullPath);
	    } else if (isDir(fullPath)) {
		c.dirs.push_back(fullPath);
	    }
	}
    } else {
	// Couldn't access directory
	perror("");
	std::cout << "Error processing " << path.c_str() << std::endl;
    }

    return c;
}

vector<SysUtil::DirContents> SysUtil::getDirContents(string dirName) {
    vector<SysUtil::DirContents> dirs;

    SysUtil::DirContents top = SysUtil::listDir(dirName);
    
    if (top.dirs.size() == 0) {
	// If there are no directories, assume you want to process files in the
	// directory
	top.path = "";
	dirs.push_back(top);
    } else {
	// Otherwise, assume that the maximum depth of the directory structure
	// is 1, and extract the file names out of the directories below the one
	// passed as a parameter.
	for (int dir = 0; dir < top.dirs.size(); dir++){
	    SysUtil::DirContents sub = SysUtil::listDir(top.dirs[dir]);
	    if (sub.files.size() != 0) {
		dirs.push_back(sub);
	    }
	}
    }
    return dirs;
}
