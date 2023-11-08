#include "file.hpp"

#ifdef BUILD_PLATFORM_WINDOWS
	#define NOMINMAX
	#include <Windows.h>
	#include <io.h>
#else
	#include <cerrno>
	#include <fcntl.h>
#endif

#include "string.hpp"


namespace Arches { namespace Util {


static std::string _normalize_path(std::string const& path) {
	return get_replaced(path,"\\","/");
}
static std::string _get_dir(std::string const& path) {
	std::string temp = _normalize_path(path);
	return path.substr(0,path.find_last_of('/')+1);
}
static std::string _get_name(std::string const& path) {
	std::string temp = _normalize_path(path);
	return path.substr(path.find_last_of('/')+1);
}


File::File(std::string const& dir,std::string const& name, MODE mode) :
	dir(dir), name(name),
	mode(mode),
	_owns(true)
{
	//TODO: we handle some "safe" cases with a check.  This results in a potential race condition with other processes.
	char const* open_flag;
	switch (mode) {
		case MODE::R:
			open_flag = "rb";
			break;
		case MODE::W_SAFE:
			try {
				File test(dir,name,MODE::R);
				throw "File \""+dir+name+"\" already exists!";
			} catch (std::string const&) { /*Doesn't exist.  Safe to fall through.*/ }
		case MODE::W_NUKE:
			open_flag = "wb";
			break;
		case MODE::RW_SAFE:
			open_flag = "r+b";
			break;
		case MODE::RW_NUKE:
			open_flag = "w+b";
			break;
		case MODE::A:
			open_flag = "ab";
			break;
		case MODE::AR:
			open_flag = "a+b";
			break;
		nodefault;
	}

	backing = fopen((dir+name).c_str(),open_flag);
	if (backing!=nullptr);
	else throw File::_get_err_str(this,OP::OPEN);
}
File::File(std::string const& path, MODE mode) : File(_get_dir(path),_get_name(path), mode) {}
File::~File(void) {
	assert(backing!=nullptr);

	if (_owns) {
		fclose(backing);
	}
}

void File::flush(void) {
	//Although, some implementations allow it, with nonportable results.  See:
	//	http://en.cppreference.com/w/cpp/io/c/fflush
	assert(mode!=MODE::R);

	int result = fflush(backing);
	if (result!=EOF); else throw File::_get_err_str(this,OP::FLUSH);
}

void File::write_asciif(char const* format, ...) {
	va_list args;
	va_start(args,format);
	#ifdef BUILD_COMPILER_CLANG
		#pragma clang diagnostic push
		#pragma clang diagnostic ignored "-Wformat-nonliteral"
	#endif
	int result = vfprintf(backing,format,args);
	#ifdef BUILD_COMPILER_CLANG
		#pragma clang diagnostic pop
		#pragma clang diagnostic ignored "-Wformat-nonliteral"
	#endif
	va_end(args);

	if (result>=0); else throw File::_get_err_str(this,OP::WRITE);
}

bool File::can_open(std::string const& path) {
	//This should be faster than checking whether creating a new "File" throws an exception.
	FILE* test = fopen(path.c_str(),"rb");
	if (test!=nullptr) {
		fclose(test);
		return true;
	} else {
		return false;
	}
}
void File::remove(std::string const& path) {
	int result = ::remove(path.c_str());
	if (result==0); else throw File::_get_err_str(nullptr,OP::REMOVE);
}
void File::rename_nuke(std::string const& path_old, std::string const& path_new) {
	//TODO: another race condition
	if (!File::can_open(path_new)); else File::remove(path_new); //To prevent the following failing.

	int result = ::rename(path_old.c_str(),path_new.c_str());
	if (result==0); else throw File::_get_err_str(nullptr,OP::RENAME);
}
void File::rename_safe(std::string const& path_old, std::string const& path_new) {
	//TODO: another race condition
	if (!File::can_open(path_new)); else throw "File \""+path_new+"\" already exists!";

	int result = ::rename(path_old.c_str(),path_new.c_str());
	if (result==0); else throw File::_get_err_str(nullptr,OP::RENAME);
}

std::string File::_get_err_str(File const* file, File::OP op) {
	std::string err = strerror(errno);
	switch (op) {
		case File::OP::     OPEN: return "Could not open file \""+file->dir+file->name+"\" ("+err+")!";
		case File::OP::    CLOSE: return "Could not close file \""+file->dir+file->name+"\" ("+err+")!";
		case File::OP::     READ: return "Read error on file ("+err+")!";
		case File::OP::    WRITE: return "Write error on file ("+err+")!";
		case File::OP::GETOFFSET: return "Could not get offset on file ("+err+")!";
		case File::OP::SETOFFSET: return "Could not set offset on file ("+err+")!";
		case File::OP::  GETMODE: return "Could not get mode of file ("+err+")!";
		case File::OP::    FLUSH: return "Could not flush file ("+err+")!";
		case File::OP::GETLENGTH: return "Could not get length of file ("+err+")!";
		case File::OP::   RENAME: return "Could not rename file ("+err+")!";
		case File::OP::   REMOVE: return "Could not remove file ("+err+")!";
		nodefault;
	}
}


}}
