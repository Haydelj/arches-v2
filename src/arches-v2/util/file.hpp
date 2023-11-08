#pragma once

#include "stdafx.hpp"


namespace Arches { namespace Util {


//Encapsulates a file.
//	Note: files must be smaller than 4GiB when compiled for 32-bit architectures.
class File {
	public:
		std::string dir;
		std::string name;

		//Modes the file can be opened as.
		//	Note: all modes are "binary"; in binary mode, some platforms will stop lying to you
		//	about the file's contents (e.g., they don't convert newlines).
		enum class MODE {
			R,       //File open for reading.  File must exist on open.
			W_SAFE,  //File open for writing.  File must not already exist on open.
			W_NUKE,  //File open for writing.  File may already exist on open.
			RW_SAFE, //File open for reading/writing.  File must not already exist on open.
			RW_NUKE, //File open for reading/writing.  File may already exist on open.
			A,       //File open for write-appending.  File may already exist on open.
			AR       //File open for write-appending/reading.  File may already exist on open.
		};
		MODE mode;

		enum class OP {
			OPEN, CLOSE,
			READ, WRITE,
			GETOFFSET, SETOFFSET,
			GETMODE,
			FLUSH,
			GETLENGTH,
			RENAME,
			REMOVE
		};

		//File API backing.  User will not generally need, but it needs to be exposed for the
		//	benefit of interoperability with other libraries.
		FILE* backing;
	private:
		bool _owns;

	public:
		//Attempts to open file for reading.  Throws a string describing the problem on failure.
		File(std::string const& dir,std::string const& name, MODE mode);
		File(std::string const& path, MODE mode);
		virtual ~File(void);

		//Flushes any caches the file uses.
		void flush(void);

		//Attempts to read by one/several primitive object(s).
		template <typename type          > type                 read_bin(void                                  ) {
			type result; if (fread(&result, sizeof(type),1, backing)==1);
			else throw File::_get_err_str(this,OP::READ);
			return result;
		}
		template <typename type          > void                 read_bin(type                result[], size_t N) {
			if (fread(result, sizeof(type),N, backing)==N);
			else throw File::_get_err_str(this,OP::READ);
		}
		                                   std::vector<uint8_t> read_bin(                              size_t N) {
			std::vector<uint8_t> result(N);
			read_bin(result.data(),N);
			return result;
		}
		template <typename type, size_t N> void                 read_bin(std::array<type,N>& result            ) {
			if (fread(result.data(), sizeof(type),N, backing)==N);
			else throw File::_get_err_str(this,OP::READ);
		}

		//Attempts to write by one/several primitive object(s).
		template <typename type> void write_bin(type const& value) {
			if (fwrite(&value, sizeof(type),1, backing)==1);
			else throw File::_get_err_str(this,OP::WRITE);
		}
		template <typename type> void write_bin(type const values[], size_t N) {
			if (fwrite(values, sizeof(type),N, backing)==N);
			else throw File::_get_err_str(this,OP::WRITE);
		}
		template <typename type, size_t N> void write_bin(std::array<type,N> const& values) {
			if (fwrite(values.data(), sizeof(type),N, backing)==N);
			else throw File::_get_err_str(this,OP::WRITE);
		}
		#ifdef BUILD_COMPILER_GNU
		//https://gcc.gnu.org/onlinedocs/gcc-4.0.1/gcc/Function-Attributes.html format.  Note
		//	implicit `this` argument.
		void write_asciif(char const* format, ...) __attribute__ ((format(printf,2,3)));
		#else
		void write_asciif(char const* format, ...);
		#endif

		//Check if file can be opened
		static bool can_open(std::string const& path);
		//Delete a file
		static void remove(std::string const& path);
		//Rename the file.  The safe version will fail if the new file path already exists.
		static void rename_nuke(std::string const& path_old, std::string const& path_new);
		static void rename_safe(std::string const& path_old, std::string const& path_new);

	private:
		static std::string _get_err_str(File const* file, File::OP op);
};


}}
