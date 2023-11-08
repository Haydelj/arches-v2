#include "string.hpp"


namespace Arches { namespace Util {


#if 0
size_t normalize_newlines(char* source) {
	//First loop skips until the first '\r' is reached.  If found, jumps into second loop,
	//	which shifts over characters in "source", in-place.

	char c;
	size_t index_read=0, index_write;

	LOOP1: {
		char c = source[index_read];
		switch (c) {
			default:
				++index_read;
				goto LOOP1;
			case '\r':
				index_write = index_read;
				goto CASE_R;
			case '\0':
				return 0; //This string contains no '\r' characters to remove!
		}
	}

	LOOP2: {
		c = source[index_read];
		switch (c) {
			default:
				source[index_write++] = c;
				//Fallthrough
			CASE_R: case '\r':
				++index_read;
				goto LOOP2;
			case '\0':
				break;
		}
	}

	source[index_write] = '\0';

	assert(index_read>=index_write);
	return index_read - index_write;
}
#endif

std::deque<std::string> get_split(std::string const& main_string, std::string const& test_string) {
	assert(!test_string.empty());

	std::deque<std::string> result;

	size_t offset = 0;
	LOOP:
		size_t loc = main_string.find(test_string,offset);
		if (loc!=main_string.npos) {
			assert(offset<=loc);
			result.emplace_back(main_string.substr(offset,loc-offset));
			offset = loc + test_string.size();

			goto LOOP;
		}
	result.emplace_back(main_string.substr(offset));

	return result;
}

#if 0
bool   contains(       char const* main_string,               char test_char  ) {
	LOOP:
		char c = *main_string;
		if (c==test_char) return true;
		else if (c!='\0') {
			++main_string;
			goto LOOP;
		}
	return false;
}
bool   contains(std::string const& main_string,               char test_char  ) {
	for (char c : main_string) {
		if (c == test_char) return true;
	}
	return false;
}
bool   contains(       char const* main_string,        char const* test_string) {
	size_t len_main = strlen(main_string);
	size_t len_test = strlen(test_string);
	for (size_t shift=0;shift<len_main-len_test+1;++shift) {
		if (memcmp(main_string,test_string,len_test)!=0);
		else return true;
	}
	return false;
}
bool   contains(std::string const& main_string, std::string const& test_string) {
	return main_string.find(test_string) != std::string::npos;
}
bool startswith(       char const* main_string,        char const* test_string) {
	//TODO: test which is faster
	#if 0
		while (*test_string!='\0') {
			if (*main_string!='\0') {
				if (*main_string==*test_string) {
					++main_string;
					++test_string;
				} else return false;
			} else return false;
		}
		return true;
	#else
		int i = 0;
		LOOP:
			char c1 = main_string[i];
			char c2 = test_string[i];

			if (c2!='\0'); else return true;
			if (c1!='\0' && c1==c2) {
				++i;
				goto LOOP;
			}
		return false;
	#endif
}
bool startswith(std::string const& main_string, std::string const& test_string) {
	if (test_string.length()>main_string.length()) return false;
	if (main_string.find(test_string)==0) return true;
	return false;
}
bool   endswith(       char const* main_string,        char const* test_string) {
	size_t len_main = strlen(main_string);
	size_t len_test = strlen(test_string);
	if (len_test<=len_main); else return false;
	return memcmp(main_string+len_main-len_test,test_string,len_test) == 0;
}
bool   endswith(std::string const& main_string, std::string const& test_string) {
	if (test_string.length()<=main_string.length()); else return false;
	if (main_string.rfind(test_string)==main_string.length()-test_string.length()) return true;
	return false;
}

size_t find_last_of(std::string const& main_string, char const* search_chars, size_t min_i,size_t max_i) {
	if (!main_string.empty()) {
		assert(min_i<main_string.length()||min_i==std::string::npos);
		if (min_i!=std::string::npos);
		else min_i=0;

		assert(max_i<=main_string.length()||max_i==std::string::npos);
		if (max_i!=std::string::npos);
		else max_i=main_string.length();

		assert(min_i<max_i);

		assert(
			min_i<main_string.length() && max_i<=main_string.length()
		);

		//TODO: edit algorithm so that we don't need to cast to `int`; which is done to prevent
		//	unsigned wraparound.
		assert(
			static_cast<size_t>(static_cast<int>(min_i))==min_i &&
			static_cast<size_t>(static_cast<int>(max_i))==max_i
		);
		for (int i=static_cast<int>(max_i)-1; i>=static_cast<int>(min_i); --i) {
			char c = main_string[static_cast<size_t>(i)];

			if (contains(search_chars,c)) return static_cast<size_t>(i);
		}
	}
	return std::string::npos;
}
#endif

std::string get_replaced(std::string const& str_input, std::string const& str_to_find, std::string const& str_to_replace_with) {
	std::string str_input_copy = str_input;
	replace(&str_input_copy,str_to_find,str_to_replace_with);
	return str_input_copy;
}
void             replace(std::string*       str_input, std::string const& str_to_find, std::string const& str_to_replace_with) {
	size_t pos = 0;
	size_t find_length = str_to_find.length();
	size_t replace_length = str_to_replace_with.length();
	if (find_length==0) return;
	for (;(pos=str_input->find(str_to_find,pos))!=std::string::npos;) {
		str_input->replace(pos, find_length, str_to_replace_with);
		pos += replace_length;
	}
}


}}
