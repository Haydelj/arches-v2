#pragma once

#include "stdafx.hpp"


namespace Arches { namespace Util {


//size_t normalize_newlines(char* source);

std::deque<std::string> get_split(std::string const& main_string, std::string const& test_string);

#if 0
bool   contains(       char const* main_string,               char test_char  );
bool   contains(std::string const& main_string,               char test_char  );
bool   contains(       char const* main_string,        char const* test_string);
bool   contains(std::string const& main_string, std::string const& test_string);
bool startswith(       char const* main_string,        char const* test_string);
bool startswith(std::string const& main_string, std::string const& test_string);
bool   endswith(       char const* main_string,        char const* test_string);
bool   endswith(std::string const& main_string, std::string const& test_string);

size_t find_last_of(std::string const& main_string, char const* search_chars, size_t min_i,size_t max_i);
#endif

std::string get_replaced(std::string const& str_input, std::string const& str_to_find, std::string const& str_to_replace_with);
void             replace(std::string*       str_input, std::string const& str_to_find, std::string const& str_to_replace_with);


}}
