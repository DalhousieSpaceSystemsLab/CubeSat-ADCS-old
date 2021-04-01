#ifndef JSON_H
#define JSON_H

/*
    TODO: function to get number of tokens
    TODO: function to convert to key-value pair
*/

#include <iostream>
#include <cstdarg>
#include <queue>
#include <string.h>

#include "Errors.h"
#include "jtok.h"

const unsigned int MAX_TOKENS = 50;
const unsigned int MAX_SIZE = 250;

// struct token
// {
//     JTOK_TYPE_t key_type;
//     JTOK_VALUE_TYPE_t value_type;
// };


class JSON
{
    public:
    JSON(std::string input);
    
    // get functions for nesting upto 5 levels, function overloading is used instead 
    // of operator overloading or variadic for simplicity
    ret_val get_token(const char *, uint &, unsigned int = 1);
    ret_val get_token(const char *, const char *, uint &, unsigned int = 1);
    ret_val get_token(const char *, const char *, const char *, uint &, unsigned int = 1);
    ret_val get_token(const char *, const char *, const char *, const char *, uint &);

    ret_val get_value_str(uint index, std::string &);
    ret_val get_value_num(uint index, double &);
    ret_val get_value_bool(uint index, bool &);

    unsigned int get_no_of_tokens();
    ret_val serialize();
    ret_val deserialize();

    private:
    char data[MAX_SIZE];
    unsigned int size = 0;
    unsigned int no_of_tokens = -1;
    jtok_tkn_t tokens[MAX_TOKENS];

    unsigned int find_key(const char *, unsigned int);
    
};



#endif  //JSON_H