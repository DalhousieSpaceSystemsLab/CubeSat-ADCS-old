#ifndef JSON_H
#define JSON_H

/*
    TODO: function to get number of tokens
    TODO: function to convert to key-value pair
*/

#include <iostream>

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
    
    unsigned int get_no_of_tokens();
    template<typename A, typename B>
    ret_val get_token(A x, B& y);
    ret_val serialize();
    ret_val deserialize();

    private:
    char data[MAX_SIZE];
    unsigned int size = 0;
    unsigned int no_of_tokens = -1;
    jtok_tkn_t tokens[MAX_TOKENS];
};



#endif  //JSON_H