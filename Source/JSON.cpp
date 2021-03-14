#include "JSON.h"

JSON::JSON(std::string input) {
    input.copy(data, input.length());
    size = input.size();
}

ret_val JSON::deserialize() {

    JTOK_PARSE_STATUS_t status;

    status = jtok_parse(data, tokens, MAX_TOKENS);
    
    if (status != JTOK_PARSE_STATUS_OK)
    {
        return ERR_INVALID_ARG;
    }

    return SUCCESS;
}

// unsigned int JSON::get_no_of_tokens() {
//     if(no_of_tokens == -1) {
//         for(int i = 0; i < MAX_TOKENS; i++) {
//             if(tokens[i].
//         }
//     }
// }
