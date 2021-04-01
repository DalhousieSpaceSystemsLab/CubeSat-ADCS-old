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
        return FAIL;
    }

    return SUCCESS;
}

unsigned int JSON::get_no_of_tokens() {
    if(tokens[0].size < 1) {
        return -1;                                          // JSON is empty
    }
    else {
        std::queue<int> tokq;                               // queue to hold unvisited nodes
        tokq.push(0);                                       // start from the root object
        unsigned int total = 1;                             // total no. of counted nodes

        // loop while there are unvisited nodes
        while(tokq.size() != 0) {
            unsigned int parent = tokq.front();             // take the first and make it the parent
            unsigned tok_to_visit = tokens[parent].size;    // no of childs to visit
            tokq.pop();                                     // pop the parent since it has been visited
            unsigned int limit = 1;                         // to make sure we don't go over MAX_TOKENS
            while(tok_to_visit > 0 && limit < MAX_TOKENS) { // loop till we find all the children
                if(tokens[limit].parent == parent) {
                    total++;
                    tok_to_visit--;
                    tokq.push(limit);                       // if a children is found, push it on the stack
                }
                limit++;
            }
        }
        size = total;
        return total;
    }
}

unsigned int JSON::find_key(const char *key, unsigned int root = 1) {

    int key_size = strlen(key);
    if(key_size == 0) {
        return -1;
    }

    // int answer = -1;
    for(int i = root; i < size; i++) {
        bool found  = false;
        if(tokens[i].type == JTOK_STRING && jtok_tokcmp(key, &tokens[i])) {
            // int t = strncmp(&data[tokens[i].start], key, key_size);
            // if(t == 0) {
            //     found = true;
            // }
            // if(jtok_tokcmp(key, &tokens[i])) {
                return i;
            // }
        }
        // if(found) {
        //     // answer = i;             // token with key found
        //     // break;
        //     return i;
        // }
    }
    return -1;
}

ret_val JSON::get_token(const char *key, uint &value, unsigned int root /* default = 1*/) {
    
    if(strlen(key) == 0) {
        return ERR_INVALID_ARG;
    }

    // find out whether any token matches our key
    unsigned int answer = find_key(key, root);
    if(answer == -1 || tokens[answer].size != 1) {
        return FAIL;                
    }

    // now find the child/value of the key
    int child = -1;
    for(int i = 0; i < size; i++) {
        if(tokens[i].parent == answer) {
            child = i;
            break;
        }
    }

    // if child is not a string or a number, abort
    if(!(tokens[child].type == JTOK_STRING || tokens[child].type == JTOK_PRIMITIVE)) {
        return FAIL;
    }

    // value.append(&data[tokens[child].start], jtok_toklen(&tokens[child]));
    value = child;
    return SUCCESS;
}

ret_val JSON::get_token(const char *key, const char *key2, uint &value, unsigned int root /* default = 1*/) {

    if(strlen(key) == 0 || strlen(key2) == 0) {
        return ERR_INVALID_ARG;
    }

    // find out whether any token matches our key
    unsigned int answer = find_key(key);
    if(answer == -1 || tokens[answer].size != 1) {
        return FAIL;                
    }

    if(tokens[answer].size == 0) {
        return FAIL;
    }

    return get_token(key2, value, answer);
}

ret_val JSON::get_token(const char *key, const char *key2, const char *key3, uint &value, unsigned int root /* default = 1*/) {

    if(strlen(key) == 0 || strlen(key2) == 0 || strlen(key3) == 0) {
        return ERR_INVALID_ARG;
    }

    // find out whether any token matches our key
    unsigned int answer = find_key(key);
    if(answer == -1 || tokens[answer].size != 1) {
        return FAIL;                
    }
    if(tokens[answer].size == 0) {
        return FAIL;
    }

    answer = find_key(key2, answer);
    if(answer == -1 || tokens[answer].size != 1) {
        return FAIL;                
    }
    if(tokens[answer].size == 0) {
        return FAIL;
    }

    return get_token(key3, value, answer);
}

ret_val JSON::get_token(const char *key, const char *key2, const char *key3, const char *key4, uint &value) {

    if(strlen(key) == 0 || strlen(key2) == 0 || strlen(key3) == 0 || strlen(key4) == 0) {
        return ERR_INVALID_ARG;
    }

    // find out whether any token matches our key
    unsigned int answer = find_key(key);
    if(answer == -1 || tokens[answer].size != 1) {
        return FAIL;                
    }
    if(tokens[answer].size == 0) {
        return FAIL;
    }

    answer = find_key(key2, answer);
    if(answer == -1 || tokens[answer].size != 1) {
        return FAIL;                
    }
    if(tokens[answer].size == 0) {
        return FAIL;
    }

    answer = find_key(key3, answer);
    if(answer == -1 || tokens[answer].size != 1) {
        return FAIL;                
    }
    if(tokens[answer].size == 0) {
        return FAIL;
    }

    return get_token(key4, value, answer);
}

ret_val JSON::get_value_str(uint index, std::string &value){
    if(index == 0 || index >= size || tokens[index].type != JTOK_STRING) {
        return ERR_INVALID_ARG;
    }
    else {
        value = std::string(&data[tokens[index].start], jtok_toklen(&tokens[index]));
        return SUCCESS;
    }
}

ret_val JSON::get_value_num(uint index, double &value) {
    if(index == 0 || index >= size || tokens[index].type != JTOK_PRIMITIVE) {
        return ERR_INVALID_ARG;
    }
    else {
        int len = jtok_toklen(&tokens[index]) + 1;
        char c[len];
        c[jtok_toklen(&tokens[index])] = '\0';
        int start = tokens[index].start;
        strncpy(c, &data[start], len-1);
        value = strtod(c, NULL);
        return SUCCESS;
    }
}

ret_val JSON::get_value_bool(uint index, bool &value) {
    if(index == 0 || index >= size || tokens[index].type != JTOK_PRIMITIVE) {
        return ERR_INVALID_ARG;
    }
    else {
        int start = tokens[index].start;
        if(memcmp(&data[start], "true", 4) == 0) {
            value = true;
            return SUCCESS;
        }
        if(memcmp(&data[start], "false", 5) == 0) {
            value = false;
            return SUCCESS;
        }
        return FAIL;
    }
}
