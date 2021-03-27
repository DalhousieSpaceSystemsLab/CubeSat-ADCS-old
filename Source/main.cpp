//this is the main function used to run our system
#include <iostream>

#include "CubeSat_ADCSConfig.h"
#include "JSON.h"

std::string json_test = "{\"name\" : \"Jack\", \"age\" : 27, \"age\" : 27, \"age\" : 27, \"age\" : 27, \"age\" : 27}";

std::string json_test2 = "{\"menu\": { \"id\": \"file\", \"value\": \"File\", \"popup\": {\"menuitem\": [{\"value\": \"New\", \"onclick\": \"CreateNewDoc()\"}, {\"value2\": \"Open\", \"onclick\": \"OpenDoc()\"}, {\"value3\": \"Close\", \"onclick\": \"CloseDoc()\"}]}}}";

std::string json_test3 = "{\"menu\": {\"id\": \"file\", \"value\": \"File\", \"popup\": {\"menuitem\": {\"key1\": \"value1\", \"key2\": {\"key3\": \"value3\"}}}}}";

int main(){
    JSON j = JSON(json_test3);
    std::cout << j.deserialize() << std::endl;
    // std::string s = j.get_string("name");
    // std::cout <<  << std::endl;
    std::cout << j.get_no_of_tokens() << std::endl;

    // std::string s;
    // std::string &sr = s;
    // ret_val x = j.get_string("onclick", sr);
    // std::cout << x << "  " << sr << std::endl << std::flush;

    // std::string s1;
    // std::string &sr1 = s1;
    // ret_val y = j.get_string("menuitem", "value", sr1);
    // std::cout << y << "  " << sr1 << std::endl << std::flush;

    // std::string s2;
    // std::string &sr2 = s2;
    // ret_val z = j.get_string("popup", "menuitem", "value2", sr2);
    // std::cout << z << "  " << sr2 << std::endl << std::flush;

    std::string s3;
    std::string &sr3 = s3;
    ret_val a = j.get_string("menu", "popup", "menuitem", "key1", sr3);
    std::cout << a << "  " << sr3 << std::endl << std::flush;
    return 0;
}