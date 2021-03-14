//this is the main function used to run our system
#include <iostream>

#include "CubeSat_ADCSConfig.h"
#include "JSON.h"

std::string json_test = "{\"name\" : \"Jack\", \"age\" : 27}";

int main(){
    
    JSON j = JSON(json_test);
    std::cout << j.deserialize() << std::endl;
    return 0;
}