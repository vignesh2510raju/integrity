//
//  Inputs.hpp
//  UserInput
//
//  Created by Guillermo Duenas Arana on 2/11/17.
//  Copyright © 2017 Guille. All rights reserved.
//

#ifndef Inputs_hpp
#define Inputs_hpp

#include <iostream>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <getopt.h>

#include "../header/classes.h"

Parameters GetUserInputs(int argc, char* argv[], Parameters p);

// void showhelpinfo(const char *s);



#endif /* Inputs_hpp */
