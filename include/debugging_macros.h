//
// Created by karlo on 12/5/2019.
//

#ifndef OMPI_DEBUGGING_MACROS_H
#define OMPI_DEBUGGING_MACROS_H

#include <stdio.h>

#define PRINT_DEBUG printf("Within function: %s in file %s \n", __func__, __FILE__)
#endif //OMPI_DEBUGGING_MACROS_H
