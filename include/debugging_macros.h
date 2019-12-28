//
// Created by karlo on 12/5/2019.
//

#ifndef OMPI_DEBUGGING_MACROS_H
#define OMPI_DEBUGGING_MACROS_H

#include <linux/module.h>   /* Needed by all modules */
#include <linux/kernel.h>   /* Needed for KERN_INFO */

#define PRINT_DEBUG printk(KERN_INFO "Within function: %s in file %s \n", __func__, __FILE__)
#endif //OMPI_DEBUGGING_MACROS_H
