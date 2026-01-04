#ifndef _PARAMETERS_PARSE_H_
#define _PARAMETERS_PARSE_H_
#include "percipio_interface.h"

namespace percipio {
    bool isValidJsonString(const char* code);
    bool gige_2_0_load_default_parameters(const TY_DEV_HANDLE hDevice, const char* jscode);
}
#endif
