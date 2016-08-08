/* builtin command list - automatically generated, do not edit */
#include <nuttx/config.h>
#include <nuttx/binfmt/builtin.h>

extern int buttons_main(int argc, char *argv[]);

const struct builtin_s g_builtins[] = {
    {"buttons_main", SCHED_PRIORITY_DEFAULT, 2048, buttons_main},
    {NULL, 0, 0, NULL}
};
const int g_builtin_count = 1;
