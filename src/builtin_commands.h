/* builtin command list */
#include <nuttx/config.h>
#include <nuttx/binfmt/builtin.h>

extern int buttons_main(int argc, char *argv[]);
extern int hello_main(int argc, char *argv[]);
extern int led_main(int argc, char *argv[]);

const struct builtin_s g_builtins[] = {
    {"buttons_main", SCHED_PRIORITY_DEFAULT, 2048, buttons_main},
	{"hello_main", SCHED_PRIORITY_DEFAULT, 2048, hello_main},
	{"led_main", SCHED_PRIORITY_DEFAULT, 2048, led_main},
    {NULL, 0, 0, NULL}
};
const int g_builtin_count = 3;
