/* Minimal host-test assert harness (host gcc only; not part of src/). */
#ifndef CHECK_H
#define CHECK_H

#include <stdio.h>
#include <math.h>

static int g_checks = 0;
static int g_fails = 0;

#define CHECK(cond)                                                        \
    do {                                                                   \
        g_checks++;                                                        \
        if (!(cond)) {                                                     \
            g_fails++;                                                     \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
        }                                                                  \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                              \
    do {                                                                   \
        g_checks++;                                                        \
        if (fabsf((float)(a) - (float)(b)) > (float)(tol)) {              \
            g_fails++;                                                      \
            printf("  FAIL %s:%d: |%g - %g| > %g\n", __FILE__, __LINE__,   \
                   (double)(a), (double)(b), (double)(tol));               \
        }                                                                  \
    } while (0)

#define CHECK_DONE()                                                       \
    do {                                                                   \
        printf("%s: %d checks, %d failed\n", __FILE__, g_checks, g_fails); \
        return g_fails ? 1 : 0;                                            \
    } while (0)

#endif /* CHECK_H */
