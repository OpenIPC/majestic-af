#include <majestic/af_algorithm.h>

#include <string.h>

enum AfAlgorithm af_algorithm_parse(const char *name) {
    if (!name) return AF_ALGORITHM_INVALID;
    if (!strcmp(name, "blind_seek")) return AF_ALGORITHM_BLIND_SEEK;
    if (!strcmp(name, "af2")) return AF_ALGORITHM_AF2;
    return AF_ALGORITHM_INVALID;
}
