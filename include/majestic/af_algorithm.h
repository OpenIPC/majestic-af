#ifndef MAJESTIC_AF_ALGORITHM_H
#define MAJESTIC_AF_ALGORITHM_H

enum AfAlgorithm {
    AF_ALGORITHM_INVALID = 0,
    AF_ALGORITHM_BLIND_SEEK,
    AF_ALGORITHM_AF2,
};

enum AfAlgorithm af_algorithm_parse(const char *name);

#endif
