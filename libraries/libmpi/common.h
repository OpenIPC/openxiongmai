#ifndef COMMON_H
#define COMMON_H

#define FATAL_ARG(msg, ...)                                                    \
  fprintf(stderr, MODULE " - %s(%d): " msg "\n", __FUNCTION__, __LINE__,       \
          __VA_ARGS__)
#define FATAL(msg)                                                             \
  fprintf(stderr, MODULE " - %s(%d): " msg "\n", __FUNCTION__, __LINE__)

#endif /* COMMON_H */
