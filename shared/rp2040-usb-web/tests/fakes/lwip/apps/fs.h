#ifndef TEST_FS_H
#define TEST_FS_H
#include <stddef.h>
#define FS_FILE_FLAGS_HEADER_INCLUDED 1
struct fs_file { const char *data; int len, index; unsigned flags; void *pextension; };
int fs_open_custom(struct fs_file *file, const char *name);
void fs_close_custom(struct fs_file *file);
#endif
