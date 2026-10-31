#ifndef WEB_HTTP_H
#define WEB_HTTP_H
#include <stdbool.h>
#include <stddef.h>
#include "lwip/apps/fs.h"

// Application adapter. Open returns 0 for unknown routes. Command returns an
// HTTP status and, on success, a stable response URI valid through POST finish.
int app_http_open(struct fs_file *file, const char *uri);
int app_http_command(const char *uri, const char **response_uri);

// Static responses must outlive their connections. JSON writers receive a
// connection-owned buffer; the common HTTP module frees it at fs_close_custom.
int web_http_text(struct fs_file *file, const char *response);
int web_http_json(struct fs_file *file, size_t capacity, bool (*write)(char *, size_t));
#endif
