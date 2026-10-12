// See schema.h. The shape is the one /api/v1/config.schema.json serves, so
// what goes in here is what a contributor reads back out of the camera.

#include "schema.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    char *buf;
    size_t cap, len;
    bool ok;
} Out;

static void put(Out *o, const char *fmt, ...) {
    if (!o->ok) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(o->buf + o->len, o->cap - o->len, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= o->cap - o->len) {
        o->ok = false;
        return;
    }
    o->len += (size_t)n;
}

// A JSON string. The names are tokens and the titles are literals in
// actuator.c, but a quote in a title written there one day must not turn the
// whole description into something the core refuses.
static void put_str(Out *o, const char *s) {
    put(o, "\"");
    for (; *s; s++) {
        if (*s == '"' || *s == '\\') {
            put(o, "\\%c", *s);
        } else if ((unsigned char)*s < 0x20) {
            put(o, "\\u%04x", (unsigned char)*s);
        } else {
            put(o, "%c", *s);
        }
    }
    put(o, "\"");
}

bool schema_write(char *buf, size_t cap,
                  const ActuatorChoice *(*choice)(unsigned i)) {
    if (!buf || !cap) {
        return false;
    }
    Out o = {buf, cap, 0, true};
    const ActuatorChoice *c;

    put(&o, "{\"properties\":{\"actuator\":{\"enum\":[");
    for (unsigned i = 0; (c = choice(i)); i++) {
        put(&o, i ? "," : "");
        put_str(&o, c->name);
    }
    put(&o, "],\"x-enum-titles\":{");
    for (unsigned i = 0; (c = choice(i)); i++) {
        put(&o, i ? "," : "");
        put_str(&o, c->name);
        put(&o, ":");
        put_str(&o, c->title);
    }
    put(&o, "}}}}");

    if (!o.ok) {
        buf[0] = '\0';
    }
    return o.ok;
}
