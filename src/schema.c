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

// The "enum" and "x-enum-titles" members of one key, from its list.
static void put_choices(Out *o, const ActuatorChoice *(*choice)(unsigned i)) {
    const ActuatorChoice *c;
    put(o, "\"enum\":[");
    for (unsigned i = 0; (c = choice(i)); i++) {
        put(o, i ? "," : "");
        put_str(o, c->name);
    }
    put(o, "],\"x-enum-titles\":{");
    for (unsigned i = 0; (c = choice(i)); i++) {
        put(o, i ? "," : "");
        put_str(o, c->name);
        put(o, ":");
        put_str(o, c->title);
    }
    put(o, "}");
}

bool schema_write(char *buf, size_t cap,
                  const ActuatorChoice *(*choice)(unsigned i),
                  const ActuatorChoice *(*head)(unsigned i)) {
    if (!buf || !cap) {
        return false;
    }
    Out o = {buf, cap, 0, true};

    // actuator is the core's key: only its list is ours to give.
    put(&o, "{\"properties\":{\"actuator\":{");
    put_choices(&o, choice);
    put(&o, "}");
    // head is ours entirely, so it carries what a plugin key must: a type, a
    // default and a title. Its default is the first choice, "none", which is
    // every camera with one motor.
    if (head && head(0)) {
        put(&o, ",\"head\":{\"type\":\"string\",\"default\":");
        put_str(&o, head(0)->name);
        put(&o, ",\"title\":\"Pan/tilt head\",");
        put_choices(&o, head);
        put(&o, ",\"help\":\"A pan/tilt base on its own wire, beside the lens "
                "named above: pan and tilt go to it, zoom and focus to the lens. "
                "It uses the serial port, speed and address set here.\"}");
    }
    // The two directions a pan/tilt motor may turn the wrong way from the
    // picture (motion.c, mo_pan_invert): off on every camera but one that says so.
    // The plugin's own keys, like head: written with it.
    if (head) {
        put(&o, ",\"panInvert\":{\"type\":\"boolean\",\"default\":false,"
                "\"title\":\"Pan inverted\",\"help\":\"Turn the head the other way "
                "for left and right: for a head mounted so that the arrows move it "
                "backwards even with the picture upright.\"}");
        put(&o, ",\"tiltInvert\":{\"type\":\"boolean\",\"default\":false,"
                "\"title\":\"Tilt inverted\",\"help\":\"Turn the head the other way "
                "for up and down.\"}");
        // The head's stored positions (motion.c, motion_preset): how many are offered,
        // and how long a go-to holds the wire while the head travels.
        put(&o, ",\"presetMax\":{\"type\":\"integer\",\"default\":32,"
                "\"minimum\":0,\"maximum\":255,\"title\":\"Presets offered\","
                "\"help\":\"Stored positions 1 to this number, on a head that has them "
                "(Pelco-D). Many bases use 33 and up for menu and command calls; 0 offers "
                "none.\"}");
        put(&o, ",\"presetMs\":{\"type\":\"integer\",\"default\":4000,"
                "\"minimum\":500,\"maximum\":30000,\"x-unit\":\"ms\","
                "\"title\":\"Preset travel time\",\"help\":\"How long a go-to-preset "
                "holds the head before another command may take it: about the longest "
                "swing between two presets.\"}");
        // The MS41908M lens's geometry (act_ms41908.c, ms_lens_from_config). The
        // defaults are the 16x LH13 lens's, so a board that sets none is as before.
        put(&o, ",\"lensZoomSteps\":{\"type\":\"integer\",\"default\":2610,"
                "\"minimum\":200,\"maximum\":20000,\"title\":\"Lens zoom travel\","
                "\"x-unit\":\"steps\",\"help\":\"MS41908 lens only: zoom microsteps "
                "from one end stop to the other.\"}");
        put(&o, ",\"lensFocusSteps\":{\"type\":\"integer\",\"default\":1280,"
                "\"minimum\":200,\"maximum\":20000,\"title\":\"Lens focus travel\","
                "\"x-unit\":\"steps\",\"help\":\"MS41908 lens only: focus microsteps "
                "from one end stop to the other.\"}");
        put(&o, ",\"lensZoomMax\":{\"type\":\"integer\",\"default\":16,"
                "\"minimum\":2,\"maximum\":60,\"title\":\"Lens zoom ratio\","
                "\"x-unit\":\"x\",\"help\":\"MS41908 lens only: the magnification "
                "at the tele end.\"}");
        put(&o, ",\"lensZoomReversed\":{\"type\":\"boolean\",\"default\":false,"
                "\"title\":\"Lens zoom reversed\",\"help\":\"MS41908 lens only: the "
                "zoom motor's home stop is the wide end, not the tele end.\"}");
        put(&o, ",\"lensSwap\":{\"type\":\"boolean\",\"default\":false,"
                "\"title\":\"Lens motors swapped\",\"help\":\"MS41908 lens only: "
                "zoom is on the focus motor and focus on the zoom motor.\"}");
        put(&o, ",\"lensHome\":{\"type\":\"boolean\",\"default\":false,"
                "\"title\":\"Home the lens at start\",\"help\":\"MS41908 lens only: "
                "drive zoom and focus into their end stops when the lens opens, then "
                "zoom back to where it was, so no position is ever a guess.\"}");
    }
    put(&o, "}}");

    if (!o.ok) {
        buf[0] = '\0';
    }
    return o.ok;
}
