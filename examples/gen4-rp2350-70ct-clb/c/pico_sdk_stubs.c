// Stubs for Pico SDK symbols referenced by libgraphics4d_rp2350.a when linked
// into Embassy (no Pico crt0, stdio_usb, or panic implementation).
// Marked used so --gc-sections does not drop them before Graphics4D resolves.

#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>

#define STUB_USED __attribute__((used))

STUB_USED void __unhandled_user_irq(void) {}

STUB_USED void hard_assertion_failure(void) {
    while (1) {
    }
}

STUB_USED void panic(const char *fmt, ...) {
    (void)fmt;
    while (1) {
    }
}

STUB_USED void stdio_usb_init(void) {}

STUB_USED int stdio_printf(const char *fmt, ...) {
    (void)fmt;
    return 0;
}

STUB_USED int vprintf(const char *fmt, va_list ap) {
    (void)fmt;
    (void)ap;
    return 0;
}

// newlib _sbrk used by Pico malloc in the static lib; Embassy heap is separate.
STUB_USED void *_sbrk(ptrdiff_t incr) {
    (void)incr;
    return (void *)-1;
}

STUB_USED void _fini(void) {}
