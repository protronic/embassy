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

// FatFS symbols referenced by Graphics4D.cpp (SD assets) — not used by OxivGL scan-out.
typedef unsigned char BYTE;
typedef unsigned int UINT;
typedef char TCHAR;
typedef int FRESULT;

typedef struct {
    void *fs;
    void *file;
    void *dir;
    void *obj;
} FIL;

typedef struct {
    BYTE dummy;
} FATFS;

STUB_USED FRESULT f_open(FIL *fp, const TCHAR *path, BYTE mode) {
    (void)fp;
    (void)path;
    (void)mode;
    return 1; /* FR_DISK_ERR */
}

STUB_USED FRESULT f_mount(FATFS *fs, const TCHAR *path, BYTE opt) {
    (void)fs;
    (void)path;
    (void)opt;
    return 1;
}

STUB_USED FRESULT f_read(FIL *fp, void *buff, UINT btr, UINT *br) {
    (void)fp;
    (void)buff;
    (void)btr;
    if (br) {
        *br = 0;
    }
    return 1;
}

STUB_USED FRESULT f_lseek(FIL *fp, unsigned long ofs) {
    (void)fp;
    (void)ofs;
    return 1;
}

STUB_USED int atexit(void (*func)(void)) {
    (void)func;
    return 0;
}
