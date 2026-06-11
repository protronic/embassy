#pragma once

#include "tlsf/tlsf.h"
#include "stdio.h"

#include "hardware/sync.h"
#include "hardware/regs/qmi.h"
#include "hardware/regs/xip.h"
#include "hardware/structs/qmi.h"
#include "hardware/structs/xip_ctrl.h"

#define CIRCUITPY_EXCEPTION_STACK_SIZE      1024
#define CIRCUITPY_DEFAULT_STACK_SIZE        (24 * 1024)

static tlsf_t _heap = NULL;
static pool_t _ram_pool = NULL;
static pool_t _psram_pool = NULL;
static size_t _psram_size = 0;

static uint32_t gpio_bank0_pin_claimed;
static uint32_t never_reset_pins;

volatile uint32_t nesting_count = 0;
void common_hal_mcu_disable_interrupts(void) {
    // Print debug message before disabling interrupts
    printf("Before disabling interrupts. Nesting count: %u\n", nesting_count);

    // Disable interrupts
    asm volatile (
        "cpsid i\n\t" // Disable interrupts
        // "isb\n\t"    // Instruction Synchronization Barrier (optional)
        : : : "memory"
    );
    __dmb(); // Ensure all previous memory operations are completed

    // Increment nesting count
    nesting_count++;

    // Print debug message after disabling interrupts
    printf("Interrupts disabled. New nesting count: %u\n", nesting_count);
}

void common_hal_mcu_enable_interrupts(void) {
    if (nesting_count == 0) {
        // reset_into_safe_mode(SAFE_MODE_INTERRUPT_ERROR);
    }
    nesting_count--;
    if (nesting_count > 0) {
        return;
    }
    __dmb();
    asm volatile ("cpsie i" : : : "memory");
}

bool board_reset_pin_number(uint8_t pin_number) {
    if (pin_number == 18) {
        // doing this (rather than gpio_init) in this specific order ensures no
        // glitch if pin was already configured as a high output. gpio_init() temporarily
        // configures the pin as an input, so the power enable value would potentially
        // glitch.
        gpio_put(pin_number, 1);
        gpio_set_dir(pin_number, GPIO_OUT);
        gpio_set_function(pin_number, GPIO_FUNC_SIO);

        return true;
    }
    return false;
}

void never_reset_pin_number(uint8_t pin_number) {
    if (pin_number >= NUM_BANK0_GPIOS) {
        return;
    }

    never_reset_pins |= 1 << pin_number;
}

void reset_pin_number(uint8_t pin_number) {
    if (pin_number >= NUM_BANK0_GPIOS) {
        return;
    }

    gpio_bank0_pin_claimed &= ~(1 << pin_number);
    never_reset_pins &= ~(1 << pin_number);

    // Allow the board to override the reset state of any pin
    if (board_reset_pin_number(pin_number)) {
        return;
    }

    // We are very aggressive in shutting down the pad fully. Both pulls are
    // disabled and both buffers are as well.
    gpio_init(pin_number);
    hw_clear_bits(&pads_bank0_hw->io[pin_number], PADS_BANK0_GPIO0_IE_BITS |
        PADS_BANK0_GPIO0_PUE_BITS |
        PADS_BANK0_GPIO0_PDE_BITS);
    hw_set_bits(&pads_bank0_hw->io[pin_number], PADS_BANK0_GPIO0_OD_BITS);
}

// From the linker script
extern uint32_t _ld_cp_dynamic_mem_start;
extern uint32_t _ld_cp_dynamic_mem_end;

uint32_t *port_stack_get_top(void) {
    return &_ld_cp_dynamic_mem_end;
}

uint32_t *port_heap_get_bottom(void) {
    return &_ld_cp_dynamic_mem_start;
}

uint32_t *port_stack_get_limit(void) {
    #pragma GCC diagnostic push

    #pragma GCC diagnostic ignored "-Warray-bounds"
    return port_stack_get_top() - (CIRCUITPY_DEFAULT_STACK_SIZE + CIRCUITPY_EXCEPTION_STACK_SIZE) / sizeof(uint32_t);
    #pragma GCC diagnostic pop
}

uint32_t *port_heap_get_top(void) {
    return port_stack_get_limit();
}


uint32_t __uninitialized_ram(saved_word);
void port_set_saved_word(uint32_t value) {
    // Store in RAM because the watchdog scratch registers don't survive
    // resetting by pulling the RUN pin low.
    saved_word = value;
}

uint32_t port_get_saved_word(void) {
    return saved_word;
}

static void setup_psram() {
    
    gpio_set_function(0, GPIO_FUNC_XIP_CS1);
    _psram_size = 0;
    printf("Setting up PSRAM...\n");
    // common_hal_mcu_disable_interrupts();
    save_and_disable_interrupts();
    printf("Disabled interrupts...\n");

    // Try and read the PSRAM ID via direct_csr.
    qmi_hw->direct_csr = 30 << QMI_DIRECT_CSR_CLKDIV_LSB |
        QMI_DIRECT_CSR_EN_BITS;

    printf("Waiting for last XIP to expire... ");
    // Need to poll for the cooldown on the last XIP transfer to expire
    // (via direct-mode BUSY flag) before it is safe to perform the first
    // direct-mode operation
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
    }

    printf("done\n");

    // Exit out of QMI in case we've inited already
    qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    // Transmit as quad.
    qmi_hw->direct_tx = QMI_DIRECT_TX_OE_BITS |
        QMI_DIRECT_TX_IWIDTH_VALUE_Q << QMI_DIRECT_TX_IWIDTH_LSB |
        0xf5;
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
    }
    (void)qmi_hw->direct_rx;
    qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS);

    // Read the id
    qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
    uint8_t kgd = 0;
    uint8_t eid = 0;
    for (size_t i = 0; i < 7; i++) {
        if (i == 0) {
            qmi_hw->direct_tx = 0x9f;
        } else {
            qmi_hw->direct_tx = 0xff;
        }
        while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_TXEMPTY_BITS) == 0) {
        }
        while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
        }
        if (i == 5) {
            kgd = qmi_hw->direct_rx;
        } else if (i == 6) {
            eid = qmi_hw->direct_rx;
        } else {
            (void)qmi_hw->direct_rx;
        }
    }
    // Disable direct csr.
    qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS | QMI_DIRECT_CSR_EN_BITS);

    if (kgd != 0x5D) {
        common_hal_mcu_enable_interrupts();
        reset_pin_number(0);
        return;
    }
    never_reset_pin_number(0);

    // Enable quad mode.
    qmi_hw->direct_csr = 30 << QMI_DIRECT_CSR_CLKDIV_LSB |
        QMI_DIRECT_CSR_EN_BITS;
    // Need to poll for the cooldown on the last XIP transfer to expire
    // (via direct-mode BUSY flag) before it is safe to perform the first
    // direct-mode operation
    while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
    }

    // RESETEN, RESET and quad enable
    for (uint8_t i = 0; i < 3; i++) {
        qmi_hw->direct_csr |= QMI_DIRECT_CSR_ASSERT_CS1N_BITS;
        if (i == 0) {
            qmi_hw->direct_tx = 0x66;
        } else if (i == 1) {
            qmi_hw->direct_tx = 0x99;
        } else {
            qmi_hw->direct_tx = 0x35;
        }
        while ((qmi_hw->direct_csr & QMI_DIRECT_CSR_BUSY_BITS) != 0) {
        }
        qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS);
        for (size_t j = 0; j < 20; j++) {
            asm ("nop");
        }
        (void)qmi_hw->direct_rx;
    }
    // Disable direct csr.
    qmi_hw->direct_csr &= ~(QMI_DIRECT_CSR_ASSERT_CS1N_BITS | QMI_DIRECT_CSR_EN_BITS);

    qmi_hw->m[1].timing =
        QMI_M0_TIMING_PAGEBREAK_VALUE_1024 << QMI_M0_TIMING_PAGEBREAK_LSB | // Break between pages.
            3 << QMI_M0_TIMING_SELECT_HOLD_LSB | // Delay releasing CS for 3 extra system cycles.
            1 << QMI_M0_TIMING_COOLDOWN_LSB |
            1 << QMI_M0_TIMING_RXDELAY_LSB |
            16 << QMI_M0_TIMING_MAX_SELECT_LSB | // In units of 64 system clock cycles. PSRAM says 8us max. 8 / 0.00752 / 64 = 16.62
            7 << QMI_M0_TIMING_MIN_DESELECT_LSB | // In units of system clock cycles. PSRAM says 50ns.50 / 7.52 = 6.64
            2 << QMI_M0_TIMING_CLKDIV_LSB;
    qmi_hw->m[1].rfmt = (QMI_M0_RFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_PREFIX_WIDTH_LSB |
            QMI_M0_RFMT_ADDR_WIDTH_VALUE_Q << QMI_M0_RFMT_ADDR_WIDTH_LSB |
            QMI_M0_RFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_RFMT_SUFFIX_WIDTH_LSB |
            QMI_M0_RFMT_DUMMY_WIDTH_VALUE_Q << QMI_M0_RFMT_DUMMY_WIDTH_LSB |
            QMI_M0_RFMT_DUMMY_LEN_VALUE_24 << QMI_M0_RFMT_DUMMY_LEN_LSB |
            QMI_M0_RFMT_DATA_WIDTH_VALUE_Q << QMI_M0_RFMT_DATA_WIDTH_LSB |
            QMI_M0_RFMT_PREFIX_LEN_VALUE_8 << QMI_M0_RFMT_PREFIX_LEN_LSB |
            QMI_M0_RFMT_SUFFIX_LEN_VALUE_NONE << QMI_M0_RFMT_SUFFIX_LEN_LSB);
    qmi_hw->m[1].rcmd = 0xeb << QMI_M0_RCMD_PREFIX_LSB |
        0 << QMI_M0_RCMD_SUFFIX_LSB;
    qmi_hw->m[1].wfmt = (QMI_M0_WFMT_PREFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_PREFIX_WIDTH_LSB |
            QMI_M0_WFMT_ADDR_WIDTH_VALUE_Q << QMI_M0_WFMT_ADDR_WIDTH_LSB |
            QMI_M0_WFMT_SUFFIX_WIDTH_VALUE_Q << QMI_M0_WFMT_SUFFIX_WIDTH_LSB |
            QMI_M0_WFMT_DUMMY_WIDTH_VALUE_Q << QMI_M0_WFMT_DUMMY_WIDTH_LSB |
            QMI_M0_WFMT_DUMMY_LEN_VALUE_NONE << QMI_M0_WFMT_DUMMY_LEN_LSB |
            QMI_M0_WFMT_DATA_WIDTH_VALUE_Q << QMI_M0_WFMT_DATA_WIDTH_LSB |
            QMI_M0_WFMT_PREFIX_LEN_VALUE_8 << QMI_M0_WFMT_PREFIX_LEN_LSB |
            QMI_M0_WFMT_SUFFIX_LEN_VALUE_NONE << QMI_M0_WFMT_SUFFIX_LEN_LSB);
    qmi_hw->m[1].wcmd = 0x38 << QMI_M0_WCMD_PREFIX_LSB |
        0 << QMI_M0_WCMD_SUFFIX_LSB;

    common_hal_mcu_enable_interrupts();

    _psram_size = 1024 * 1024; // 1 MiB
    uint8_t size_id = eid >> 5;
    if (eid == 0x26 || size_id == 2) {
        _psram_size *= 8;
    } else if (size_id == 0) {
        _psram_size *= 2;
    } else if (size_id == 1) {
        _psram_size *= 4;
    }

    // Mark that we can write to PSRAM.
    xip_ctrl_hw->ctrl |= XIP_CTRL_WRITABLE_M1_BITS;
}

void port_heap_init(void) {
    uint32_t *heap_bottom = port_heap_get_bottom();
    uint32_t *heap_top = port_heap_get_top();
    size_t size = (heap_top - heap_bottom) * sizeof(uint32_t);
    _heap = tlsf_create_with_pool(heap_bottom, size, 64 * 1024 * 1024);
    _ram_pool = tlsf_get_pool(_heap);
    if (_psram_size > 0) {
        _psram_pool = tlsf_add_pool(_heap, (void *)0x11000004, _psram_size - 4);
    }
}

void *port_malloc(size_t size, bool dma_capable) {
    printf("malloc %d\r\n", size);
    void *block = tlsf_malloc(_heap, size);
    printf("malloc %d -> %p\r\n", size, block);
    return block;
}

void port_free(void *ptr) {
    tlsf_free(_heap, ptr);
}

void *port_realloc(void *ptr, size_t size) {
    return tlsf_realloc(_heap, ptr, size);
}

static void max_size_walker(void *ptr, size_t size, int used, void *user) {
    size_t *max_size = (size_t *)user;
    if (!used && *max_size < size) {
        *max_size = size;
    }
}

size_t port_heap_get_largest_free_size(void) {
    size_t max_size = 0;
    printf("largest free\r\n");
    tlsf_walk_pool(_ram_pool, max_size_walker, &max_size);
    if (_psram_pool != NULL) {
        tlsf_walk_pool(_psram_pool, max_size_walker, &max_size);
    }
    // IDF does this. Not sure why.
    return tlsf_fit_size(_heap, max_size);
}