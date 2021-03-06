/* =========== CONFIGURATION OF THE STK500v2 BOOTLOADER ========================
 */

/* ----------- configuration of the bootloader features ------------------------
 *
 * Uncomment the following REMOVE_* lines to disable features in order to
 * save code space.
 */

/* disable program lock bits
 */
//#define REMOVE_PROGRAM_LOCK_BIT_SUPPORT

/* disable reading lock and fuse bits
 */
//#define REMOVE_READ_LOCK_FUSE_BIT_SUPPORT

/* disable processing of SPI_MULTI commands (SPI MULTI command only used by AVRdude)
 */
//#define REMOVE_CMD_SPI_MULTI

/* no LED to show active bootloader
 */
#define REMOVE_BOOTLOADER_LED

/* disable internal pull-up, use external pull-up resistor
 */
#define REMOVE_PROG_PIN_PULLUP

/* uncomment to disable "pin mode". disables usage of PROG_PIN
 */
//#define REMOVE_PROG_PIN_ENTER

/* uncomment to disable "forced enter"
 */
#define REMOVE_FORCED_MODE_ENTER

/* uncomment to enable handling of WDT resets
 */
#define REMOVE_WDT_RESET

/* uncomment to disable periodic pinging of WDT
 */
//#define REMOVE_WDT_PING

/* Uncomment to allow protocol to leave the bootloader and jump to the
 * application after programming.
 */
#define ENABLE_LEAVE_BOOTLADER


/* ----------- configuration of the hardware -----------------------------------
 */

/* Pin "PROG_PIN" on port "PROG_PORT" has to be pulled low
 * (active low) to start the bootloader
 *
 *  - uncomment #define REMOVE_PROG_PIN_PULLUP if using an external pullup
 *  - uncomment #define REMOVE_PROG_PIN_ENTER if now boot pin is available
 */
#ifndef REMOVE_PROG_PIN_ENTER
#  define PROG_PORT  PORTD
#  define PROG_DDR   DDRD
#  define PROG_IN    PIND
#  define PROG_PIN   PIND7
#endif

/* Active-low LED on pin "PROGLED_PIN" on port "PROGLED_PORT"
 * indicates that bootloader is active
 */
#ifndef REMOVE_BOOTLOADER_LED
#  define PROGLED_PORT PORTD
#  define PROGLED_DDR  DDRD
#  define PROGLED_PIN  PD7
#endif


/* define which UART channel will be used, if device with two UARTs is used
 */
#define USE_USART0        // use first USART
//#define USE_USART1      // use second USART


/* UART Baudrate, AVRStudio AVRISP only accepts 115200 bps
 */
#define BAUDRATE 115200

/*
 *  Enable (1) or disable (0) USART double speed operation
 */
#define UART_BAUDRATE_DOUBLE_SPEED 0


/* To avoid hardware damage while the bootloader is active, it's possible to
 * define the DDR and PORT register of every port of the MCU. Select the
 * port which must have another than the powerup defaults.
 *
 * To enable the setup of an port, uncomment the according VAL_PORT_* and
 * DIR_PORT_* !
 */

#define DIR_PORT_A  0b00000000
#define VAL_PORT_A  0b00000000

#define DIR_PORT_B  0b00000000
#define VAL_PORT_B  0b00000000

#define DIR_PORT_C  0b00000000
#define VAL_PORT_C  0b00000000

#define DIR_PORT_D  0b00101010
#define VAL_PORT_D  0b00001010

#define DIR_PORT_E  0b00000111
#define VAL_PORT_E  0b00000000

#define DIR_PORT_F  0b00100000
#define VAL_PORT_F  0b11111111

#define DIR_PORT_G  0b00000000
#define VAL_PORT_G  0b11111111

#define DIR_PORT_H  0b00000000
#define VAL_PORT_H  0b11111111

#define DIR_PORT_J  0b00000000
#define VAL_PORT_J  0b11111111

#define DIR_PORT_K  0b00000000
#define VAL_PORT_K  0b11111111

#define DIR_PORT_L  0b00000000
#define VAL_PORT_L  0b11111111

/* =========== END OF CONFIGURATION ========================================= */
