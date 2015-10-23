#ifndef _BSP_H
#define _BSP_H

/**
 * @brief Prende un led
 *
 * @param led Led a prender
 */
void led_on(uint8_t led);

/**
 * @brief Apaga un led
 *
 * @param led Led a apagar
 */
void led_off(uint8_t led);

/**
 * @brief Conmuta un led
 *
 * @param led Led a apagar
 */
void led_toggle(uint8_t led);

/**
 * @brief Delay de x ms
 *
 * @param x cantidad de ms
 */
void bsp_delay_ms(uint16_t x);

/**
 * @brief Devuelve el estado del boton
 *  *
 * @param boton Boton a consultar
 */
uint8_t sw_getState(uint16_t boton);

/**
 * @brief Devuelve el estado del boton
 *  *
 * @param boton Boton a consultar
 */
void uart_tx (char* data);

/**
 * @brief Devuelve el estado del boton
 *  *
 * @param boton Boton a consultar
 */
char uart_rx (void);

/**
 * @brief Inicializacion de los servicios de BSP
 */
void bsp_init();

#endif
