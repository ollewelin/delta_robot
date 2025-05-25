#include "fsl_clock.h"
#include "fsl_pwm.h"
#include "fsl_lpspi.h"
#include "fsl_gpio.h"

#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "peripherals.h"

/* ---- SPI one-byte pulse ------------------------------------------------ */
#include "fsl_lpspi.h"

/* prepare once (after BootPeripherals) ---------------------------------- */
lpspi_transfer_t xfer =
{
    .txData      = NULL,              /* we’ll fill at run-time          */
    .rxData      = NULL,
    .dataSize    = 1,                 /* one byte                        */
    .configFlags = kLPSPI_MasterPcs0 | kLPSPI_MasterPcsContinuous
};



/* Adjusted names --------------------------------------------------- */
#define PWM_BASE   PWM1
#define PWM_SM     kPWM_Module_0
#define SPI_BASE   LPSPI4

#define CS_GPIO    GPIO2          /* B0_10 belongs to GPIO2 port */
#define CS_PIN     10U            /* B0_10 */

static void spin_ms(uint32_t ms)
{
    SDK_DelayAtLeastUs(ms * 1000U, SystemCoreClock);   /* bare-metal delay */
}

int main(void)
{
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    gpio_pin_config_t cs_cfg = {kGPIO_DigitalOutput, 1};
    GPIO_PinInit(CS_GPIO, CS_PIN, &cs_cfg);

    pwm_signal_param_t sig = {
        .pwmChannel       = kPWM_PwmA,
        .level            = kPWM_HighTrue,
        .dutyCyclePercent = 10,
    };
    PWM_SetupPwm(PWM_BASE, PWM_SM, &sig, 1,
                 kPWM_SignedCenterAligned, 20000U,
                 CLOCK_GetFreq(kCLOCK_IpgClk));

  //PWM_StartTimer(PWM_BASE, kPWM_Control_Module_0 | kPWM_Control_LoadOK);
    PWM_SetPwmLdok(PWM_BASE, kPWM_Control_Module_0, true);  /* latch registers */
    PWM_StartTimer (PWM_BASE, kPWM_Control_Module_0);       /* run counter     */



    while (1)
    {
        static uint8_t duty = 10;
        duty = (duty == 10) ? 70 : 10;
        PWM_UpdatePwmDutycycle(PWM_BASE, PWM_SM,
                               kPWM_PwmA, kPWM_SignedCenterAligned, duty);

        GPIO_PortClear(CS_GPIO, 1U << CS_PIN);
        uint8_t junk = 0x55;
        //LPSPI_WriteBlocking(SPI_BASE, &junk, 1);

        xfer.txData = &junk;
        LPSPI_MasterTransferBlocking(SPI_BASE, &xfer);

        GPIO_PortSet  (CS_GPIO, 1U << CS_PIN);

        spin_ms(1000);
    }
}
