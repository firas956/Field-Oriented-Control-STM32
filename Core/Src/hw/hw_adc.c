#include "hw/hw_adc.h"
#include "hw/hw_pwm.h"

// Bring handles in cleanly at the file scope
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim1; 

static float offset_ia = 3000.0f;
static float offset_ib = 3000.0f;

void HW_ADC_Init(void){
    uint32_t sum_a = 0;
    uint32_t sum_b = 0;
    const uint32_t num_samples = 1000;
    uint32_t timeout_counter;

    // 1. CH1/CH2/CH3 are safely off right now (not started in main yet).
    // We will ONLY start CH4 to generate the trigger for calibration.

    // Ensure interrupts don't fire during polling calibration
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);
    
    // 2. Start the ADCs in Injected mode
    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);
    
    // Force update event to latch timer settings
    htim1.Instance->EGR = TIM_EGR_UG;
    
    // 3. Start TIM1 CH4 in PWM mode to generate continuous hardware triggers.
    // This safely handles MOE and CCER properly for TIM1 without turning on CH1-3.
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    
    // 4. Calibration Loop with Timeout Protection
    for (uint32_t i = 0; i < num_samples; i++) {
        timeout_counter = 0xFFFF; 
        
        // Wait for End of Injected Conversion (JEOC)
        while ((ADC1->SR & ADC_SR_JEOC) == 0) {
            if (--timeout_counter == 0) {
                // Something is wrong with the timer trigger. Don't brick silently.
                Error_Handler(); 
            }
        }

        // Clean flag clearing for BOTH ADCs in dual mode to prevent overruns
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
        __HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC);

        sum_a += ADC1->JDR1;
        sum_b += ADC2->JDR1;
    }

    // 5. Calculate true offsets
    offset_ia = (float)sum_a / (float)num_samples;
    offset_ib = (float)sum_b / (float)num_samples;
    
    // 6. Clean up everything used for calibration before moving to main operation
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    HAL_ADCEx_InjectedStop(&hadc1);
    HAL_ADCEx_InjectedStop(&hadc2);
}

void HW_ADC_ReadCurrents(float *i_a, float *i_b){
    // Read registers directly for performance inside the ISR
    uint32_t raw_a = ADC1->JDR1;
    uint32_t raw_b = ADC2->JDR1;

    *i_a = ((float)raw_a - offset_ia) * ADC_RAW_TO_AMPS;
    *i_b = ((float)raw_b - offset_ib) * ADC_RAW_TO_AMPS;
}