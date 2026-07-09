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
    uint32_t timeout_counter = 0;

    // 1. Safety first: Disable Main Output Enable (MOE)
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    //__HAL_TIM_MOE_ENABLE(&htim1);
    // 2. Start the ADCs in Injected mode for calibration
    __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_JEOC);
    
    HAL_ADCEx_InjectedStart(&hadc1);
    HAL_ADCEx_InjectedStart(&hadc2);
    // Force update event to latch timer settings
    htim1.Instance->EGR = TIM_EGR_UG;
    htim1.Instance->CCER |= TIM_CCER_CC4E;
    // 3. Start the timer to begin generating hardware triggers via CH4
    HAL_TIM_Base_Start(&htim1);
    
    
    // 4. Calibration Loop with Timeout Protection
    for (uint32_t i = 0; i < num_samples; i++) {
        timeout_counter = 0xFFFF; 
        
        while ((ADC1->SR & ADC_SR_JEOC) == 0) {
            if (--timeout_counter == 0) {
                // Something is wrong with the timer trigger. Don't brick silently.
                Error_Handler(); 
            }
        }

        // Clean flag clearing via HAL macro (avoids RMW race conditions)
        __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);

        sum_a += ADC1->JDR1;
        sum_b += ADC2->JDR1;
    }

    // 5. Calculate true offsets
    offset_ia = (float)sum_a / (float)num_samples;
    offset_ib = (float)sum_b / (float)num_samples;
    
    // 6. Clean up everything used for calibration
    HAL_TIM_Base_Stop(&htim1);
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