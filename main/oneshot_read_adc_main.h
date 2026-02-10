//ADC1 Channels
#define EXAMPLE_ADC1_CHAN0          ADC_CHANNEL_3
//#define EXAMPLE_ADC1_CHAN1          ADC_CHANNEL_3
#define EXAMPLE_ADC_ATTEN           ADC_ATTEN_DB_12

adc_oneshot_unit_handle_t adc1_handle;
bool do_calibration1_chan0;
adc_cali_handle_t adc1_cali_chan0_handle ;
adc_cali_handle_t adc1_cali_chan1_handle ;

void adc_config(void);

