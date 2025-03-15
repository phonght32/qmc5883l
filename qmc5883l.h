// MIT License

// Copyright (c) 2024 phonght32

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef __QMC5883L_H__
#define __QMC5883L_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "err_code.h"

#define QMC5883L_I2C_ADDR		(0x0D)

typedef err_code_t (*qmc5883l_func_i2c_send)(uint8_t reg_addr, uint8_t *buf_send, uint16_t len);
typedef err_code_t (*qmc5883l_func_i2c_recv)(uint8_t reg_addr, uint8_t *buf_recv, uint16_t len);
typedef void (*qmc5883l_func_delay)(uint32_t ms);

/**
 * @brief   Handle structure.
 */
typedef struct qmc5883l *qmc5883l_handle_t;

/**
 * @brief   Operating mode.
 */
typedef enum {
	QMC5883L_OPR_MODE_STANDBY = 0,							/*!< Standby */
	QMC5883L_OPR_MODE_CONTINUOUS							/*!< Continuous measurement */
} qmc5883l_opr_mode_t;

/**
 * @brief   Sensor field range.
 */
typedef enum {
	QMC5883L_RANGE_2G = 0,								/*!< +/- 2G */
	QMC5883L_RANGE_8G,									/*!< +/- 8G */
} qmc5883l_range_t;

/**
 * @brief   Data output rate.
 */
typedef enum {
	QMC5883L_DATA_RATE_10HZ = 0,							/*!< 10 Hz */
	QMC5883L_DATA_RATE_50HZ,								/*!< 50 Hz */
	QMC5883L_DATA_RATE_100HZ,								/*!< 100 Hz */
	QMC5883L_DATA_RATE_200HZ								/*!< 200 Hz */
} qmc5883l_data_rate_t;

/**
 * @brief   Over sample rate.
 */
typedef enum {
	QMC5883L_SAMPLE_RATE_512 = 0,							/*!< 512 sample */
	QMC5883L_SAMPLE_RATE_256,								/*!< 256 samples */
	QMC5883L_SAMPLE_RATE_128,								/*!< 128 samples */
	QMC5883L_SAMPLE_RATE_64									/*!< 64 samples */
} qmc5883l_sample_rate_t;

typedef enum {
	QMC5883L_INTERRUPT_ENABLE = 0,
	QMC5883L_INTERRUPT_DISABLE
} qmc5883l_intr_en_t;

/**
 * @brief   Configuration structure.
 */
typedef struct {
	qmc5883l_range_t  			range;						/*!< Sensor field range */
	qmc5883l_opr_mode_t  		opr_mode;  					/*!< Operating mode */
	qmc5883l_data_rate_t  		data_rate;					/*!< Data output rate */
	qmc5883l_sample_rate_t 		sample_rate;				/*!< Over sample rate */
	qmc5883l_intr_en_t 			intr_en;					/*!< Enable interupt */
	float 						hard_bias_x;  				/*!< Magnetometer hard iron bias x axis */
	float 						hard_bias_y;  				/*!< Magnetometer hard iron bias y axis */
	float 						hard_bias_z;  				/*!< Magnetometer hard iron bias z axis */
	float 						soft_bias_c11;				/*!< Magnetometer soft iron bias */
	float 						soft_bias_c12;				/*!< Magnetometer soft iron bias */
	float 						soft_bias_c13;				/*!< Magnetometer soft iron bias */
	float 						soft_bias_c21;				/*!< Magnetometer soft iron bias */
	float 						soft_bias_c22;				/*!< Magnetometer soft iron bias */
	float 						soft_bias_c23;				/*!< Magnetometer soft iron bias */
	float 						soft_bias_c31;				/*!< Magnetometer soft iron bias */
	float 						soft_bias_c32;				/*!< Magnetometer soft iron bias */
	float 						soft_bias_c33;				/*!< Magnetometer soft iron bias */
	qmc5883l_func_i2c_send      i2c_send;        			/*!< QMC5883L send bytes */
	qmc5883l_func_i2c_recv      i2c_recv;         			/*!< QMC5883L receive bytes */
	qmc5883l_func_delay         delay;                 		/*!< QMC5883L delay function */
} qmc5883l_cfg_t;

/*
 * @brief   Initialize QMC5883L with default parameters.
 *
 * @note    This function must be called first.
 *
 * @param   None.
 *
 * @return
 *      - Handle structure: Success.
 *      - Others:           Fail.
 */
qmc5883l_handle_t qmc5883l_init(void);

/*
 * @brief   Set configuration parameters.
 *
 * @param 	handle Handle structure.
 * @param   config Configuration structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t qmc5883l_set_config(qmc5883l_handle_t handle, qmc5883l_cfg_t config);

/*
 * @brief   Configure QMC5883L to run.
 *
 * @param 	handle Handle structure.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t qmc5883l_config(qmc5883l_handle_t handle);

/*
 * @brief   Get magnetometer raw value.
 *
 * @param   handle Handle structure.
 * @param   raw_x Raw value x axis.
 * @param   raw_y Raw value y axis.
 * @param   raw_z Raw value z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t qmc5883l_get_mag_raw(qmc5883l_handle_t handle, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z);

/*
 * @brief   Get magnetometer calibrated data.
 *
 * @param   handle Handle structure.
 * @param   calib_x Calibrated data x axis.
 * @param   calib_y Calibrated data y axis.
 * @param   calib_z Calibrated data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t qmc5883l_get_mag_calib(qmc5883l_handle_t handle, float *calib_x, float *calib_y, float *calib_z);

/*
 * @brief   Get magnetometer scaled data.
 *
 * @param   handle Handle structure.
 * @param   scale_x Scaled data x axis.
 * @param   scale_y Scaled data y axis.
 * @param   scale_z Scaled data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t qmc5883l_get_mag_scale(qmc5883l_handle_t handle, float *scale_x, float *scale_y, float *scale_z);

/*
 * @brief   Set magnetometer bias data.
 *
 * @param   handle Handle structure.
 * @param   bias_x Bias data x axis.
 * @param   bias_y Bias data y axis.
 * @param   bias_z Bias data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t qmc5883l_set_mag_bias(qmc5883l_handle_t handle, int16_t bias_x, int16_t bias_y, int16_t bias_z);

/*
 * @brief   Set magnetometer bias data.
 *
 * @param   handle Handle structure.
 * @param   bias_x Bias data x axis.
 * @param   bias_y Bias data y axis.
 * @param   bias_z Bias data z axis.
 *
 * @return
 *      - ERR_CODE_SUCCESS: Success.
 *      - Others:           Fail.
 */
err_code_t qmc5883l_get_mag_bias(qmc5883l_handle_t handle, int16_t *bias_x, int16_t *bias_y, int16_t *bias_z);


#ifdef __cplusplus
}
#endif

#endif /* __QMC5883L_H__ */
