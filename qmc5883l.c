#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "qmc5883l.h"

/* Register numbers */
#define QMC5883L_REG_XOUT_LSB	 		0x00
#define QMC5883L_REG_XOUT_MSB 	 		0x01
#define QMC5883L_REG_YOUT_LSB 	 		0x02
#define QMC5883L_REG_YOUT_MSB 	 		0x03
#define QMC5883L_REG_ZOUT_LSB 	 		0x04
#define QMC5883L_REG_ZOUT_MSB 	 		0x05
#define QMC5883L_REG_STATUS 	 		0x06
#define QMC5883L_REG_TEMP_LSB 	 		0x07
#define QMC5883L_REG_TEMP_MSB 	 		0x08
#define QMC5883L_REG_CONFIG 	 		0x09
#define QMC5883L_REG_CONFIG2 	 		0x0A
#define QMC5883L_REG_RESET  	 		0x0B
#define QMC5883L_REG_RESERVED  	 		0x0C
#define QMC5883L_REG_CHIP_ID  	 		0x0D

typedef struct qmc5883l {
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
	float 						mag_scaling_factor;			/*!< Magnetometer scaling factor */
} qmc5883l_t;

qmc5883l_handle_t qmc5883l_init(void)
{
	qmc5883l_handle_t handle = calloc(1, sizeof(qmc5883l_t));
	if (handle == NULL)
	{
		return NULL;
	}

	return handle;
}

qmc5883l_status_t qmc5883l_set_config(qmc5883l_handle_t handle, qmc5883l_cfg_t config)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return QMC5883L_STATUS_INVALID_ARG;
	}

	handle->range = config.range;
	handle->opr_mode = config.opr_mode;
	handle->data_rate = config.data_rate;
	handle->sample_rate = config.sample_rate;
	handle->intr_en = config.intr_en;
	handle->hard_bias_x = config.hard_bias_x;
	handle->hard_bias_y = config.hard_bias_y;
	handle->hard_bias_z = config.hard_bias_z;
	handle->soft_bias_c11 = config.soft_bias_c11;
	handle->soft_bias_c12 = config.soft_bias_c12;
	handle->soft_bias_c13 = config.soft_bias_c13;
	handle->soft_bias_c21 = config.soft_bias_c21;
	handle->soft_bias_c22 = config.soft_bias_c22;
	handle->soft_bias_c23 = config.soft_bias_c23;
	handle->soft_bias_c31 = config.soft_bias_c31;
	handle->soft_bias_c32 = config.soft_bias_c32;
	handle->soft_bias_c33 = config.soft_bias_c33;
	handle->i2c_send = config.i2c_send;
	handle->i2c_recv = config.i2c_recv;
	handle->delay = config.delay;

	return QMC5883L_STATUS_SUCCESS;
}

qmc5883l_status_t qmc5883l_config(qmc5883l_handle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return QMC5883L_STATUS_INVALID_ARG;
	}

	uint8_t data[1] = {0};

	data[0] = 0x01;
	handle->i2c_send(QMC5883L_REG_RESET, data, 1);

	handle->delay(500);

	data[0] = (handle-> opr_mode << 0) | (handle->data_rate << 2) | (handle->range << 4) | (handle->sample_rate << 6);
	handle->i2c_send(QMC5883L_REG_CONFIG, data, 1);



	return QMC5883L_STATUS_SUCCESS;
}

qmc5883l_status_t qmc5883l_get_mag_raw(qmc5883l_handle_t handle, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return QMC5883L_STATUS_INVALID_ARG;
	}

	uint8_t mag_raw_data[7] = {0};

	handle->i2c_recv(QMC5883L_REG_XOUT_LSB, mag_raw_data, 6);

	*raw_x = (int16_t)((mag_raw_data[1] << 8) | mag_raw_data[0]);
	*raw_y = (int16_t)((mag_raw_data[3] << 8) | mag_raw_data[2]);
	*raw_z = (int16_t)((mag_raw_data[5] << 8) | mag_raw_data[4]);

	return QMC5883L_STATUS_SUCCESS;
}

qmc5883l_status_t qmc5883l_get_mag_calib(qmc5883l_handle_t handle, float *calib_x, float *calib_y, float *calib_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return QMC5883L_STATUS_INVALID_ARG;
	}

	int16_t raw_x, raw_y, raw_z;
	float xm_off, ym_off, zm_off;

	qmc5883l_get_mag_raw(handle, &raw_x, &raw_y, &raw_z);

	xm_off  = raw_x - handle->hard_bias_x;
	ym_off  = raw_y - handle->hard_bias_y;
	zm_off  = raw_z - handle->hard_bias_z;

	*calib_x = xm_off *  handle->soft_bias_c11 + ym_off *  handle->soft_bias_c12  + zm_off *  handle->soft_bias_c13;
	*calib_y = xm_off *  handle->soft_bias_c21 + ym_off *  handle->soft_bias_c22  + zm_off *  handle->soft_bias_c23;
	*calib_z = xm_off *  handle->soft_bias_c31 + ym_off *  handle->soft_bias_c32  + zm_off *  handle->soft_bias_c33;

	return QMC5883L_STATUS_SUCCESS;
}

qmc5883l_status_t qmc5883l_get_mag_scale(qmc5883l_handle_t handle, float *scale_x, float *scale_y, float *scale_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return QMC5883L_STATUS_INVALID_ARG;
	}

	return QMC5883L_STATUS_SUCCESS;
}

qmc5883l_status_t qmc5883l_set_mag_bias(qmc5883l_handle_t handle, int16_t bias_x, int16_t bias_y, int16_t bias_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return QMC5883L_STATUS_INVALID_ARG;
	}

	handle->hard_bias_x = bias_x;
	handle->hard_bias_y = bias_y;
	handle->hard_bias_z = bias_z;

	return QMC5883L_STATUS_SUCCESS;
}

qmc5883l_status_t qmc5883l_get_mag_bias(qmc5883l_handle_t handle, int16_t *bias_x, int16_t *bias_y, int16_t *bias_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return QMC5883L_STATUS_INVALID_ARG;
	}

	*bias_x = handle->hard_bias_x;
	*bias_y = handle->hard_bias_y;
	*bias_z = handle->hard_bias_z;

	return QMC5883L_STATUS_SUCCESS;
}

