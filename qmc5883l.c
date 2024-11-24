#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "qmc5883l.h"

typedef struct qmc5883l {
	qmc5883l_range_t  			range;						/*!< Sensor field range */
	qmc5883l_opr_mode_t  		opr_mode;  					/*!< Operating mode */
	qmc5883l_data_rate_t  		data_rate;					/*!< Data output rate */
	qmc5883l_sample_rate_t 		sample_rate;				/*!< Over sample rate */
	int 						mag_bias_x;  				/*!< Magnetometer bias x axis */
	int 						mag_bias_y;  				/*!< Magnetometer bias y axis */
	int 						mag_bias_z;  				/*!< Magnetometer bias z axis */
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

err_code_t qmc5883l_set_config(qmc5883l_handle_t handle, qmc5883l_cfg_t config)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	handle->range = config.range;
	handle->opr_mode = config.opr_mode;
	handle->data_rate = config.data_rate;
	handle->sample_rate = config.sample_rate;
	handle->mag_bias_x = config.mag_bias_x;
	handle->mag_bias_y = config.mag_bias_y;
	handle->mag_bias_z = config.mag_bias_z;
	handle->i2c_send = config.i2c_send;
	handle->i2c_recv = config.i2c_recv;
	handle->delay = config.delay;

	return ERR_CODE_SUCCESS;
}

err_code_t qmc5883l_config(qmc5883l_handle_t handle)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t qmc5883l_get_mag_raw(qmc5883l_handle_t handle, int16_t *raw_x, int16_t *raw_y, int16_t *raw_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t qmc5883l_get_mag_calib(qmc5883l_handle_t handle, int16_t *calib_x, int16_t *calib_y, int16_t *calib_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t qmc5883l_get_mag_scale(qmc5883l_handle_t handle, float *scale_x, float *scale_y, float *scale_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	return ERR_CODE_SUCCESS;
}

err_code_t qmc5883l_set_mag_bias(qmc5883l_handle_t handle, int16_t bias_x, int16_t bias_y, int16_t bias_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	handle->mag_bias_x = bias_x;
	handle->mag_bias_y = bias_y;
	handle->mag_bias_z = bias_z;

	return ERR_CODE_SUCCESS;
}

err_code_t qmc5883l_get_mag_bias(qmc5883l_handle_t handle, int16_t *bias_x, int16_t *bias_y, int16_t *bias_z)
{
	/* Check if handle structure is NULL */
	if (handle == NULL)
	{
		return ERR_CODE_NULL_PTR;
	}

	*bias_x = handle->mag_bias_x;
	*bias_y = handle->mag_bias_y;
	*bias_z = handle->mag_bias_z;

	return ERR_CODE_SUCCESS;
}

err_code_t qmc5883l_auto_calib(qmc5883l_handle_t handle)
{
	return ERR_CODE_SUCCESS;
}
