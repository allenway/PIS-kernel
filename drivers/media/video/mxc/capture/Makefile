ifeq ($(CONFIG_VIDEO_MXC_IPU_CAMERA),y)
	obj-$(CONFIG_VIDEO_MXC_CAMERA) += mxc_v4l2_capture.o 
	obj-$(CONFIG_MXC_IPU_PRP_VF_SDC) += ipu_prp_vf_sdc.o ipu_prp_vf_sdc_bg.o
	obj-$(CONFIG_MXC_IPU_DEVICE_QUEUE_SDC) += ipu_fg_overlay_sdc.o ipu_bg_overlay_sdc.o
	obj-$(CONFIG_MXC_IPU_PRP_ENC) += ipu_prp_enc.o ipu_still.o
	obj-$(CONFIG_MXC_IPU_CSI_ENC) += ipu_csi_enc.o ipu_still.o
endif

obj-$(CONFIG_VIDEO_MXC_CSI_CAMERA) += fsl_csi.o csi_v4l2_capture.o

mx27_capture-objs := mx27_prphw.o mx27_prpsw.o emma_v4l2_capture.o
obj-$(CONFIG_VIDEO_MXC_EMMA_CAMERA) += mx27_csi.o mx27_capture.o

camera_sensor_clock-objs := sensor_clock.o
obj-$(CONFIG_MXC_CAMERA_SENSOR_CLK) += camera_sensor_clock.o

ov2640_camera-objs := ov2640.o
obj-$(CONFIG_MXC_CAMERA_OV2640) += ov2640_camera.o

ov3640_camera-objs := ov3640.o
ifneq ($(wildcard drivers/media/video/mxc/capture/ov3640.c),)
obj-$(CONFIG_MXC_CAMERA_OV3640) += ov3640_camera.o
else
obj-$(CONFIG_MXC_CAMERA_OV3640) += ov3640_camera.module
endif

