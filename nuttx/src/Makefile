include $(APPDIR)/Make.defs

PROGNAME = $(CONFIG_APPLICATION_ENGINE_SPEED_PROGNAME)
PRIORITY = $(CONFIG_APPLICATION_ENGINE_SPEED_PRIORITY)
STACKSIZE = $(CONFIG_APPLICATION_ENGINE_SPEED_STACKSIZE)
MODULE = $(CONFIG_APPLICATION_ENGINE_SPEED)

GSL_DIR = "gsl-2.7"

MAINSRC = engine_speed.c
CSRCS = sensor_ops.c

CFLAGS += -I$(GSL_DIR)

include $(APPDIR)/Application.mk
